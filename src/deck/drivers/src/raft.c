#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "timers.h"
#include "debug.h"
#include "system.h"
#include "param.h"
#include "raft.h"

#ifndef RAFT_DEBUG_ENABLE
#undef DEBUG_PRINT
#define DEBUG_PRINT
#endif

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

static QueueHandle_t rxQueue;
static QueueHandle_t commandBufferQueue;
static TaskHandle_t raftRxTaskHandle;
static TaskHandle_t raftCommandTaskHandle;
static Raft_Node_t raftNode;
static TimerHandle_t raftElectionTimer;
static TimerHandle_t raftHeartbeatTimer;
static TimerHandle_t raftLogApplyTimer;
static uint16_t raftClientRequestId = 1;
static SemaphoreHandle_t raftRequestIdMutex;
static uint16_t raftLeaderApply;
static uint32_t raftMembers;
static uint8_t raftClusterId;
static Raft_Log_Item_t EMPTY_LOG_ITEM = {
    .term = 0,
    .index = 0,
    .command = {.type = RAFT_LOG_COMMAND_RESERVED, .clientId = UWB_DEST_EMPTY, .requestId = 0} // TODO: init payload
};
static Raft_Config_t EMPTY_CONFIG = {
    .currentConfig = 0,
    .previousConfig = 0,
    .clusterId = RAFT_CLUSTER_ID_EMPTY,
    .clusterSize = 0
};

static bool raftConfigAdd(UWB_Address_t node) {
  raftNode.config.previousConfig = raftNode.config.currentConfig;
  raftNode.config.currentConfig = raftNode.config.currentConfig | (1 << node);
  if (raftNode.config.previousConfig == raftNode.config.currentConfig) {
    DEBUG_PRINT("raftConfigAdd: %u try to add an existing node %u to cluster %u, ignore.\n",
                raftNode.me,
                node,
                raftNode.config.clusterId);
    return false;
  }
  raftNode.config.clusterSize++;
  raftNode.peerVote[node] = false;
  raftNode.nextIndex[node] = raftNode.log.items[raftNode.log.size - 1].index + 1;
  raftNode.matchIndex[node] = 0;
  DEBUG_PRINT("raftConfigAdd: %u add node %u to cluster %u.\n", raftNode.me, node, raftNode.config.clusterId);
  return true;
}

static bool raftConfigRemove(UWB_Address_t node) {
  raftNode.config.previousConfig = raftNode.config.currentConfig;
  raftNode.config.currentConfig = raftNode.config.currentConfig & (~(1 << node));
  if (raftNode.config.previousConfig == raftNode.config.currentConfig) {
    DEBUG_PRINT("raftConfigRemove: %u try to remove an unknown node %u in cluster %u, ignore.\n",
                raftNode.me,
                node,
                raftNode.config.clusterId);
    return false;
  }
  raftNode.config.clusterSize--;
  raftNode.peerVote[node] = false;
  raftNode.nextIndex[node] = raftNode.log.items[raftNode.log.size - 1].index + 1;
  raftNode.matchIndex[node] = 0;
  DEBUG_PRINT("raftConfigRemove: %u remove node %u in cluster %u.\n", raftNode.me, node, raftNode.config.clusterId);
  return true;
}

static bool raftConfigHasPeer(UWB_Address_t peer) {
  return (raftNode.config.currentConfig & (1 << peer));
}

static uint16_t getNextRequestId() {
  xSemaphoreTake(raftRequestIdMutex, M2T(0));
  uint16_t nextId = raftClientRequestId++;
  xSemaphoreGive(raftRequestIdMutex);
  return nextId;
}

static int raftLogFindByIndex(Raft_Log_t *raftLog, uint16_t logIndex) {
  for (int i = raftLog->size - 1; i >= 0; i--) {
    if (raftLog->items[i].index == logIndex) {
      return i;
    }
  }
  return -1;
}

static int raftLogFindMatched(Raft_Log_t *raftLog, uint16_t logIndex, uint16_t logTerm) {
  /* Binary Search */
  int left = -1, right = raftLog->size;
  while (left + 1 != right) {
    int mid = left + (right - left) / 2;
    if (raftLog->items[mid].index == logIndex) {
      if (raftLog->items[mid].term == logTerm) {
        return mid;
      }
      break;
    } else if (raftLog->items[mid].index > logIndex) {
      right = mid;
    } else {
      left = mid;
    }
  }
  return -1;
}

static void raftLogCleanFrom(Raft_Log_t *raftLog, uint16_t itemStartIndex) {
  ASSERT(itemStartIndex >= 0);
  for (int i = itemStartIndex; i < raftLog->size; i++) {
    raftLog->items[i] = EMPTY_LOG_ITEM;
  }
  raftLog->size = itemStartIndex;
}

static int raftLogGetLastLogItemByTerm(Raft_Log_t *raftLog, uint16_t logTerm) {
  /* Binary search, i.e. find last 6 in [1,2,3,3,6,6,6,6,7,8] */
  int left = -1, right = raftLog->size;
  while (left + 1 != right) {
    int mid = left + (right - left) / 2;
    if (raftLog->items[mid].term <= logTerm) {
      left = mid;
    } else if (raftLog->items[mid].term > logTerm) {
      right = mid;
    }
  }
  return left;
}

static void raftLogApply(Raft_Log_t *raftLog, uint16_t logItemIndex) {
  DEBUG_PRINT("raftLogApply: Apply log index = %u, term = %u, clientId = %u, requestId = %u.\n",
              raftLog->items[logItemIndex].index,
              raftLog->items[logItemIndex].term,
              raftLog->items[logItemIndex].command.clientId,
              raftLog->items[logItemIndex].command.requestId);
  uint16_t clientId = raftLog->items[logItemIndex].command.clientId;
  uint16_t requestId = raftLog->items[logItemIndex].command.requestId;
  if (requestId <= raftNode.latestAppliedRequestId[clientId]) {
    DEBUG_PRINT("raftLogApply: %u deduplicate log from %u with same requestId = %u to guarantee EXACTLY_ONCE.\n",
                raftNode.me,
                raftLog->items[logItemIndex].command.clientId,
                raftLog->items[logItemIndex].command.requestId);
    return;
  }
  raftNode.latestAppliedRequestId[clientId] = MAX(raftNode.latestAppliedRequestId[clientId], requestId);
  // TODO: handlers
  Raft_Log_Command_t *command = &raftLog->items[logItemIndex].command;
  switch (command->type) {
    case RAFT_LOG_COMMAND_NO_OPS:break;
    case RAFT_LOG_COMMAND_CONFIG_ADD:
      DEBUG_PRINT("raftLogApply: RAFT_LOG_COMMAND_CONFIG_ADD %u.\n", *(uint16_t *) &command->payload);
      raftConfigAdd(*(uint16_t *) &command->payload);
      break;
    case RAFT_LOG_COMMAND_CONFIG_REMOVE:
      DEBUG_PRINT("raftLogApply: RAFT_LOG_COMMAND_CONFIG_REMOVE %u.\n", *(uint16_t *) &command->payload);
      raftConfigRemove(*(uint16_t *) &command->payload);
      if (!raftConfigHasPeer(raftNode.me)) {
        raftLogCleanFrom(raftLog, 1);
        raftNode.currentLeader = UWB_DEST_EMPTY;
        raftNode.currentState = RAFT_STATE_FOLLOWER;
        raftNode.currentTerm = 0;
        raftNode.lastApplied = 0;
        raftNode.commitIndex = 0;
        for (int i = 0; i <= RAFT_CLUSTER_PEER_NODE_ADDRESS_MAX; i++) {
          raftNode.peerVote[i] = false;
          raftNode.nextIndex[i] = raftNode.log.items[raftNode.log.size - 1].index + 1;
          raftNode.matchIndex[i] = 0;
        }
        raftNode.config = EMPTY_CONFIG;
      }
      break;
    case RAFT_LOG_COMMAND_GET:break;
    case RAFT_LOG_COMMAND_PUT:break;
    default:DEBUG_PRINT("raftLogApply: %u try to apply an unknown type = %u, do nothing.\n",
                        raftNode.me,
                        command->type);
  }
}

static void raftLogAppend(Raft_Log_t *raftLog, uint16_t logTerm, Raft_Log_Command_t *command) {
  if (raftNode.currentState == RAFT_STATE_LEADER && raftLog->size >= RAFT_LOG_SIZE_MAX * 0.75) {
    // TODO: snapshot
  }
  int index = raftLog->size;
  raftLog->items[index].term = logTerm;
  raftLog->items[index].index = raftLog->items[index - 1].index + 1;
  raftLog->items[index].command = *command;
  raftLog->size++;
  DEBUG_PRINT("raftLogAdd: Add log index = %u, term = %u.\n", raftLog->items[index].index, raftLog->items[index].term);
  if ((raftLog->items[index].command.type == RAFT_LOG_COMMAND_CONFIG_ADD
      || raftLog->items[index].command.type == RAFT_LOG_COMMAND_CONFIG_REMOVE)
      && *(uint16_t *) &raftLog->items[index].command.payload == raftNode.me) {
        // 这个判断条件是什么？为何要直接应用该日志
    raftLogApply(raftLog, index);
  }
}
// TODO: check
static void raftUpdateCommitIndex(Raft_Node_t *node) {
  if (node->currentState != RAFT_STATE_LEADER) {
    return;
  }
  for (int peer = 0; peer <= RAFT_CLUSTER_PEER_NODE_ADDRESS_MAX; peer++) {
    if (!raftConfigHasPeer(peer)) {
      continue;
    }
    uint16_t candidateCommitIndex = node->matchIndex[peer];
    if (candidateCommitIndex <= node->commitIndex) {
      continue;
    }
    int index = raftLogFindByIndex(&node->log, candidateCommitIndex);
    if (index == -1) {
      continue;
    }
    Raft_Log_Item_t logItem = node->log.items[index];
    /* Leader can only commit logs generated in its term, logs generated in preceding terms are committed implicitly. */
    if (logItem.term != raftNode.currentTerm) {
      DEBUG_PRINT(
          "raftUpdateCommitIndex: Leader can only commit logs generated in its term, logs generated in preceding terms are committed implicitly.\n");
      continue;
    }
    uint8_t count = 0;
    for (int i = 0; i <= RAFT_CLUSTER_PEER_NODE_ADDRESS_MAX; i++) {
      if (i != node->me && raftConfigHasPeer(i) && node->matchIndex[i] >= candidateCommitIndex) {
        count++;
      }
    }
    if (count >= raftNode.config.clusterSize / 2) {
      DEBUG_PRINT("raftUpdateCommitIndex: %u update commit index from %u to %u.\n",
                  node->me,
                  node->commitIndex,
                  MAX(node->commitIndex, candidateCommitIndex));
      node->commitIndex = MAX(node->commitIndex, candidateCommitIndex);
    }
  }
}

static void convertToFollower(Raft_Node_t *node) {
  node->currentState = RAFT_STATE_FOLLOWER;
  node->currentLeader = UWB_DEST_EMPTY;
  node->voteFor = RAFT_VOTE_FOR_NO_ONE;
  for (int i = 0; i <= RAFT_CLUSTER_PEER_NODE_ADDRESS_MAX; i++) {
    node->peerVote[i] = false;
  }
  node->lastHeartbeatTime = xTaskGetTickCount();
}

static void convertToLeader(Raft_Node_t *node) {
  node->currentState = RAFT_STATE_LEADER;
  node->currentLeader = node->me;
  for (int peer = 0; peer <= RAFT_CLUSTER_PEER_NODE_ADDRESS_MAX; peer++) {
    node->peerVote[peer] = false;
  }
  /* Append an empty log entry (NO_OPS) for implicitly commit preceding logs. */
  Raft_Log_Command_t noOpsLog = {
      .type = RAFT_LOG_COMMAND_NO_OPS,
      .requestId = 0,
      .clientId = node->me,
      // TODO: init empty payload
  };
  raftLogAppend(&node->log, node->currentTerm, &noOpsLog);
}

static void convertToCandidate(Raft_Node_t *node) {
  node->currentTerm++;
  node->currentState = RAFT_STATE_CANDIDATE;
  node->currentLeader = UWB_DEST_EMPTY;
  for (int peer = 0; peer <= RAFT_CLUSTER_PEER_NODE_ADDRESS_MAX; peer++) {
    node->peerVote[peer] = false;
  }
  node->voteFor = raftNode.me;
  node->lastHeartbeatTime = xTaskGetTickCount();
}

static void raftApplyLog() {
  int startIndex = raftNode.lastApplied + 1;
  for (int i = startIndex; i <= raftNode.commitIndex; i++) {
    raftNode.lastApplied++;
    raftLogApply(&raftNode.log, i);
  }
}

static void raftHeartbeatTimerCallback(TimerHandle_t timer) {
//  DEBUG_PRINT("raftHeartbeatTimerCallback: %u trigger heartbeat timer at %lu.\n", raftNode.me, xTaskGetTickCount());
  xSemaphoreTake(raftNode.mu, portMAX_DELAY);
  printRaftConfig(raftNode.config);
  printRaftLog(&raftNode.log);
  if (raftNode.currentState == RAFT_STATE_LEADER) {
    for (int peer = 0; peer <= RAFT_CLUSTER_PEER_NODE_ADDRESS_MAX; peer++) {
      if (peer != raftNode.me && raftConfigHasPeer(peer) && raftConfigHasPeer(peer)) {
        raftSendAppendEntries(peer);
        vTaskDelay(M2T(5));
      }
    }
  }
  xSemaphoreGive(raftNode.mu);
}

static void raftElectionTimerCallback(TimerHandle_t timer) {
//  DEBUG_PRINT("raftElectionTimerCallback: %u trigger election timer at %lu, lastHeartbeat = %lu.\n",
//              raftNode.me,
//              xTaskGetTickCount(),
//              raftNode.lastHeartbeatTime);
  xSemaphoreTake(raftNode.mu, portMAX_DELAY);
  Time_t curTime = xTaskGetTickCount();
  if (raftNode.currentState != RAFT_STATE_LEADER) {
    if ((curTime - raftNode.lastHeartbeatTime) > RAFT_ELECTION_TIMEOUT) {
      DEBUG_PRINT("raftLogApplyTimerCallback: %u timeout in term %u, commitIndex = %u.\n",
                  raftNode.me,
                  raftNode.currentTerm,
                  raftNode.commitIndex);
      convertToCandidate(&raftNode);
      for (int peer = 0; peer <= RAFT_CLUSTER_PEER_NODE_ADDRESS_MAX; peer++) {
        if (peer != raftNode.me && raftConfigHasPeer(peer)) {
          raftSendRequestVote(peer);
          vTaskDelay(M2T(5));
        }
      }
    }
  } else {
    raftNode.lastHeartbeatTime = xTaskGetTickCount();
  }
  xSemaphoreGive(raftNode.mu);
}

static void raftLogApplyTimerCallback(TimerHandle_t timer) {
//  DEBUG_PRINT("raftLogApplyTimerCallback: %u trigger log apply timer at %lu.\n", raftNode.me, xTaskGetTickCount());
  xSemaphoreTake(raftNode.mu, portMAX_DELAY);
  raftApplyLog();
  xSemaphoreGive(raftNode.mu);
}

static void raftRxTask() {
  systemWaitStart();

  UWB_Data_Packet_t dataRxPacket;
  while (1) {
    if (uwbReceiveDataPacketBlock(UWB_DATA_MESSAGE_RAFT, &dataRxPacket)) {
      xSemaphoreTake(raftNode.mu, portMAX_DELAY);
      UWB_Address_t peer = dataRxPacket.header.srcAddress;
      uint8_t type = dataRxPacket.payload[0];
      switch (type) {
        case RAFT_REQUEST_VOTE:
          raftProcessRequestVote(peer, (Raft_Request_Vote_Args_t *) dataRxPacket.payload);
          break;
        case RAFT_REQUEST_VOTE_REPLY:
          raftProcessRequestVoteReply(peer, (Raft_Request_Vote_Reply_t *) dataRxPacket.payload);
          break;
        case RAFT_APPEND_ENTRIES:
          raftProcessAppendEntries(peer, (Raft_Append_Entries_Args_t *) dataRxPacket.payload);
          break;
        case RAFT_APPEND_ENTRIES_REPLY:
          raftProcessAppendEntriesReply(peer, (Raft_Append_Entries_Reply_t *) dataRxPacket.payload);
          break;
        case RAFT_COMMAND_REQUEST:
          raftProcessCommand(peer, (Raft_Command_Args_t *) dataRxPacket.payload);
          break;
        case RAFT_COMMAND_REPLY:
          raftProcessCommandReply(peer, (Raft_Command_Reply_t *) dataRxPacket.payload);
          break;
        default:
          DEBUG_PRINT("raftRxTask: %u received unknown raft message type = %u.\n",
                            raftNode.me,
                            type);
      }
      xSemaphoreGive(raftNode.mu);
    }
    vTaskDelay(M2T(1));
  }
}

static void bufferRaftCommand(uint16_t readIndex, Raft_Log_Command_t *command) {
  Raft_Log_Command_Buffer_Item_t item = {
      .type = command->type,
      .clientId = command->clientId,
      .requestId = command->requestId,
      .readIndex = readIndex
  };
  if (xQueueSend(commandBufferQueue, &item, M2T(0)) == pdFALSE) {
    Raft_Log_Command_Buffer_Item_t evicted;
    xQueueReceive(commandBufferQueue, &evicted, M2T(0));
    DEBUG_PRINT(
        "bufferRaftCommand: %u command buffer is full, evict command from %u, type = %d, requestId = %u, readIndex = %u.\n",
        raftNode.me,
        evicted.clientId,
        evicted.type,
        evicted.requestId,
        evicted.readIndex
    );
    xQueueSend(commandBufferQueue, &item, M2T(0));
  }
  DEBUG_PRINT("bufferRaftCommand: %u buffer command from %u, type = %d, requestId = %u, readIndex = %u.\n",
              raftNode.me,
              item.clientId,
              item.type,
              item.requestId,
              item.readIndex
  );
}

static void raftCommandBufferConsumeTask() {
  systemWaitStart();

  Raft_Log_Command_Buffer_Item_t item;

  while (1) {
    if (xQueuePeek(commandBufferQueue, &item, portMAX_DELAY)) {
      if (item.readIndex <= raftNode.lastApplied) {
        xQueueReceive(commandBufferQueue, &item, M2T(0));
        // 这个在做什么？
        raftSendCommandReply(item.clientId, raftNode.latestAppliedRequestId[item.clientId], raftNode.currentLeader, true);
      } else {
        vTaskDelay(M2T(RAFT_HEARTBEAT_INTERVAL / 5));
      }
    }
    vTaskDelay(M2T(1));
  }
}

void raftInit() {
  rxQueue = xQueueCreate(RAFT_RX_QUEUE_SIZE, RAFT_RX_QUEUE_ITEM_SIZE);
  commandBufferQueue = xQueueCreate(RAFT_COMMAND_BUFFER_QUEUE_SIZE, RAFT_COMMAND_BUFFER_QUEUE_ITEM_SIZE);
  raftRequestIdMutex = xSemaphoreCreateMutex();
  UWB_Data_Packet_Listener_t listener = {
      .type = UWB_DATA_MESSAGE_RAFT,
      .rxQueue = rxQueue
  };
  uwbRegisterDataPacketListener(&listener);

  raftNode.mu = xSemaphoreCreateMutex();
  raftNode.currentLeader = UWB_DEST_EMPTY;
  raftNode.me = uwbGetAddress();
  for (int i = 0; i <= RAFT_CLUSTER_PEER_NODE_ADDRESS_MAX; i++) {
    raftNode.peerVote[i] = false;
  }
  raftNode.currentState = RAFT_STATE_FOLLOWER;
  raftNode.currentTerm = 0;
  raftNode.voteFor = RAFT_VOTE_FOR_NO_ONE;
  raftNode.log.items[0] = EMPTY_LOG_ITEM;
  raftNode.log.size = 1;
  raftNode.commitIndex = 0;
  raftNode.lastApplied = 0;
  for (int i = 0; i <= RAFT_CLUSTER_PEER_NODE_ADDRESS_MAX; i++) {
    raftNode.nextIndex[i] = raftNode.log.items[raftNode.log.size - 1].index + 1;
    raftNode.matchIndex[i] = 0;
    raftNode.latestAppliedRequestId[i] = 0;
  }
  raftNode.lastHeartbeatTime = xTaskGetTickCount();
  raftNode.config = EMPTY_CONFIG;
  raftNode.config.clusterId = raftClusterId;
  for (int i = 0; i <= 4; i++) {
    raftConfigAdd(i);
  }
  DEBUG_PRINT("raftInit: node id = %u, cluster id = %u.\n", raftNode.me, raftClusterId);
  printRaftConfig(raftNode.config);

  raftHeartbeatTimer = xTimerCreate("raftHeartbeatTimer",
                                   M2T(RAFT_HEARTBEAT_INTERVAL),
                                   pdTRUE,
                                   (void *) 0,
                                    raftHeartbeatTimerCallback);
  xTimerStart(raftHeartbeatTimer, M2T(0));
  raftElectionTimer = xTimerCreate("raftElectionTimer",
                                   M2T((rand() % 50 + RAFT_ELECTION_TIMEOUT) / 2),
                                   pdTRUE,
                                   (void *) 0,
                                   raftElectionTimerCallback);
  xTimerStart(raftElectionTimer, M2T(0));
  raftLogApplyTimer = xTimerCreate("raftLogApplyTimer",
                                   M2T(RAFT_LOG_APPLY_INTERVAL),
                                   pdTRUE,
                                   (void *) 0,
                                   raftLogApplyTimerCallback);
  xTimerStart(raftLogApplyTimer, M2T(0));

  xTaskCreate(raftRxTask, ADHOC_DECK_RAFT_RX_TASK_NAME, UWB_TASK_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &raftRxTaskHandle);
  xTaskCreate(raftCommandBufferConsumeTask, ADHOC_DECK_RAFT_COMMAND_BUFFER_CONSUME_TASK_NAME, UWB_TASK_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &raftCommandTaskHandle);
}

Raft_Node_t *getGlobalRaftNode() {
  return &raftNode;
}

// TODO: leader broadcasting, follower uni-casting
void raftSendRequestVote(UWB_Address_t peerAddress) {
  UWB_Data_Packet_t dataTxPacket;
  dataTxPacket.header.type = UWB_DATA_MESSAGE_RAFT;
  dataTxPacket.header.srcAddress = raftNode.me;
  dataTxPacket.header.destAddress = peerAddress;
  dataTxPacket.header.ttl = 10;
  dataTxPacket.header.length = sizeof(UWB_Data_Packet_Header_t) + sizeof(Raft_Request_Vote_Args_t);
  Raft_Request_Vote_Args_t *args = (Raft_Request_Vote_Args_t *) &dataTxPacket.payload;
  args->type = RAFT_REQUEST_VOTE;
  args->term = raftNode.currentTerm;
  args->candidateId = raftNode.me;
  args->lastLogIndex = raftNode.log.items[raftNode.log.size - 1].index;
  args->lastLogTerm = raftNode.log.items[raftNode.log.size - 1].term;
  DEBUG_PRINT("raftSendRequestVote: %u send request vote to %u.\n", raftNode.me, peerAddress);
  uwbSendDataPacketBlock(&dataTxPacket);
}

void raftProcessRequestVote(UWB_Address_t peerAddress, Raft_Request_Vote_Args_t *args) {
  DEBUG_PRINT("raftProcessRequestVote: %u received vote request from %u.\n", raftNode.me, peerAddress);
  if (!raftConfigHasPeer(peerAddress)) {
    DEBUG_PRINT("raftProcessRequestVote: Peer %u not in current config, ignore.\n", peerAddress);
    return;
  }
  if (args->term < raftNode.currentTerm) {
    DEBUG_PRINT("raftProcessRequestVote: Candidate term = %u < my term = %u, ignore.\n",
                args->term,
                raftNode.currentTerm);
    raftSendRequestVoteReply(args->candidateId, raftNode.currentTerm, false);
    return;
  }
  /* If RPC request or response contains term T > currentTerm, set currentTerm = T, convert to follower. */
  if (args->term > raftNode.currentTerm) {
    DEBUG_PRINT("raftProcessRequestVote: Candidate term = %u > my term = %u, convert to follower.\n",
                args->term,
                raftNode.currentTerm
    );
    // TODO:难道不应该设置投票吗？
    raftNode.currentTerm = args->term;
    convertToFollower(&raftNode);
    raftSendRequestVoteReply(args->candidateId, raftNode.currentTerm, true);
    return;
  }
  if (raftNode.voteFor != RAFT_VOTE_FOR_NO_ONE && raftNode.voteFor != args->candidateId) {
    DEBUG_PRINT("raftProcessRequestVote: I %u have already granted vote to %u this term, don't grant vote.\n",
                raftNode.me,
                raftNode.voteFor);
    raftSendRequestVoteReply(args->candidateId, raftNode.currentTerm, false);
    return;
  }
  Raft_Log_Item_t lastLog = raftNode.log.items[raftNode.log.size - 1];
  if (args->lastLogTerm < lastLog.term || (args->lastLogTerm == lastLog.term && args->lastLogIndex < lastLog.index)) {
    DEBUG_PRINT("raftProcessRequestVote: My %u local log entries are more up-to-date than %u, don't grant vote.\n",
                raftNode.me,
                args->candidateId);
    raftSendRequestVoteReply(args->candidateId, raftNode.currentTerm, false);
    return;
  }
  // TODO:是不是只要raftNode.voteFor != RAFT_VOTE_FOR_NO_ONE 就应该直接返回，防止重复投票？
  // TODO:那应该是没收到投票请求，才会再来请求，而至于重复投票问题由leader决定
  // TODO:投票的幂等性，是由leader来决定的
  raftNode.voteFor = args->candidateId;
  raftNode.lastHeartbeatTime = xTaskGetTickCount();
  DEBUG_PRINT("raftProcessRequestVote: %u grant vote to %u.\n", raftNode.me, args->candidateId);
  raftSendRequestVoteReply(args->candidateId, raftNode.currentTerm, true);
}

void raftSendRequestVoteReply(UWB_Address_t peerAddress, uint16_t term, bool voteGranted) {
  UWB_Data_Packet_t dataTxPacket;
  dataTxPacket.header.type = UWB_DATA_MESSAGE_RAFT;
  dataTxPacket.header.srcAddress = raftNode.me;
  dataTxPacket.header.destAddress = peerAddress;
  dataTxPacket.header.ttl = 10;
  dataTxPacket.header.length = sizeof(UWB_Data_Packet_Header_t) + sizeof(Raft_Request_Vote_Reply_t);
  Raft_Request_Vote_Reply_t *reply = (Raft_Request_Vote_Reply_t *) &dataTxPacket.payload;
  reply->type = RAFT_REQUEST_VOTE_REPLY;
  reply->term = term;
  reply->voteGranted = voteGranted;
  DEBUG_PRINT("raftSendRequestVoteReply: %u send vote reply to %u, term = %u, granted = %d.\n",
              raftNode.me,
              peerAddress,
              term,
              voteGranted);
  uwbSendDataPacketBlock(&dataTxPacket);
}

void raftProcessRequestVoteReply(UWB_Address_t peerAddress, Raft_Request_Vote_Reply_t *reply) {
  DEBUG_PRINT("raftProcessRequestVoteReply: %u received vote request reply from %u, grant = %d.\n",
              raftNode.me,
              peerAddress,
              reply->voteGranted);
  if (!raftConfigHasPeer(peerAddress)) {
    DEBUG_PRINT("raftProcessRequestVoteReply: Peer %u not in current config, ignore.\n", peerAddress);
    return;
  }
  // 这种应该就是过期的请求，因为如果是正常请求，其他投票者一定大于等于自己的任期，因为如果投票者任期小，那么一定已经更新为当前的
  if (reply->term < raftNode.currentTerm) {
    DEBUG_PRINT("raftProcessRequestVoteReply: Peer term = %u < my term = %u, ignore.\n",
                reply->term,
                raftNode.currentTerm);
    return;
  }
  /* If RPC request or response contains term T > currentTerm, set currentTerm = T, convert to follower. */
  if (reply->term > raftNode.currentTerm) {
    DEBUG_PRINT("raftProcessRequestVoteReply: Peer term = %u > my term = %u, convert to follower.\n",
                reply->term,
                raftNode.currentTerm
    );
    raftNode.currentTerm = reply->term;
    convertToFollower(&raftNode);
  }
  // 这里就是要么投票了，要么没有投票
  if (reply->voteGranted && raftNode.currentState == RAFT_STATE_CANDIDATE) {
    raftNode.peerVote[peerAddress] = true;
    uint8_t voteCount = 0;
    for (int i = 0; i <= RAFT_CLUSTER_PEER_NODE_ADDRESS_MAX; i++) {
      if (i != raftNode.me && raftConfigHasPeer(i) && raftNode.peerVote[i]) {
        voteCount++;
      }
    }
    DEBUG_PRINT("raftProcessRequestVoteReply: voteCount = %u.\n", voteCount);
    if (voteCount >= raftNode.config.clusterSize / 2) {
      DEBUG_PRINT("raftProcessRequestVoteReply: %u elected as leader.\n", raftNode.me);
      convertToLeader(&raftNode);
      /* Upon election: send initial empty AppendEntries RPCs (heartbeat) to each server, repeat during idle periods to
       * prevent election timeouts.
       */
      for (int peerNode = 0; peerNode <= RAFT_CLUSTER_PEER_NODE_ADDRESS_MAX; peerNode++) {
        if (peerNode != raftNode.me && raftConfigHasPeer(peerNode)) {
          raftNode.nextIndex[peerNode] = raftNode.log.size;
          raftNode.matchIndex[peerNode] = 0;
          raftSendAppendEntries(peerNode);
        }
      }
    }
  }
}

void raftSendAppendEntries(UWB_Address_t peerAddress) {
  UWB_Data_Packet_t dataTxPacket;
  dataTxPacket.header.type = UWB_DATA_MESSAGE_RAFT;
  dataTxPacket.header.srcAddress = raftNode.me;
  dataTxPacket.header.destAddress = peerAddress;
  dataTxPacket.header.ttl = 10;
  dataTxPacket.header.length = sizeof(UWB_Data_Packet_Header_t) + sizeof(Raft_Append_Entries_Args_t);
  Raft_Append_Entries_Args_t *args = (Raft_Append_Entries_Args_t *) &dataTxPacket.payload;
  args->type = RAFT_APPEND_ENTRIES;
  args->term = raftNode.currentTerm;
  args->leaderId = raftNode.me;
  // TODO: check snapshot
  int preLogItemIndex = raftLogFindByIndex(&raftNode.log, raftNode.nextIndex[peerAddress] - 1);
  if (preLogItemIndex < 0) {
    DEBUG_PRINT("raftSendAppendEntries: %u has preLogItemIndex < 0 for peer %u, ignore.\n", raftNode.me, peerAddress);
    return;
  }
  args->prevLogIndex = raftNode.log.items[preLogItemIndex].index;
  args->prevLogTerm = raftNode.log.items[preLogItemIndex].term;
  args->leaderConfig = raftNode.config;
  int startItemIndex = preLogItemIndex + 1;
  int endItemIndex = MIN(preLogItemIndex + RAFT_LOG_ENTRIES_SIZE_MAX, raftNode.log.size - 1);
  DEBUG_PRINT("raftSendAppendEntries: startItemIndex = %u, endItemIndex = %u, matchIndex = %u.\n",
              startItemIndex,
              endItemIndex,
              raftNode.matchIndex[peerAddress]);
  /* Include log items with item index in [startItemIndex, endItemIndex] */
  args->entryCount = 0;
  for (int i = startItemIndex; i <= endItemIndex; i++) {
    args->entries[args->entryCount] = raftNode.log.items[i];
    args->entryCount++;
  }
  args->leaderCommit = raftNode.commitIndex;
  DEBUG_PRINT("raftSendAppendEntries: %u send %d entries to %u, current log size = %u.\n",
              raftNode.me,
              args->entryCount,
              peerAddress,
              raftNode.log.size);
  uwbSendDataPacketBlock(&dataTxPacket);
}

void raftProcessAppendEntries(UWB_Address_t peerAddress, Raft_Append_Entries_Args_t *args) {
  DEBUG_PRINT("raftProcessAppendEntries: %u received append entries request from %u.\n", raftNode.me, peerAddress);
  if (raftNode.config.clusterId == RAFT_CLUSTER_ID_EMPTY && (args->leaderConfig.currentConfig & (1 << raftNode.me))) {
    /* 1. This works for one-step member changes, since new or removed node doesn't have a valid cluster id, only the
     * cluster leader that received RAFT_LOG_COMMAND_CONFIG_ADD may send append entries requests to current node.
     * 2. If the leader changes, this still works since other cluster members still using the previous configuration, so
     * current node doesn't affect the correctness: before the member change command been applied, only the leader can
     * send append entries to current node, new leaders within the same cluster will take over.
     * 3. To guarantee the log consistency, newly added node just act as a listener before actually join the cluster.
     * 4. If the leader changes, this still works since other cluster members still using the previous configuration, so
     * current node doesn't affect the correctness.
     */
    raftNode.config = args->leaderConfig;
    DEBUG_PRINT("raftProcessAppendEntries: %u try to join the cluster %u.\n", raftNode.me, raftNode.config.clusterId);
  }
  if (args->leaderConfig.clusterId != raftNode.config.clusterId || !raftConfigHasPeer(peerAddress)) {
    DEBUG_PRINT("raftProcessAppendEntries: Peer %u not in current config, ignore.\n", peerAddress);
    return;
  }
  if (args->term < raftNode.currentTerm) {
    DEBUG_PRINT("raftProcessAppendEntries: Peer term = %u < my term = %u, ignore.\n",
                args->term,
                raftNode.currentTerm);
    raftSendAppendEntriesReply(peerAddress, raftNode.currentTerm, false, 0);  // 我比你数据更多，你别给我发了
    return;
  }
  /* If RPC request or response contains term T > currentTerm, set currentTerm = T, convert to follower. */
  if (args->term > raftNode.currentTerm) {
    DEBUG_PRINT("raftProcessAppendEntries: Peer term = %u > my term = %u, convert to follower.\n",
                args->term,
                raftNode.currentTerm
    );
    raftNode.currentTerm = args->term;
    convertToFollower(&raftNode); // 转到follower并更新当前任期
  }
  /* Candidate: If AppendEntries RPC received from new leader, convert to follower. */
  if (raftNode.currentState == RAFT_STATE_CANDIDATE) {
    DEBUG_PRINT(
        "raftProcessAppendEntries: Candidate %u received append entries request from new leader %u, convert to follower.\n",
        raftNode.me,
        peerAddress);
    convertToFollower(&raftNode);
  }
  raftNode.currentLeader = peerAddress;
  raftNode.lastHeartbeatTime = xTaskGetTickCount();
  /* Reply false if log doesn't contain an entry at prevLogIndex whose term matches prevLogTerm. */
  // TODO: check snapshot
  int matchedItemIndex = raftLogFindMatched(&raftNode.log, args->prevLogIndex, args->prevLogTerm);
  if (matchedItemIndex == -1) {
    DEBUG_PRINT(
        "raftProcessAppendEntries: %u log doesn't contain an entry at prevLogIndex = %u whose term matches prevLogTerm = %u, log size = %u, last applied = %u.\n",
        raftNode.me,
        args->prevLogIndex,
        args->prevLogTerm,
        raftNode.log.size,
        raftNode.lastApplied);
    raftSendAppendEntriesReply(peerAddress, raftNode.currentTerm, false, raftNode.lastApplied + 1);
    return;
  }
  /* If an existing entry conflicts with a new one (same index but different terms), delete the existing entry and all
   * that follow it. Here we just clean all entries follow and overwrite them with entries from request args.
   */
  // TODO: check snapshot
  int startItemIndex = matchedItemIndex + 1;
  raftLogCleanFrom(&raftNode.log, startItemIndex);
  DEBUG_PRINT("raftProcessAppendEntries: start index = %u, log size = %u.\n",
              startItemIndex,
              raftNode.log.size);
  /* Append any new entries not already in the log. */
  for (int i = 0; i < args->entryCount; i++) {
    raftLogAppend(&raftNode.log, args->entries[i].term, &args->entries[i].command);
  }
  /* If leaderCommit > commitIndex, set commitIndex = minInt(leaderCommit, index of last new entry) */
  if (args->leaderCommit > raftNode.commitIndex) {
    DEBUG_PRINT("raftProcessAppendEntries: Leader commit = %u > commitIndex = %u, update.\n",
                args->leaderCommit,
                raftNode.commitIndex);
    raftNode.commitIndex = MIN(args->leaderCommit, raftNode.log.items[raftNode.log.size - 1].index);
    // 从leader的commitIndex来更新更新commitIndex
  }
  raftApplyLog();
  raftSendAppendEntriesReply(peerAddress, raftNode.currentTerm, true, raftNode.log.items[raftNode.log.size - 1].index + 1);
}

void raftSendAppendEntriesReply(UWB_Address_t peerAddress, uint16_t term, bool success, uint16_t nextIndex) {
  UWB_Data_Packet_t dataTxPacket;
  dataTxPacket.header.type = UWB_DATA_MESSAGE_RAFT;
  dataTxPacket.header.srcAddress = raftNode.me;
  dataTxPacket.header.destAddress = peerAddress;
  dataTxPacket.header.ttl = 10;
  dataTxPacket.header.length = sizeof(UWB_Data_Packet_Header_t) + sizeof(Raft_Append_Entries_Reply_t);
  Raft_Append_Entries_Reply_t *reply = (Raft_Append_Entries_Reply_t *) &dataTxPacket.payload;
  reply->type = RAFT_APPEND_ENTRIES_REPLY;
  reply->term = term;
  reply->success = success;
  reply->nextIndex = nextIndex;
  DEBUG_PRINT("raftSendAppendEntriesReply: %u send append entries reply to %u, term = %u, success = %d, nextIndex = %u.\n",
              raftNode.me,
              peerAddress,
              term,
              success,
              nextIndex);
  uwbSendDataPacketBlock(&dataTxPacket);
}

void raftProcessAppendEntriesReply(UWB_Address_t peerAddress, Raft_Append_Entries_Reply_t *reply) {
  DEBUG_PRINT("raftProcessAppendEntriesReply: %u received append entries reply from %u.\n", raftNode.me, peerAddress);
  if (!raftConfigHasPeer(peerAddress)) {
    DEBUG_PRINT("raftProcessAppendEntriesReply: Peer %u not in current config, ignore.\n", peerAddress);
    return;
  }
  if (reply->term < raftNode.currentTerm) {
    DEBUG_PRINT("raftProcessAppendEntriesReply: Peer term = %u < my term = %u, ignore.\n",
                reply->term,
                raftNode.currentTerm);
    return;
  }
  /* If RPC request or response contains term T > currentTerm, set currentTerm = T, convert to follower. */
  if (reply->term > raftNode.currentTerm) {
    DEBUG_PRINT("raftProcessAppendEntriesReply: Peer term = %u > my term = %u, convert to follower.\n",
                reply->term,
                raftNode.currentTerm
    );
    raftNode.currentTerm = reply->term;
    convertToFollower(&raftNode);
  }
  if (raftNode.currentState != RAFT_STATE_LEADER) {
    DEBUG_PRINT("raftProcessAppendEntriesReply: %u is not a leader now, ignore.\n", raftNode.me);
    return;
  }
  if (reply->success) {
    /* If successful: update nextIndex and matchIndex for follower. */
    raftNode.nextIndex[peerAddress] = MAX(raftNode.nextIndex[peerAddress], reply->nextIndex);
    raftNode.matchIndex[peerAddress] = raftNode.nextIndex[peerAddress] - 1;
    /* If there exists an N such that N > commitIndex, a majority of matchIndex[i] ≥ N,
     * and log[N].term == currentTerm: set commitIndex = N.
     */
    raftUpdateCommitIndex(&raftNode);
    raftApplyLog();
  } else {
    /* If AppendEntries fails because of log inconsistency: decrement nextIndex and retry.
     * Here we decrement nextIndex from nextIndex to matchIndex in a term by term way for efficiency.
     */
    int matchedIndex = raftLogFindByIndex(&raftNode.log, raftNode.matchIndex[peerAddress]);
    if (matchedIndex == -1) {
      DEBUG_PRINT("raftProcessAppendEntriesReply: %u cannot find log with matchIndex = %u.\n",
                  raftNode.me,
                  raftNode.matchIndex[peerAddress]
      );
    } else {
      int itemIndex = raftLogGetLastLogItemByTerm(&raftNode.log, raftNode.log.items[matchedIndex].term - 1);
      if (itemIndex == -1) {
        DEBUG_PRINT("raftProcessAppendEntriesReply: %u cannot find log with term = %u.\n",
                    raftNode.me,
                    raftNode.log.items[matchedIndex].term - 1
        );
      } else {
        if (reply->nextIndex != 0) {
          if (reply->nextIndex < raftNode.matchIndex[peerAddress]) {
            DEBUG_PRINT("raftProcessAppendEntriesReply: Peer %u may restart, resync all log entries.\n", peerAddress);
            raftNode.nextIndex[peerAddress] = 1;
            raftNode.matchIndex[peerAddress] = 0;
          } else {
            raftNode.nextIndex[peerAddress] = reply->nextIndex;
            raftNode.matchIndex[peerAddress] = raftNode.nextIndex[peerAddress] - 1;
          }
        } else {
          // TODO:这是什么情况
          raftNode.nextIndex[peerAddress] = raftNode.log.items[itemIndex].index;
          DEBUG_PRINT("raftProcessAppendEntriesReply: Try to adjust next index to %u.\n", raftNode.nextIndex[peerAddress]);
        }
      }
    }
  }
}

void raftSendCommand(Raft_Command_Args_t *args) {
  // 一次只发送一条命令
  UWB_Data_Packet_t dataTxPacket;
  dataTxPacket.header.type = UWB_DATA_MESSAGE_RAFT;
  dataTxPacket.header.srcAddress = raftNode.me;
  dataTxPacket.header.destAddress = raftNode.currentLeader;
  dataTxPacket.header.ttl = 10;
  dataTxPacket.header.length = sizeof(UWB_Data_Packet_Header_t) + sizeof(Raft_Command_Args_t);
  memcpy(dataTxPacket.payload, args, sizeof (Raft_Command_Args_t));
  DEBUG_PRINT("raftSendCommand: %u send command to leader %u, requestId = %u.\n",
              raftNode.me,
              raftNode.currentLeader,
              args->command.requestId);
  uwbSendDataPacketBlock(&dataTxPacket);
}

void raftProcessCommand(UWB_Address_t clientId, Raft_Command_Args_t *args) {
  DEBUG_PRINT("raftProcessCommand: %u received command from client %u, requestId = %u.\n",
              raftNode.me,
              clientId,
              args->command.requestId);
  if (raftNode.currentLeader != raftNode.me) {
    DEBUG_PRINT("raftProcessCommand: %u is not the leader, current leader is %u.\n",
                raftNode.me,
                raftNode.currentLeader);
    raftSendCommandReply(clientId, raftNode.latestAppliedRequestId[clientId], raftNode.currentLeader, false);
    return;
  }
  if (args->command.requestId <= raftNode.latestAppliedRequestId[clientId]) {
    // 通过全局requestId进行幂等性保证
    DEBUG_PRINT("raftProcessCommand: %u received duplicated command from client %u, requestId = %u.\n",
                raftNode.me,
                clientId,
                args->command.requestId);
    raftSendCommandReply(clientId, raftNode.latestAppliedRequestId[clientId], raftNode.me, true);
    return;
  }
  // 走到这里，当前是leader
  // 合并C_OLD和C_NEW执行一遍成员变更，只有log提交才会变更成功
  if ((args->command.type == RAFT_LOG_COMMAND_CONFIG_ADD && raftConfigAdd(*(uint16_t *) &args->command.payload)) ||
      (args->command.type == RAFT_LOG_COMMAND_CONFIG_REMOVE && raftConfigRemove(*(uint16_t *) &args->command.payload))) {
    /* Now use the combination of C_OLD and C_NEW to perform one-step membership change, when the change log is committed,
     * the log applier will preform the final member change operations (raftConfigAdd or raftConfigRemove).
     */
    raftNode.config.currentConfig = raftNode.config.currentConfig | raftNode.config.previousConfig;
    /* Here we append a no ops log entry to ensure success leader */
    Raft_Log_Command_t noOpsLog = {
        .type = RAFT_LOG_COMMAND_NO_OPS,
        .requestId = 0,
        .clientId = raftNode.me,
        // TODO: init empty payload
    };
    // 这里为何一个空的心跳包？ 心跳包发完不需要return吗？
    raftLogAppend(&raftNode.log, raftNode.currentTerm, &noOpsLog);
  }
  /* Append new log entry and then buffer this command with readIndex = index of the new log entry. */
  raftLogAppend(&raftNode.log, raftNode.currentTerm, &args->command);
  // 为何要buffer命令
  bufferRaftCommand(raftNode.log.items[raftNode.log.size - 1].index, &args->command);
}

void raftSendCommandReply(UWB_Address_t clientId, uint16_t latestApplied, UWB_Address_t leaderAddress, bool success) {
  UWB_Data_Packet_t dataTxPacket;
  dataTxPacket.header.type = UWB_DATA_MESSAGE_RAFT;
  dataTxPacket.header.srcAddress = raftNode.me;
  dataTxPacket.header.destAddress = clientId;
  dataTxPacket.header.ttl = 10;
  dataTxPacket.header.length = sizeof(UWB_Data_Packet_Header_t) + sizeof(Raft_Command_Reply_t);
  Raft_Command_Reply_t *reply = (Raft_Command_Reply_t *) &dataTxPacket.payload;
  reply->type = RAFT_COMMAND_REPLY;
  reply->latestApplied = latestApplied;
  reply->leaderAddress = leaderAddress;
  reply->success = success;
  DEBUG_PRINT("raftSendRequestVote: %u send command reply to client %u, latestApplied = %u, leader = %u, success = %d.\n",
              raftNode.me,
              clientId,
              latestApplied,
              leaderAddress,
              success);
  uwbSendDataPacketBlock(&dataTxPacket);
}

void raftProcessCommandReply(UWB_Address_t peerAddress, Raft_Command_Reply_t *reply) {
  DEBUG_PRINT("raftProcessCommandReply: %u received command reply from %u, latestApplied = %u, leader = %u.\n",
              raftNode.me,
              peerAddress,
              reply->latestApplied,
              reply->leaderAddress);
  if (reply->success) {
    raftLeaderApply = MAX(raftLeaderApply, reply->latestApplied);
  } else {
    if (reply->leaderAddress != raftNode.currentLeader) {
      DEBUG_PRINT("raftProcessCommandReply: %u think current leader is %u not %u.\n",
                  peerAddress,
                  reply->leaderAddress,
                  raftNode.currentLeader);
      // TODO: ignore or retry?
    }
  }
}

uint16_t raftProposeNew(RAFT_LOG_COMMAND_TYPE type, uint8_t *payload, uint16_t size) {
  uint16_t requestId = getNextRequestId();
  if (raftNode.currentLeader != UWB_DEST_EMPTY) {
    Raft_Command_Args_t args = {
        .type = RAFT_COMMAND_REQUEST,
        .command = {
            .type = type,
            .clientId = raftNode.me,
            .requestId = requestId,
        },
    };
    memcpy(args.command.payload, payload, size);
    raftSendCommand(&args);
  }
  return requestId;
}

void raftProposeRetry(uint16_t requestId, RAFT_LOG_COMMAND_TYPE type, uint8_t *payload, uint16_t size) {
  if (raftNode.currentLeader != UWB_DEST_EMPTY) {
    Raft_Command_Args_t args = {
        .type = RAFT_COMMAND_REQUEST,
        .command = {
            .type = type,
            .clientId = raftNode.me,
            .requestId = requestId,
        },
    };
    memcpy(args.command.payload, payload, size);
    raftSendCommand(&args);
  }
}

bool raftProposeCheck(uint16_t requestId, int wait) {
  if (raftLeaderApply >= requestId) {
    return true;
  }
  vTaskDelay(M2T(wait));
  return raftLeaderApply >= requestId;
}

void printRaftConfig(Raft_Config_t config) {
  DEBUG_PRINT("cluster id = %u, size = %u, Members = ", config.clusterId, config.clusterSize);
  for (int i = 0; i <= RAFT_CLUSTER_PEER_NODE_ADDRESS_MAX; i++) {
    if (config.currentConfig & (1 << i)) {
      DEBUG_PRINT("%d ", i);
    }
  }
  DEBUG_PRINT("\n");
}

void printRaftLog(Raft_Log_t *raftLog) {
  DEBUG_PRINT("term\t index\t clientId\t reqId\t type\t commit\t \n");
  for (int i = 0; i < raftLog->size; i++) {
    DEBUG_PRINT("%u\t %u\t %u\t %u\t %u\t %u\t \n",
                raftLog->items[i].term,
                raftLog->items[i].index,
                raftLog->items[i].command.clientId,
                raftLog->items[i].command.requestId,
                raftLog->items[i].command.type,
                raftNode.commitIndex);
  }
}

void printRaftLogItem(Raft_Log_Item_t *item) {
  DEBUG_PRINT("term\t index\t clientId\t reqId\t type\t \n");
  DEBUG_PRINT("%u\t %u\t %u\t %u\t %u\t \n",
              item->term,
              item->index,
              item->command.clientId,
              item->command.requestId,
              item->command.type);
}

PARAM_GROUP_START(RAFT)
  PARAM_ADD_CORE(PARAM_UINT32 | PARAM_PERSISTENT, RAFT_CONFIG, &raftMembers)
  PARAM_ADD_CORE(PARAM_UINT8 | PARAM_PERSISTENT, RAFT_CONFIG, &raftClusterId)
PARAM_GROUP_STOP(RAFT)