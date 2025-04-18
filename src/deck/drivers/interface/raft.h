#ifndef __RAFT_H__
#define __RAFT_H__
#include <stdint.h>
#include <stdbool.h>
#include "semphr.h"
#include "routing.h"

//#define RAFT_DEBUG_ENABLE

/* Queue Constants */
#define RAFT_RX_QUEUE_SIZE 5
#define RAFT_RX_QUEUE_ITEM_SIZE sizeof(UWB_Data_Packet_t)
#define RAFT_COMMAND_BUFFER_QUEUE_SIZE 10
#define RAFT_COMMAND_BUFFER_QUEUE_ITEM_SIZE sizeof(Raft_Log_Command_Buffer_Item_t)

/* Raft Constants */
#define RAFT_LOG_SIZE_MAX 50
#define RAFT_CLUSTER_PEER_NODE_ADDRESS_MAX 31
#define RAFT_VOTE_FOR_NO_ONE UWB_DEST_EMPTY
#define RAFT_HEARTBEAT_INTERVAL 1000 // default 150ms
#define RAFT_ELECTION_TIMEOUT (5 * RAFT_HEARTBEAT_INTERVAL)
#define RAFT_LOG_APPLY_INTERVAL 50 // default 50ms
#define RAFT_LOG_COMMAND_PAYLOAD_SIZE_MAX 15 // TODO: fine tuning
#define RAFT_LOG_ENTRIES_SIZE_MAX ((ROUTING_DATA_PACKET_PAYLOAD_SIZE_MAX - 28) / sizeof(Raft_Log_Item_t))
#define RAFT_CLUSTER_ID_EMPTY 255

typedef enum {
  RAFT_STATE_FOLLOWER,
  RAFT_STATE_CANDIDATE,
  RAFT_STATE_LEADER
} RAFT_STATE;

typedef enum {
  RAFT_LOG_COMMAND_RESERVED,
  RAFT_LOG_COMMAND_NO_OPS,
  RAFT_LOG_COMMAND_CONFIG_ADD,
  RAFT_LOG_COMMAND_CONFIG_REMOVE,
  RAFT_LOG_COMMAND_GET,
  RAFT_LOG_COMMAND_PUT
} RAFT_LOG_COMMAND_TYPE;

typedef struct {
  RAFT_LOG_COMMAND_TYPE type;
  UWB_Address_t clientId;
  uint16_t requestId;
  // TODO payload
  uint8_t payload[RAFT_LOG_COMMAND_PAYLOAD_SIZE_MAX];
} __attribute__((packed)) Raft_Log_Command_t;

typedef struct {
  RAFT_LOG_COMMAND_TYPE type;
  UWB_Address_t clientId;
  uint16_t requestId;
  uint16_t readIndex;
} Raft_Log_Command_Buffer_Item_t;

typedef struct {
  uint16_t term;
  uint16_t index;
  Raft_Log_Command_t command; // TODO: COMMAND enum type definition
} Raft_Log_Item_t;

typedef struct {
  uint16_t size;
  Raft_Log_Item_t items[RAFT_LOG_SIZE_MAX];
} Raft_Log_t;

typedef struct {
  uint32_t currentConfig; /* C_NEW */
  uint32_t previousConfig; /* C_OLD */
  uint8_t clusterId;
  uint8_t clusterSize;
//  uint8_t prevClusterId;
} __attribute__((packed)) Raft_Config_t;

typedef struct {
  SemaphoreHandle_t mu;
  UWB_Address_t currentLeader;
  UWB_Address_t me;
  bool peerVote[RAFT_CLUSTER_PEER_NODE_ADDRESS_MAX + 1]; /* granted vote count from peer nodes in current term */
  // TODO: heartbeat flag for read index
  RAFT_STATE currentState;
  uint16_t currentTerm; /* latest term server has seen (initialized to 0 on first boot, increases monotonically) */
  UWB_Address_t voteFor; /* candidate that received vote in current term (or null if none), RAFT_VOTE_FOR_NO_ONE == null */
  Raft_Log_t log; /* log entries, each entry contains command for state machine, and term when entry was received by leader (first index is 1) */
  uint16_t commitIndex; /* index of highest log entry known to be committed (initialized to 0, increases monotonically) */
  uint16_t lastApplied; /* index of highest log entry known to be applied to state machine (initialized to 0, increases monotonically) */
  uint16_t nextIndex[RAFT_CLUSTER_PEER_NODE_ADDRESS_MAX + 1]; /* for each server, index of the next log entry to send to that server (initialized to leader last log index + 1) */
  uint16_t matchIndex[RAFT_CLUSTER_PEER_NODE_ADDRESS_MAX + 1]; /* for each server, index of highest log entry known to be replicated on server (initialized to 0, increases monotonically) */
  Time_t lastHeartbeatTime; /* heartbeat used for trigger leader election */
  Raft_Config_t config; /* peer nodes in current raft cluster configuration */
  /* State for client */
  uint16_t latestAppliedRequestId[RAFT_CLUSTER_PEER_NODE_ADDRESS_MAX + 1]; /* latest request id applied to the state machine for each client */
} Raft_Node_t;

typedef enum {
  RAFT_REQUEST_VOTE,
  RAFT_REQUEST_VOTE_REPLY,
  RAFT_APPEND_ENTRIES,
  RAFT_APPEND_ENTRIES_REPLY,
  RAFT_COMMAND_REQUEST,
  RAFT_COMMAND_REPLY
} RAFT_MESSAGE_TYPE;

typedef struct {
  RAFT_MESSAGE_TYPE type;
  uint16_t term; /* candidate's term */
  UWB_Address_t candidateId; /* candidate that requesting vote */
  uint16_t lastLogIndex; /* index of candidate's last log entry */
  uint16_t lastLogTerm; /* term of candidate's last log entry */
} __attribute__((packed)) Raft_Request_Vote_Args_t;

typedef struct {
  RAFT_MESSAGE_TYPE type;
  uint16_t term; /* currentTerm, for candidate to update itself */
  bool voteGranted; /* true means candidate received vote */
} __attribute__((packed)) Raft_Request_Vote_Reply_t;

typedef struct {
  RAFT_MESSAGE_TYPE type;
  uint16_t term; /* leader's term */
  UWB_Address_t leaderId; /* so follower can redirect clients */
  uint16_t prevLogIndex; /* index of log entry immediately preceding new ones */
  uint16_t prevLogTerm; /* term of prevLogIndex entry */
  uint16_t entryCount; /* log entries count */
  uint16_t leaderCommit; /* leader's commitIndex */
  Raft_Config_t leaderConfig; /* leader's config for newly added nodes to update itself */
  Raft_Log_Item_t entries[RAFT_LOG_ENTRIES_SIZE_MAX]; /* log entries to store (empty for heartbeat; may send more than one for efficiency) */
} __attribute__((packed)) Raft_Append_Entries_Args_t;

typedef struct {
  RAFT_MESSAGE_TYPE type;
  uint16_t term; /* currentTerm, for leader to update itself */
  bool success; /* true if follower contained entry matching prevLogIndex and prevLogTerm */
  /* Since we don't have rpc mechanism here, to help leader update nextIndex and matchIndex, follower should tell
   * leader it's replication progress. */
  uint16_t nextIndex; /* 0 represents null */
} __attribute__((packed)) Raft_Append_Entries_Reply_t;

typedef struct {
  RAFT_MESSAGE_TYPE type;
  Raft_Log_Command_t command;
} __attribute__((packed)) Raft_Command_Args_t;

typedef struct {
  RAFT_MESSAGE_TYPE type;
  uint16_t latestApplied; /* latest applied request id in the state machine */
  UWB_Address_t leaderAddress; /* current leader address for the client to retry */
  bool success;
} __attribute__((packed)) Raft_Command_Reply_t;

/* Raft Server Operations */
void raftInit();
Raft_Node_t *getGlobalRaftNode();
void raftSendRequestVote(UWB_Address_t peerAddress);
void raftProcessRequestVote(UWB_Address_t peerAddress, Raft_Request_Vote_Args_t *args);
void raftSendRequestVoteReply(UWB_Address_t peerAddress, uint16_t term, bool voteGranted);
void raftProcessRequestVoteReply(UWB_Address_t peerAddress, Raft_Request_Vote_Reply_t *reply);

void raftSendAppendEntries(UWB_Address_t peerAddress);
void raftProcessAppendEntries(UWB_Address_t peerAddress, Raft_Append_Entries_Args_t *args);
void raftSendAppendEntriesReply(UWB_Address_t peerAddress, uint16_t term, bool success, uint16_t nextIndex);
void raftProcessAppendEntriesReply(UWB_Address_t peerAddress, Raft_Append_Entries_Reply_t *reply);

void raftSendCommand(Raft_Command_Args_t *args);
void raftProcessCommand(UWB_Address_t clientId, Raft_Command_Args_t *args);
void raftSendCommandReply(UWB_Address_t clientId, uint16_t latestApplied, UWB_Address_t leaderAddress, bool success);
void raftProcessCommandReply(UWB_Address_t peerAddress, Raft_Command_Reply_t *reply);

void printRaftConfig(Raft_Config_t config);
void printRaftLog(Raft_Log_t *raftLog);
void printRaftLogItem(Raft_Log_Item_t *item);

/* Raft Client Operations */
uint16_t raftProposeNew(RAFT_LOG_COMMAND_TYPE type, uint8_t *payload, uint16_t size);
void raftProposeRetry(uint16_t requestId, RAFT_LOG_COMMAND_TYPE type, uint8_t *payload, uint16_t size);
bool raftProposeCheck(uint16_t requestId, int wait);

#endif