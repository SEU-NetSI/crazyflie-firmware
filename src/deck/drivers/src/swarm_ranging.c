#define DEBUG_MODULE "swarmranging"
#include <stdint.h>
#include <math.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "system.h"
#include "timers.h"
#include "autoconf.h"
#include "debug.h"
#include "log.h"
#include "assert.h"
#include "adhocdeck.h"
#include "ranging_struct.h"
#include "swarm_ranging.h"

static uint16_t MY_UWB_ADDRESS;

static QueueHandle_t rxQueue;
static Ranging_Table_Set_t rangingTableSet;
static UWB_Message_Listener_t listener;
static TaskHandle_t uwbRangingTxTaskHandle = 0;
static TaskHandle_t uwbRangingRxTaskHandle = 0;

static Timestamp_Tuple_t TfBuffer[Tf_BUFFER_POOL_SIZE] = {0};
static int TfBufferIndex = 0;
static int rangingSeqNumber = 1;

static logVarId_t idVelocityX, idVelocityY, idVelocityZ;
static float velocity;

int16_t distanceTowards[RANGING_TABLE_SIZE + 1] = {[0 ... RANGING_TABLE_SIZE] = -1};
uint8_t distanceSource[RANGING_TABLE_SIZE + 1] = {[0 ... RANGING_TABLE_SIZE] = -1};
float distanceReal[RANGING_TABLE_SIZE + 1] = {[0 ... RANGING_TABLE_SIZE] = -1};
typedef struct Stastistic
{
  uint16_t recvSeq;
  uint16_t recvnum;
  uint16_t compute1num;
  uint16_t compute2num;
} Stastistic;
static Stastistic statistic[RANGING_TABLE_SIZE + 1];
static TimerHandle_t statisticTimer;

void printStasticCallback(TimerHandle_t timer)
{
  // DEBUG_PRINT("recvnum:%d,compute1num:%d,compute2num:%d\n",
  //             statistic[1].recvnum,
  //             statistic[1].compute1num,
  //             statistic[1].compute2num);
}

void statisticInit()
{
  for (int i = 0; i <= RANGING_TABLE_SIZE; i++)
  {
    statistic[i].recvSeq = 0;
    statistic[i].recvnum = 0;
    statistic[i].compute1num = 0;
    statistic[i].compute2num = 0;
  }
  statisticTimer = xTimerCreate("statisticTimer",
                                500,
                                pdTRUE,
                                (void *)0,
                                printStasticCallback);
  xTimerStart(statisticTimer, M2T(0));
}


void rangingRxCallback(void *parameters)
{
  // DEBUG_PRINT("rangingRxCallback \n");

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  UWB_Packet_t *packet = (UWB_Packet_t *)parameters;

  dwTime_t rxTime = {0};
  dwt_readrxtimestamp((uint8_t *)&rxTime.raw);
  Ranging_Message_With_Timestamp_t rxMessageWithTimestamp;
  rxMessageWithTimestamp.rxTime = rxTime;
  Ranging_Message_t *rangingMessage = (Ranging_Message_t *)packet->payload;
  rxMessageWithTimestamp.rangingMessage = *rangingMessage;

  xQueueSendFromISR(rxQueue, &rxMessageWithTimestamp, &xHigherPriorityTaskWoken);
}

void rangingTxCallback(void *parameters)
{
  dwTime_t txTime = {0};
  dwt_readtxtimestamp((uint8_t *)&txTime.raw);
  TfBufferIndex++;
  TfBufferIndex %= Tf_BUFFER_POOL_SIZE;
  TfBuffer[TfBufferIndex].seqNumber = rangingSeqNumber;
  TfBuffer[TfBufferIndex].timestamp = txTime;
}

int16_t getDistance(uint16_t neighborAddress)
{
  ASSERT(neighborAddress <= RANGING_TABLE_SIZE);
  return distanceTowards[neighborAddress];
}

void setDistance(uint16_t neighborAddress, int16_t distance)
{
  ASSERT(neighborAddress <= RANGING_TABLE_SIZE);
  distanceTowards[neighborAddress] = distance;
}

static void uwbRangingTxTask(void *parameters)
{
  systemWaitStart();

  /* velocity log variable id */
  idVelocityX = logGetVarId("stateEstimate", "vx");
  idVelocityY = logGetVarId("stateEstimate", "vy");
  idVelocityZ = logGetVarId("stateEstimate", "vz");

  UWB_Packet_t txPacketCache;
  txPacketCache.header.type = RANGING;
  //  txPacketCache.header.mac = ? TODO init mac header
  while (true)
  {
    int msgLen = generateRangingMessage((Ranging_Message_t *)&txPacketCache.payload);
    txPacketCache.header.length = sizeof(Packet_Header_t) + msgLen;
    uwbSendPacketBlock(&txPacketCache);
    int delayms = 30+rand()%61;
    vTaskDelay(delayms);
  }
}

static void uwbRangingRxTask(void *parameters)
{
  systemWaitStart();
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  Ranging_Message_With_Timestamp_t rxPacketCache;

  while (true)
  {
    if (xQueueReceive(rxQueue, &rxPacketCache, portMAX_DELAY))
    {
      //      DEBUG_PRINT("uwbRangingRxTask: received ranging message \n");
      processRangingMessage(&rxPacketCache);
    }
  }
}

void rangingInit()
{
  MY_UWB_ADDRESS = getUWBAddress();
  DEBUG_PRINT("MY_UWB_ADDRESS = %d \n", MY_UWB_ADDRESS);
  rxQueue = xQueueCreate(RANGING_RX_QUEUE_SIZE, RANGING_RX_QUEUE_ITEM_SIZE);
  rangingTableSetInit(&rangingTableSet);

  listener.type = RANGING;
  listener.rxQueue = NULL; // handle rxQueue in swarm_ranging.c instead of adhocdeck.c
  listener.rxCb = rangingRxCallback;
  listener.txCb = rangingTxCallback;
  uwbRegisterListener(&listener);

  idVelocityX = logGetVarId("stateEstimate", "vx");
  idVelocityY = logGetVarId("stateEstimate", "vy");
  idVelocityZ = logGetVarId("stateEstimate", "vz");

  statisticInit();

  xTaskCreate(uwbRangingTxTask, ADHOC_DECK_RANGING_TX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbRangingTxTaskHandle); // TODO optimize STACK SIZE
  xTaskCreate(uwbRangingRxTask, ADHOC_DECK_RANGING_RX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbRangingRxTaskHandle); // TODO optimize STACK SIZE
}

int16_t computeDistance(Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp,
                        Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr,
                        Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf)
{

  int64_t tRound1, tReply1, tRound2, tReply2, diff1, diff2, tprop_ctn;
  tRound1 = (Rr.timestamp.full - Tp.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  tReply1 = (Tr.timestamp.full - Rp.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  tRound2 = (Rf.timestamp.full - Tr.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  tReply2 = (Tf.timestamp.full - Rr.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  diff1 = tRound1 - tReply1;
  diff2 = tRound2 - tReply2;
  tprop_ctn = (diff1 * tReply2 + diff2 * tReply1 + diff2 * diff1) / (tRound1 + tRound2 + tReply1 + tReply2);
  int16_t distance = (int16_t)tprop_ctn * 0.4691763978616;

  bool isErrorOccurred = false;
  if (distance > 1000 || distance < 0)
  {
    DEBUG_PRINT("isErrorOccurred\n");
    isErrorOccurred = true;
  }

  if (tRound2 < 0 || tReply2 < 0)
  {
    DEBUG_PRINT("tRound2 < 0 || tReply2 < 0\n");
    isErrorOccurred = true;
  }

  if (isErrorOccurred)
  {
    return 0;
  }
  // DEBUG_PRINT("d=%d\n",distance);
  return distance;
}

void processRangingMessage(Ranging_Message_With_Timestamp_t *rangingMessageWithTimestamp)
{
  Ranging_Message_t *rangingMessage = &rangingMessageWithTimestamp->rangingMessage;
  uint16_t neighborAddress = rangingMessage->header.srcAddress;
  set_index_t neighborIndex = findInRangingTableSet(&rangingTableSet, neighborAddress);

  statistic[neighborAddress].recvnum++;
  statistic[neighborAddress].recvSeq = rangingMessage->header.msgSequence;

  /* handle new neighbor */
  if (neighborIndex == -1)
  {
    if (rangingTableSet.freeQueueEntry == -1)
    {
      /* ranging table set is full, ignore this ranging message */
      return;
    }
    Ranging_Table_t table;
    rangingTableInit(&table, neighborAddress);
    neighborIndex = rangingTableSetInsert(&rangingTableSet, &table);
  }

  Ranging_Table_t *neighborRangingTable = &rangingTableSet.setData[neighborIndex].data;
  Ranging_Table_Tr_Rr_Buffer_t *neighborTrRrBuffer = &neighborRangingTable->TrRrBuffer;

  /* update Re */
  neighborRangingTable->Re.timestamp = rangingMessageWithTimestamp->rxTime;
  neighborRangingTable->Re.seqNumber = rangingMessage->header.msgSequence;

  /* update Tr and Rr */
  Timestamp_Tuple_t neighborTr = rangingMessage->header.lastTxTimestamp;
  if (neighborTr.timestamp.full && neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr.timestamp.full && neighborTr.seqNumber == neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr.seqNumber)
  {
    rangingTableBufferUpdate(&neighborRangingTable->TrRrBuffer,
                             neighborTr,
                             neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr);
  }

  /* update Rf */
  Timestamp_Tuple_t neighborRf = {.timestamp.full = 0};
  if (rangingMessage->header.filter & (1 << (getUWBAddress() % 16)))
  {
    /* retrieve body unit */
    uint8_t bodyUnitCount = (rangingMessage->header.msgLength - sizeof(Ranging_Message_Header_t)) / sizeof(Body_Unit_t);
    for (int i = 0; i < bodyUnitCount; i++)
    {
      if (rangingMessage->bodyUnits[i].address == getUWBAddress())
      {
        // neighborRf = rangingMessage->bodyUnits[i].timestamp;
        neighborRf.timestamp = rangingMessage->bodyUnits[i].timestamp;
        neighborRf.seqNumber = rangingMessage->bodyUnits[i].seqNumber;
        break;
      }
    }
  }
  // printRangingTable(neighborRangingTable);
  if (neighborRf.timestamp.full)
  {
    neighborRangingTable->Rf = neighborRf;
    // TODO it is possible that can not find corresponding Tf
    /* find corresponding Tf in TfBuffer */
    for (int i = 0; i < Tf_BUFFER_POOL_SIZE; i++)
    {
      if (TfBuffer[i].seqNumber == neighborRf.seqNumber)
      {
        neighborRangingTable->Tf = TfBuffer[i];
      }
    }

    Ranging_Table_Tr_Rr_Candidate_t Tr_Rr_Candidate = rangingTableBufferGetCandidate(&neighborRangingTable->TrRrBuffer,
                                                                                     neighborRangingTable->Tf);
    /* try to compute distance */
    if (Tr_Rr_Candidate.Tr.timestamp.full && Tr_Rr_Candidate.Rr.timestamp.full &&
        neighborRangingTable->Tp.timestamp.full && neighborRangingTable->Rp.timestamp.full &&
        neighborRangingTable->Tf.timestamp.full && neighborRangingTable->Rf.timestamp.full)
    {
      int16_t distance = computeDistance(neighborRangingTable->Tp, neighborRangingTable->Rp,
                                         Tr_Rr_Candidate.Tr, Tr_Rr_Candidate.Rr,
                                         neighborRangingTable->Tf, neighborRangingTable->Rf);
      if (distance > 0)
      {
        statistic[neighborRangingTable->neighborAddress].compute1num++;
        neighborRangingTable->distance = distance;
        setDistance(neighborRangingTable->neighborAddress, distance);
      }
      else
      {
        // DEBUG_PRINT("distance is not updated since some error occurs\n");
      }
    }
  }

  /* Tp <- Tf, Rp <- Rf */
  if (neighborRangingTable->Tf.timestamp.full && neighborRangingTable->Rf.timestamp.full)
  {
    rangingTableShift(neighborRangingTable);
  }

  /* update Rr */
  neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr = neighborRangingTable->Re;

  /* update expiration time */
  neighborRangingTable->expirationTime = xTaskGetTickCount() + M2T(RANGING_TABLE_HOLD_TIME);

  neighborRangingTable->state = RECEIVED;
}

int generateRangingMessage(Ranging_Message_t *rangingMessage)
{
#ifdef ENABLE_BUS_BOARDING_SCHEME
  sortRangingTableSet(&rangingTableSet);
#endif
  rangingTableSetClearExpire(&rangingTableSet);
  int8_t bodyUnitNumber = 0;
  rangingSeqNumber++;
  int curSeqNumber = rangingSeqNumber;
  rangingMessage->header.filter = 0;
  /* generate message body */
  for (set_index_t index = rangingTableSet.fullQueueEntry; index != -1;
       index = rangingTableSet.setData[index].next)
  {
    Ranging_Table_t *table = &rangingTableSet.setData[index].data;
    if (bodyUnitNumber >= MAX_BODY_UNIT_NUMBER)
    {
      break;
    }
    if (table->state == RECEIVED)
    {
      rangingMessage->bodyUnits[bodyUnitNumber].timestamp = table->Re.timestamp;
      rangingMessage->bodyUnits[bodyUnitNumber].seqNumber = table->Re.seqNumber;
      rangingMessage->bodyUnits[bodyUnitNumber].address = table->neighborAddress;
      /* It is possible that Re is not the newest timestamp, because the newest may be in rxQueue
       * waiting to be handled.
       */
      bodyUnitNumber++;
      table->state = TRANSMITTED;
      rangingMessage->header.filter |= 1 << (table->neighborAddress % 16);
    }
  }
  /* generate message header */
  rangingMessage->header.srcAddress = MY_UWB_ADDRESS;
  rangingMessage->header.msgLength = sizeof(Ranging_Message_Header_t) + sizeof(Body_Unit_t) * bodyUnitNumber;
  rangingMessage->header.msgSequence = curSeqNumber;
  rangingMessage->header.lastTxTimestamp = TfBuffer[TfBufferIndex];
  float velocityX = logGetFloat(idVelocityX);
  float velocityY = logGetFloat(idVelocityY);
  float velocityZ = logGetFloat(idVelocityZ);
  velocity = sqrt(pow(velocityX, 2) + pow(velocityY, 2) + pow(velocityZ, 2));
  /* velocity in cm/s */
  rangingMessage->header.velocity = (short)(velocity * 100);
  return rangingMessage->header.msgLength;
}

LOG_GROUP_START(Ranging)
LOG_ADD(LOG_INT16, distTo1, distanceTowards + 1)
LOG_ADD(LOG_INT16, distTo2, distanceTowards + 2)
LOG_ADD(LOG_INT16, distTo3, distanceTowards + 3)
LOG_ADD(LOG_INT16, distTo4, distanceTowards + 4)
LOG_ADD(LOG_INT16, distTo5, distanceTowards + 5)
LOG_ADD(LOG_INT16, distTo6, distanceTowards + 6)
LOG_ADD(LOG_INT16, distTo7, distanceTowards + 7)
LOG_ADD(LOG_INT16, distTo8, distanceTowards + 8)
LOG_ADD(LOG_INT16, distTo9, distanceTowards + 9)
LOG_ADD(LOG_INT16, distTo10, distanceTowards + 10)
LOG_GROUP_STOP(Ranging)


LOG_GROUP_START(Statistic)
LOG_ADD(LOG_UINT16, recvSeq0, &statistic[0].recvSeq)
LOG_ADD(LOG_UINT16, recvNum0, &statistic[0].recvnum)
LOG_ADD(LOG_UINT16, compute1num0, &statistic[0].compute1num)
LOG_ADD(LOG_UINT16, compute2num0, &statistic[0].compute2num)
LOG_ADD(LOG_INT16, dist0, distanceTowards + 0)
LOG_ADD(LOG_UINT8, distSrc0, distanceSource + 0)
LOG_ADD(LOG_FLOAT, distReal0, distanceReal + 0)


LOG_ADD(LOG_UINT16, recvSeq1, &statistic[1].recvSeq)
LOG_ADD(LOG_UINT16, recvNum1, &statistic[1].recvnum)
LOG_ADD(LOG_UINT16, compute1num1, &statistic[1].compute1num)
LOG_ADD(LOG_UINT16, compute2num1, &statistic[1].compute2num)
LOG_ADD(LOG_INT16, dist1, distanceTowards + 1)
LOG_ADD(LOG_UINT8, distSrc1, distanceSource + 1)
LOG_ADD(LOG_FLOAT, distReal1, distanceReal + 1)

LOG_ADD(LOG_UINT16, recvSeq2, &statistic[2].recvSeq)
LOG_ADD(LOG_UINT16, recvNum2, &statistic[2].recvnum)
LOG_ADD(LOG_UINT16, compute1num2, &statistic[2].compute1num)
LOG_ADD(LOG_UINT16, compute2num2, &statistic[2].compute2num)
LOG_ADD(LOG_INT16, dist2, distanceTowards + 2)
LOG_ADD(LOG_UINT8, distSrc2, distanceSource + 2)
LOG_ADD(LOG_FLOAT, distReal2, distanceReal + 2)

LOG_ADD(LOG_UINT16, recvSeq3, &statistic[3].recvSeq)
LOG_ADD(LOG_UINT16, recvNum3, &statistic[3].recvnum)
LOG_ADD(LOG_UINT16, compute1num3, &statistic[3].compute1num)
LOG_ADD(LOG_UINT16, compute2num3, &statistic[3].compute2num)
LOG_ADD(LOG_INT16, dist3, distanceTowards + 3)
LOG_ADD(LOG_UINT8, distSrc3, distanceSource + 3)
LOG_ADD(LOG_FLOAT, distReal3, distanceReal + 3)

LOG_ADD(LOG_UINT16, recvSeq4, &statistic[4].recvSeq)
LOG_ADD(LOG_UINT16, recvNum4, &statistic[4].recvnum)
LOG_ADD(LOG_UINT16, compute1num4, &statistic[4].compute1num)
LOG_ADD(LOG_UINT16, compute2num4, &statistic[4].compute2num)
LOG_ADD(LOG_INT16, dist4, distanceTowards + 4)
LOG_ADD(LOG_UINT8, distSrc4, distanceSource + 4)
LOG_ADD(LOG_FLOAT, distReal4, distanceReal + 4)

LOG_ADD(LOG_UINT16, recvSeq5, &statistic[5].recvSeq)
LOG_ADD(LOG_UINT16, recvNum5, &statistic[5].recvnum)
LOG_ADD(LOG_UINT16, compute1num5, &statistic[5].compute1num)
LOG_ADD(LOG_UINT16, compute2num5, &statistic[5].compute2num)
LOG_ADD(LOG_INT16, dist5, distanceTowards + 5)
LOG_ADD(LOG_UINT8, distSrc5, distanceSource + 5)
LOG_ADD(LOG_FLOAT, distReal5, distanceReal + 5)

LOG_ADD(LOG_UINT16, recvSeq6, &statistic[6].recvSeq)
LOG_ADD(LOG_UINT16, recvNum6, &statistic[6].recvnum)
LOG_ADD(LOG_UINT16, compute1num6, &statistic[6].compute1num)
LOG_ADD(LOG_UINT16, compute2num6, &statistic[6].compute2num)
LOG_ADD(LOG_INT16, dist6, distanceTowards + 6)
LOG_ADD(LOG_UINT8, distSrc6, distanceSource + 6)
LOG_ADD(LOG_FLOAT, distReal6, distanceReal + 6)

LOG_ADD(LOG_UINT16, recvSeq7, &statistic[7].recvSeq)
LOG_ADD(LOG_UINT16, recvNum7, &statistic[7].recvnum)
LOG_ADD(LOG_UINT16, compute1num7, &statistic[7].compute1num)
LOG_ADD(LOG_UINT16, compute2num7, &statistic[7].compute2num)
LOG_ADD(LOG_INT16, dist7, distanceTowards + 7)
LOG_ADD(LOG_UINT8, distSrc7, distanceSource + 7)
LOG_ADD(LOG_FLOAT, distReal7, distanceReal + 7)

LOG_ADD(LOG_UINT16, recvSeq8, &statistic[8].recvSeq)
LOG_ADD(LOG_UINT16, recvNum8, &statistic[8].recvnum)
LOG_ADD(LOG_UINT16, compute1num8, &statistic[8].compute1num)
LOG_ADD(LOG_UINT16, compute2num8, &statistic[8].compute2num)
LOG_ADD(LOG_INT16, dist8, distanceTowards + 8)
LOG_ADD(LOG_UINT8, distSrc8, distanceSource + 8)
LOG_ADD(LOG_FLOAT, distReal8, distanceReal + 8)

LOG_ADD(LOG_UINT16, recvSeq9, &statistic[9].recvSeq)
LOG_ADD(LOG_UINT16, recvNum9, &statistic[9].recvnum)
LOG_ADD(LOG_UINT16, compute1num9, &statistic[9].compute1num)
LOG_ADD(LOG_UINT16, compute2num9, &statistic[9].compute2num)
LOG_ADD(LOG_INT16, dist9, distanceTowards + 9)
LOG_ADD(LOG_UINT8, distSrc9, distanceSource + 9)
LOG_ADD(LOG_FLOAT, distReal9, distanceReal + 9)

LOG_ADD(LOG_UINT16, recvSeq10, &statistic[10].recvSeq)
LOG_ADD(LOG_UINT16, recvNum10, &statistic[10].recvnum)
LOG_ADD(LOG_UINT16, compute1num10, &statistic[10].compute1num)
LOG_ADD(LOG_UINT16, compute2num10, &statistic[10].compute2num)
LOG_ADD(LOG_INT16, dist10, distanceTowards + 10)
LOG_ADD(LOG_UINT8, distSrc10, distanceSource + 10)
LOG_ADD(LOG_FLOAT, distReal10, distanceReal + 10)

LOG_ADD(LOG_UINT16, recvSeq11, &statistic[11].recvSeq)
LOG_ADD(LOG_UINT16, recvNum11, &statistic[11].recvnum)
LOG_ADD(LOG_UINT16, compute1num11, &statistic[11].compute1num)
LOG_ADD(LOG_UINT16, compute2num11, &statistic[11].compute2num)
LOG_ADD(LOG_INT16, dist11, distanceTowards + 11)
LOG_ADD(LOG_UINT8, distSrc11, distanceSource + 11)
LOG_ADD(LOG_FLOAT, distReal11, distanceReal + 11)

LOG_ADD(LOG_UINT16, recvSeq12, &statistic[12].recvSeq)
LOG_ADD(LOG_UINT16, recvNum12, &statistic[12].recvnum)
LOG_ADD(LOG_UINT16, compute1num12, &statistic[12].compute1num)
LOG_ADD(LOG_UINT16, compute2num12, &statistic[12].compute2num)
LOG_ADD(LOG_INT16, dist12, distanceTowards + 12)
LOG_ADD(LOG_UINT8, distSrc12, distanceSource + 12)
LOG_ADD(LOG_FLOAT, distReal12, distanceReal + 12)

LOG_ADD(LOG_UINT16, recvSeq13, &statistic[13].recvSeq)
LOG_ADD(LOG_UINT16, recvNum13, &statistic[13].recvnum)
LOG_ADD(LOG_UINT16, compute1num13, &statistic[13].compute1num)
LOG_ADD(LOG_UINT16, compute2num13, &statistic[13].compute2num)
LOG_ADD(LOG_INT16, dist13, distanceTowards + 13)
LOG_ADD(LOG_UINT8, distSrc13, distanceSource + 13)
LOG_ADD(LOG_FLOAT, distReal13, distanceReal + 13)

LOG_ADD(LOG_UINT16, recvSeq14, &statistic[14].recvSeq)
LOG_ADD(LOG_UINT16, recvNum14, &statistic[14].recvnum)
LOG_ADD(LOG_UINT16, compute1num14, &statistic[14].compute1num)
LOG_ADD(LOG_UINT16, compute2num14, &statistic[14].compute2num)
LOG_ADD(LOG_INT16, dist14, distanceTowards + 14)
LOG_ADD(LOG_UINT8, distSrc14, distanceSource + 14)
LOG_ADD(LOG_FLOAT, distReal14, distanceReal + 14)

LOG_ADD(LOG_UINT16, recvSeq15, &statistic[15].recvSeq)
LOG_ADD(LOG_UINT16, recvNum15, &statistic[15].recvnum)
LOG_ADD(LOG_UINT16, compute1num15, &statistic[15].compute1num)
LOG_ADD(LOG_UINT16, compute2num15, &statistic[15].compute2num)
LOG_ADD(LOG_INT16, dist15, distanceTowards + 15)
LOG_ADD(LOG_UINT8, distSrc15, distanceSource + 15)
LOG_ADD(LOG_FLOAT, distReal15, distanceReal + 15)

LOG_ADD(LOG_UINT16, recvSeq16, &statistic[16].recvSeq)
LOG_ADD(LOG_UINT16, recvNum16, &statistic[16].recvnum)
LOG_ADD(LOG_UINT16, compute1num16, &statistic[16].compute1num)
LOG_ADD(LOG_UINT16, compute2num16, &statistic[16].compute2num)
LOG_ADD(LOG_INT16, dist16, distanceTowards + 16)
LOG_ADD(LOG_UINT8, distSrc16, distanceSource + 16)
LOG_ADD(LOG_FLOAT, distReal16, distanceReal + 16)

LOG_ADD(LOG_UINT16, recvSeq17, &statistic[17].recvSeq)
LOG_ADD(LOG_UINT16, recvNum17, &statistic[17].recvnum)
LOG_ADD(LOG_UINT16, compute1num17, &statistic[17].compute1num)
LOG_ADD(LOG_UINT16, compute2num17, &statistic[17].compute2num)
LOG_ADD(LOG_INT16, dist17, distanceTowards + 17)
LOG_ADD(LOG_UINT8, distSrc17, distanceSource + 17)
LOG_ADD(LOG_FLOAT, distReal17, distanceReal + 17)

LOG_ADD(LOG_UINT16, recvSeq18, &statistic[18].recvSeq)
LOG_ADD(LOG_UINT16, recvNum18, &statistic[18].recvnum)
LOG_ADD(LOG_UINT16, compute1num18, &statistic[18].compute1num)
LOG_ADD(LOG_UINT16, compute2num18, &statistic[18].compute2num)
LOG_ADD(LOG_INT16, dist18, distanceTowards + 18)
LOG_ADD(LOG_UINT8, distSrc18, distanceSource + 18)
LOG_ADD(LOG_FLOAT, distReal18, distanceReal + 18)

LOG_ADD(LOG_UINT16, recvSeq19, &statistic[19].recvSeq)
LOG_ADD(LOG_UINT16, recvNum19, &statistic[19].recvnum)
LOG_ADD(LOG_UINT16, compute1num19, &statistic[19].compute1num)
LOG_ADD(LOG_UINT16, compute2num19, &statistic[19].compute2num)
LOG_ADD(LOG_INT16, dist19, distanceTowards + 19)
LOG_ADD(LOG_UINT8, distSrc19, distanceSource + 19)
LOG_ADD(LOG_FLOAT, distReal19, distanceReal + 19)

LOG_ADD(LOG_UINT16, recvSeq20, &statistic[20].recvSeq)
LOG_ADD(LOG_UINT16, recvNum20, &statistic[20].recvnum)
LOG_ADD(LOG_UINT16, compute1num20, &statistic[20].compute1num)
LOG_ADD(LOG_UINT16, compute2num20, &statistic[20].compute2num)
LOG_ADD(LOG_INT16, dist20, distanceTowards + 20)
LOG_ADD(LOG_UINT8, distSrc20, distanceSource + 20)
LOG_ADD(LOG_FLOAT, distReal20, distanceReal + 20)

LOG_ADD(LOG_UINT16, recvSeq21, &statistic[21].recvSeq)
LOG_ADD(LOG_UINT16, recvNum21, &statistic[21].recvnum)
LOG_ADD(LOG_UINT16, compute1num21, &statistic[21].compute1num)
LOG_ADD(LOG_UINT16, compute2num21, &statistic[21].compute2num)
LOG_ADD(LOG_INT16, dist21, distanceTowards + 21)
LOG_ADD(LOG_UINT8, distSrc21, distanceSource + 21)
LOG_ADD(LOG_FLOAT, distReal21, distanceReal + 21)

LOG_ADD(LOG_UINT16, recvSeq22, &statistic[22].recvSeq)
LOG_ADD(LOG_UINT16, recvNum22, &statistic[22].recvnum)
LOG_ADD(LOG_UINT16, compute1num22, &statistic[22].compute1num)
LOG_ADD(LOG_UINT16, compute2num22, &statistic[22].compute2num)
LOG_ADD(LOG_INT16, dist22, distanceTowards + 22)
LOG_ADD(LOG_UINT8, distSrc22, distanceSource + 22)
LOG_ADD(LOG_FLOAT, distReal22, distanceReal + 22)

LOG_ADD(LOG_UINT16, recvSeq23, &statistic[23].recvSeq)
LOG_ADD(LOG_UINT16, recvNum23, &statistic[23].recvnum)
LOG_ADD(LOG_UINT16, compute1num23, &statistic[23].compute1num)
LOG_ADD(LOG_UINT16, compute2num23, &statistic[23].compute2num)
LOG_ADD(LOG_INT16, dist23, distanceTowards + 23)
LOG_ADD(LOG_UINT8, distSrc23, distanceSource + 23)
LOG_ADD(LOG_FLOAT, distReal23, distanceReal + 23)

LOG_ADD(LOG_UINT16, recvSeq24, &statistic[24].recvSeq)
LOG_ADD(LOG_UINT16, recvNum24, &statistic[24].recvnum)
LOG_ADD(LOG_UINT16, compute1num24, &statistic[24].compute1num)
LOG_ADD(LOG_UINT16, compute2num24, &statistic[24].compute2num)
LOG_ADD(LOG_INT16, dist24, distanceTowards + 24)
LOG_ADD(LOG_UINT8, distSrc24, distanceSource + 24)
LOG_ADD(LOG_FLOAT, distReal24, distanceReal + 24)

LOG_GROUP_STOP(Statistic)