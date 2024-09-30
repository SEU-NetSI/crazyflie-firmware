#include <math.h>
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "system.h"

#include "autoconf.h"
#include "debug.h"
#include "log.h"
#include "adhocdeck.h"
#include "swarm_ranging.h"
#include "routing.h"
#include "olsr.h"
#include "timers.h"
#include "static_mem.h"

#ifndef RANGING_DEBUG_ENABLE
#undef DEBUG_PRINT
#define DEBUG_PRINT
#endif

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

static uint16_t MY_UWB_ADDRESS;

static QueueHandle_t rxQueue;
static Neighbor_Set_t neighborSet;
static TimerHandle_t neighborSetEvictionTimer;
NO_DMA_CCM_SAFE_ZERO_INIT Ranging_Table_Set_t rangingTableSet;
static TimerHandle_t rangingTableSetEvictionTimer;
static UWB_Message_Listener_t listener;
static TaskHandle_t uwbRangingTxTaskHandle = 0;
static TaskHandle_t uwbRangingRxTaskHandle = 0;
static int TfBufferIndex = 0;
static Timestamp_Tuple_t TfBuffer[Tf_BUFFER_POOL_SIZE] = {0};
static SemaphoreHandle_t TfBufferMutex;
static int rangingSeqNumber = 1;
static logVarId_t idVelocityX, idVelocityY, idVelocityZ;
static logVarId_t idX, idY, idZ;
static float velocity;
static Ranging_Table_t EMPTY_RANGING_TABLE = {
    .neighborAddress = UWB_DEST_EMPTY,
    .Rp.timestamp.full = 0,
    .Rp.seqNumber = 0,
    .Tp.timestamp.full = 0,
    .Tp.seqNumber = 0,
    .Rf.timestamp.full = 0,
    .Rf.seqNumber = 0,
    .Tf.timestamp.full = 0,
    .Tf.seqNumber = 0,
    .Re.timestamp.full = 0,
    .Re.seqNumber = 0,
    .latestReceived.timestamp.full = 0,
    .latestReceived.seqNumber = 0,
    .TrRrBuffer.cur = 0,
    .TrRrBuffer.latest = 0,
    .state = RANGING_STATE_S1,
    .period = RANGING_PERIOD,
    .nextExpectedDeliveryTime = M2T(RANGING_PERIOD),
    .expirationTime = M2T(RANGING_TABLE_HOLD_TIME),
    .lastSendTime = 0,
    .distance = -1};

int16_t distanceTowards[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = -1};
uint8_t distanceSource[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = -1};
float distanceReal[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = -1};

typedef struct Stastistic
{
  uint16_t recvSeq;
  uint16_t recvnum;
  uint16_t compute1num;
  uint16_t compute2num;
  uint16_t compute3num;
} Stastistic;
static Stastistic statistic[NEIGHBOR_ADDRESS_MAX + 1];
static TimerHandle_t statisticTimer;

int16_t getDistance(UWB_Address_t neighborAddress)
{
  ASSERT(neighborAddress <= NEIGHBOR_ADDRESS_MAX);
  return distanceTowards[neighborAddress];
}

void setDistance(UWB_Address_t neighborAddress, int16_t distance, uint8_t source)
{
  ASSERT(neighborAddress <= NEIGHBOR_ADDRESS_MAX);
  distanceTowards[neighborAddress] = distance;
  distanceSource[neighborAddress] = source;
}

#ifdef ENABLE_OPTIMAL_RANGING_SCHEDULE
int rx_buffer_index = 0;
static Timestamp_Tuple_t rx_buffer[NEIGHBOR_ADDRESS_MAX + 1];
#define SAFETY_DISTANCE_MIN 2
int8_t temp_delay = 0;
void predict_period_in_rx(int rx_buffer_index)
{

  for (int i = 0; i < Tf_BUFFER_POOL_SIZE; i++)
  {
    if (TfBuffer[i].timestamp.full && rx_buffer[rx_buffer_index].timestamp.full)
    {
      /*
      +-------+------+-------+-------+-------+------+
      |  RX3  |  TX  |  RX1  |  RX2  |  RX3  |  TX  |
      +-------+------+-------+-------+-------+------+
                  ^              ^
                 Tf             now
      */

      if (((-TfBuffer[i].timestamp.full + rx_buffer[rx_buffer_index].timestamp.full) % UWB_MAX_TIMESTAMP < (uint64_t)(RANGING_PERIOD / (DWT_TIME_UNITS * 1000))) && (TfBuffer[i].timestamp.full % UWB_MAX_TIMESTAMP) < (rx_buffer[rx_buffer_index].timestamp.full % UWB_MAX_TIMESTAMP))
      {
        /*上一次TX时间 到 本次RX时间 太近*/
        if ((-TfBuffer[i].timestamp.full + rx_buffer[rx_buffer_index].timestamp.full) % UWB_MAX_TIMESTAMP < (uint64_t)(SAFETY_DISTANCE_MIN / (DWT_TIME_UNITS * 1000)))
        {
          temp_delay = -1;
          // keeping_times = 1;
          return;
        }

        /*本次RX时间 到 预测的下一次TX 太近*/
        if ((TfBuffer[i].timestamp.full + (uint64_t)(RANGING_PERIOD / (DWT_TIME_UNITS * 1000)) - rx_buffer[rx_buffer_index].timestamp.full) % UWB_MAX_TIMESTAMP < (uint64_t)(SAFETY_DISTANCE_MIN / (DWT_TIME_UNITS * 1000)))
        {
          temp_delay = +1;
          // keeping_times = 1;
          return;
        }
      }
    }
  }
}

void predict_period_in_tx_2(int TfBufferIndex)
{

  bool temp_control[2] = {0, 0};

  for (int i = 0; i < NEIGHBOR_ADDRESS_MAX; i++)
  {
    if (TfBuffer[TfBufferIndex].timestamp.full && rx_buffer[i].timestamp.full)
    {
      /*
      +-------+------+-------+-------+-------+------+
      |  RX3  |  TX  |  RX1  |  RX2  |  RX3  |  TX  |
      +-------+------+-------+-------+-------+------+
                                                ^
                                                now
      */
      if (((TfBuffer[TfBufferIndex].timestamp.full - rx_buffer[i].timestamp.full) % UWB_MAX_TIMESTAMP < (uint64_t)(RANGING_PERIOD / (DWT_TIME_UNITS * 1000))) && (TfBuffer[TfBufferIndex].timestamp.full % UWB_MAX_TIMESTAMP) > (rx_buffer[i].timestamp.full % UWB_MAX_TIMESTAMP))
      {
        if ((TfBuffer[TfBufferIndex].timestamp.full - rx_buffer[i].timestamp.full) % UWB_MAX_TIMESTAMP < (uint64_t)(RANGING_PERIOD / rangingTableSet.size / (DWT_TIME_UNITS * 1000)))
        {
          temp_control[0] = 1;
        }
      }

      if (((TfBuffer[TfBufferIndex].timestamp.full - rx_buffer[i].timestamp.full) % UWB_MAX_TIMESTAMP < (uint64_t)(RANGING_PERIOD / (DWT_TIME_UNITS * 1000))) && (TfBuffer[TfBufferIndex].timestamp.full % UWB_MAX_TIMESTAMP) > (rx_buffer[i].timestamp.full % UWB_MAX_TIMESTAMP))
      {
        if ((uint64_t)(RANGING_PERIOD / (DWT_TIME_UNITS * 1000) - TfBuffer[TfBufferIndex].timestamp.full + rx_buffer[i].timestamp.full) % UWB_MAX_TIMESTAMP < (uint64_t)(RANGING_PERIOD / rangingTableSet.size / (DWT_TIME_UNITS * 1000)))
        {
          temp_control[1] = 1;
        }
      }
    }
  }

  if (!temp_control[0] && temp_control[1])
  {
    temp_delay = -1;
  }
  if (temp_control[0] && !temp_control[1])
  {
    temp_delay = +1;
  }
}

#endif

void rangingTableBufferInit(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer)
{
  rangingTableBuffer->cur = 0;
  rangingTableBuffer->latest = 0;
  Timestamp_Tuple_t empty = {.seqNumber = 0, .timestamp.full = 0};
  for (set_index_t i = 0; i < Tr_Rr_BUFFER_POOL_SIZE; i++)
  {
    rangingTableBuffer->candidates[i].Tr = empty;
    rangingTableBuffer->candidates[i].Rr = empty;
  }
}

void rangingTableTxRxHistoryInit(Ranging_Table_Tx_Rx_History_t *history)
{
  Timestamp_Tuple_t empty = {.seqNumber = 0, .timestamp.full = 0};
  history->Tx = empty;
  history->Rx = empty;
}

void printStasticCallback(TimerHandle_t timer)
{
  DEBUG_PRINT("recvnum:%d,compute1num:%d,compute2num:%d\n",
              statistic[1].recvnum,
              statistic[1].compute1num,
              statistic[1].compute2num);
}

void statisticInit()
{
  for (int i = 0; i <= NEIGHBOR_ADDRESS_MAX; i++)
  {
    statistic[i].recvSeq = 0;
    statistic[i].recvnum = 0;
    statistic[i].compute1num = 0;
    statistic[i].compute2num = 0;
  }
  // statisticTimer = xTimerCreate("statisticTimer",
  //                               M2T(NEIGHBOR_SET_HOLD_TIME / 2),
  //                               pdTRUE,
  //                               (void *)0,
  //                               printStasticCallback);
  // xTimerStart(statisticTimer, M2T(0));
}

void rangingTableBufferUpdate(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer,
                              Timestamp_Tuple_t_2 Tr,
                              Timestamp_Tuple_t Rr)
{
  rangingTableBuffer->candidates[rangingTableBuffer->cur].Tr.seqNumber = Tr.seqNumber;
  rangingTableBuffer->candidates[rangingTableBuffer->cur].Tr.timestamp = Tr.timestamp;
  rangingTableBuffer->candidates[rangingTableBuffer->cur].Rr = Rr;
  // shift
  rangingTableBuffer->latest = rangingTableBuffer->cur;
  rangingTableBuffer->cur = (rangingTableBuffer->cur + 1) % Tr_Rr_BUFFER_POOL_SIZE;
}

Ranging_Table_Tr_Rr_Candidate_t rangingTableBufferGetCandidate(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer,
                                                               Timestamp_Tuple_t Tf, Timestamp_Tuple_t Tp)
{
  set_index_t index = rangingTableBuffer->latest;
  uint64_t rightBound = Tf.timestamp.full % UWB_MAX_TIMESTAMP;
  uint64_t leftBound = Tp.timestamp.full % UWB_MAX_TIMESTAMP;
  Ranging_Table_Tr_Rr_Candidate_t candidate = {.Rr.timestamp.full = 0, .Tr.timestamp.full = 0};

  for (int count = 0; count < Tr_Rr_BUFFER_POOL_SIZE; count++)
  {
    if (rangingTableBuffer->candidates[index].Rr.timestamp.full &&
        rangingTableBuffer->candidates[index].Rr.timestamp.full % UWB_MAX_TIMESTAMP < rightBound &&
        rangingTableBuffer->candidates[index].Rr.timestamp.full % UWB_MAX_TIMESTAMP > leftBound &&
        rangingTableBuffer->candidates[index].Rr.seqNumber == rangingTableBuffer->candidates[index].Tr.seqNumber)
    {
      candidate.Tr = rangingTableBuffer->candidates[index].Tr;
      candidate.Rr = rangingTableBuffer->candidates[index].Rr;
      break;
    }
    index = (index - 1 + Tr_Rr_BUFFER_POOL_SIZE) % Tr_Rr_BUFFER_POOL_SIZE;
  }

  return candidate;
}

Ranging_Table_Tr_Rr_Candidate_t rangingTableBufferGetLatest(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer)
{

  Ranging_Table_Tr_Rr_Candidate_t candidate = {.Rr.timestamp.full = 0, .Tr.timestamp.full = 0};
  int index = rangingTableBuffer->latest;
  candidate.Tr = rangingTableBuffer->candidates[index].Tr;
  candidate.Rr = rangingTableBuffer->candidates[index].Rr;

  return candidate;
}

void updateTfBuffer(Timestamp_Tuple_t timestamp)
{
  xSemaphoreTake(TfBufferMutex, portMAX_DELAY);
  TfBufferIndex++;
  TfBufferIndex %= Tf_BUFFER_POOL_SIZE;
  TfBuffer[TfBufferIndex] = timestamp;
#ifdef ENABLE_OPTIMAL_RANGING_SCHEDULE
  predict_period_in_tx_2(TfBufferIndex);
#endif
  //  DEBUG_PRINT("updateTfBuffer: time = %llu, seq = %d\n", TfBuffer[TfBufferIndex].timestamp.full, TfBuffer[TfBufferIndex].seqNumber);
  xSemaphoreGive(TfBufferMutex);
}

Timestamp_Tuple_t findTfBySeqNumber(uint16_t seqNumber)
{
  xSemaphoreTake(TfBufferMutex, portMAX_DELAY);
  Timestamp_Tuple_t Tf = {.timestamp.full = 0, .seqNumber = 0};
  int startIndex = TfBufferIndex;
  /* Backward search */
  for (int i = startIndex; i >= 0; i--)
  {
    if (TfBuffer[i].seqNumber == seqNumber)
    {
      Tf = TfBuffer[i];
      break;
    }
  }
  if (!Tf.timestamp.full)
  {
    /* Forward search */
    for (int i = startIndex + 1; i < Tf_BUFFER_POOL_SIZE; i++)
    {
      if (TfBuffer[i].seqNumber == seqNumber)
      {
        Tf = TfBuffer[i];
        break;
      }
    }
  }
  xSemaphoreGive(TfBufferMutex);
  return Tf;
}

Timestamp_Tuple_t getLatestTxTimestamp()
{
  return TfBuffer[TfBufferIndex];
}

// void getLatestNTxTimestamps(Timestamp_Tuple_t_2 *timestamps, int n)
// {
//   ASSERT(n <= Tf_BUFFER_POOL_SIZE);
//   xSemaphoreTake(TfBufferMutex, portMAX_DELAY);
//   int startIndex = (TfBufferIndex + 1 - n + Tf_BUFFER_POOL_SIZE) % Tf_BUFFER_POOL_SIZE;
//   for (int i = n - 1; i >= 0; i--)
//   {
//     timestamps[i].timestamp = TfBuffer[startIndex].timestamp;
//     timestamps[i].seqNumber = TfBuffer[startIndex].seqNumber;
//     startIndex = (startIndex + 1) % Tf_BUFFER_POOL_SIZE;
//   }
//   xSemaphoreGive(TfBufferMutex);
// }

Ranging_Table_Set_t *getGlobalRangingTableSet()
{
  return &rangingTableSet;
}

void rangingTableInit(Ranging_Table_t *table, UWB_Address_t neighborAddress)
{
  memset(table, 0, sizeof(Ranging_Table_t));
  table->state = RANGING_STATE_S1;
  table->neighborAddress = neighborAddress;
  table->period = RANGING_PERIOD;
  table->nextExpectedDeliveryTime = 0;
  table->expirationTime = 0;
  table->lastSendTime = 0;
  rangingTableBufferInit(&table->TrRrBuffer); // Can be safely removed this line since memset() is called
  rangingTableTxRxHistoryInit(&table->TxRxHistory);
}

/* Ranging Table Set Operations */
void rangingTableSetInit(Ranging_Table_Set_t *set)
{
  set->mu = xSemaphoreCreateMutex();
  set->size = 0;
  for (int i = 0; i < RANGING_TABLE_SIZE_MAX; i++)
  {
    set->tables[i] = EMPTY_RANGING_TABLE;
  }
}

static void rangingTableSetSwapTable(Ranging_Table_Set_t *set, int first, int second)
{
  Ranging_Table_t temp = set->tables[first];
  set->tables[first] = set->tables[second];
  set->tables[second] = temp;
}

static int rangingTableSetSearchTable(Ranging_Table_Set_t *set, UWB_Address_t targetAddress)
{
  /* Binary Search */
  int left = -1, right = set->size, res = -1;
  while (left + 1 != right)
  {
    int mid = left + (right - left) / 2;
    if (set->tables[mid].neighborAddress == targetAddress)
    {
      res = mid;
      break;
    }
    else if (set->tables[mid].neighborAddress > targetAddress)
    {
      right = mid;
    }
    else
    {
      left = mid;
    }
  }
  return res;
}

typedef int (*rangingTableCompareFunc)(Ranging_Table_t *, Ranging_Table_t *);

static int COMPARE_BY_ADDRESS(Ranging_Table_t *first, Ranging_Table_t *second)
{
  if (first->neighborAddress == second->neighborAddress)
  {
    return 0;
  }
  if (first->neighborAddress > second->neighborAddress)
  {
    return 1;
  }
  return -1;
}

static int COMPARE_BY_EXPIRATION_TIME(Ranging_Table_t *first, Ranging_Table_t *second)
{
  if (first->expirationTime == second->expirationTime)
  {
    return 0;
  }
  if (first->expirationTime > second->expirationTime)
  {
    return -1;
  }
  return 1;
}

static int COMPARE_BY_NEXT_EXPECTED_DELIVERY_TIME(Ranging_Table_t *first, Ranging_Table_t *second)
{
  if (first->nextExpectedDeliveryTime == second->nextExpectedDeliveryTime)
  {
    return 0;
  }
  if (first->nextExpectedDeliveryTime > second->nextExpectedDeliveryTime)
  {
    return 1;
  }
  return -1;
}

static int COMPARE_BY_LAST_SEND_TIME(Ranging_Table_t *first, Ranging_Table_t *second)
{
  if (first->lastSendTime == second->lastSendTime)
  {
    return 0;
  }
  if (first->lastSendTime > second->lastSendTime)
  {
    return 1;
  }
  return -1;
}

/* Build the heap */
static void rangingTableSetArrange(Ranging_Table_Set_t *set, int index, int len, rangingTableCompareFunc compare)
{
  int leftChild = 2 * index + 1;
  int rightChild = 2 * index + 2;
  int maxIndex = index;
  if (leftChild < len && compare(&set->tables[maxIndex], &set->tables[leftChild]) < 0)
  {
    maxIndex = leftChild;
  }
  if (rightChild < len && compare(&set->tables[maxIndex], &set->tables[rightChild]) < 0)
  {
    maxIndex = rightChild;
  }
  if (maxIndex != index)
  {
    rangingTableSetSwapTable(set, index, maxIndex);
    rangingTableSetArrange(set, maxIndex, len, compare);
  }
}

/* Sort the ranging table */
static void rangingTableSetRearrange(Ranging_Table_Set_t *set, rangingTableCompareFunc compare)
{
  /* Build max heap */
  for (int i = set->size / 2 - 1; i >= 0; i--)
  {
    rangingTableSetArrange(set, i, set->size, compare);
  }
  for (int i = set->size - 1; i >= 0; i--)
  {
    rangingTableSetSwapTable(set, 0, i);
    rangingTableSetArrange(set, 0, i, compare);
  }
}

static int rangingTableSetClearExpire(Ranging_Table_Set_t *set)
{
  Time_t curTime = xTaskGetTickCount();
  int evictionCount = 0;

  for (int i = 0; i < rangingTableSet.size; i++)
  {
    if (rangingTableSet.tables[i].expirationTime <= curTime)
    {
      DEBUG_PRINT("rangingTableSetClearExpire: Clean ranging table for neighbor %u that expire at %lu.\n",
                  rangingTableSet.tables[i].neighborAddress,
                  rangingTableSet.tables[i].expirationTime);
      setDistance(rangingTableSet.tables[i].neighborAddress, -1, -1);
      rangingTableSet.tables[i] = EMPTY_RANGING_TABLE;
      evictionCount++;
    }
  }
  /* Keeps ranging table set in order. */
  rangingTableSetRearrange(&rangingTableSet, COMPARE_BY_ADDRESS);
  rangingTableSet.size -= evictionCount;

  return evictionCount;
}

static void rangingTableSetClearExpireTimerCallback(TimerHandle_t timer)
{
  xSemaphoreTake(rangingTableSet.mu, portMAX_DELAY);

  Time_t curTime = xTaskGetTickCount();
  DEBUG_PRINT("rangingTableSetClearExpireTimerCallback: Trigger expiration timer at %lu.\n", curTime);

  int evictionCount = rangingTableSetClearExpire(&rangingTableSet);
  if (evictionCount > 0)
  {
    DEBUG_PRINT("rangingTableSetClearExpireTimerCallback: Evict total %d ranging tables.\n", evictionCount);
  }
  else
  {
    DEBUG_PRINT("rangingTableSetClearExpireTimerCallback: Evict none.\n");
  }

  xSemaphoreGive(rangingTableSet.mu);
}

bool rangingTableSetAddTable(Ranging_Table_Set_t *set, Ranging_Table_t table)
{
  int index = rangingTableSetSearchTable(set, table.neighborAddress);
  if (index != -1)
  {
    DEBUG_PRINT(
        "rangingTableSetAddTable: Try to add an already added ranging table for neighbor %u, update it instead.\n",
        table.neighborAddress);
    set->tables[index] = table;
    return true;
  }
  /* If ranging table is full now and there is no expired ranging table, then ignore. */
  if (set->size == RANGING_TABLE_SIZE_MAX && rangingTableSetClearExpire(&rangingTableSet) == 0)
  {
    DEBUG_PRINT("rangingTableSetAddTable: Ranging table if full, ignore new neighbor %u.\n",
                table.neighborAddress);
    return false;
  }
  /* Add the new entry to the last */
  uint8_t curIndex = set->size;
  set->tables[curIndex] = table;
  set->size++;
  /* Sort the ranging table, keep it in order. */
  rangingTableSetRearrange(set, COMPARE_BY_ADDRESS);
  DEBUG_PRINT("rangingTableSetAddTable: Add new neighbor %u to ranging table.\n", table.neighborAddress);
  return true;
}

void rangingTableSetUpdateTable(Ranging_Table_Set_t *set, Ranging_Table_t table)
{
  int index = rangingTableSetSearchTable(set, table.neighborAddress);
  if (index == -1)
  {
    DEBUG_PRINT("rangingTableSetUpdateTable: Cannot find correspond table for neighbor %u, add it instead.\n",
                table.neighborAddress);
    rangingTableSetAddTable(set, table);
  }
  else
  {
    set->tables[index] = table;
    DEBUG_PRINT("rangingTableSetUpdateTable: Update table for neighbor %u.\n", table.neighborAddress);
  }
}

void rangingTableSetRemoveTable(Ranging_Table_Set_t *set, UWB_Address_t neighborAddress)
{
  if (set->size == 0)
  {
    DEBUG_PRINT("rangingTableSetRemoveTable: Ranging table is empty, ignore.\n");
    return;
  }
  int index = rangingTableSetSearchTable(set, neighborAddress);
  if (index == -1)
  {
    DEBUG_PRINT("rangingTableSetRemoveTable: Cannot find correspond table for neighbor %u, ignore.\n", neighborAddress);
    return;
  }
  rangingTableSetSwapTable(set, index, set->size - 1);
  set->tables[set->size - 1] = EMPTY_RANGING_TABLE;
  set->size--;
  rangingTableSetRearrange(set, COMPARE_BY_ADDRESS);
}

Ranging_Table_t rangingTableSetFindTable(Ranging_Table_Set_t *set, UWB_Address_t neighborAddress)
{
  int index = rangingTableSetSearchTable(set, neighborAddress);
  Ranging_Table_t table = EMPTY_RANGING_TABLE;
  if (index == -1)
  {
    DEBUG_PRINT("rangingTableSetFindTable: Cannot find correspond table for neighbor %u.\n", neighborAddress);
  }
  else
  {
    table = set->tables[index];
  }
  return table;
}

void neighborBitSetInit(Neighbor_Bit_Set_t *bitSet)
{
  bitSet->bits = 0;
  bitSet->size = 0;
}

void neighborBitSetAdd(Neighbor_Bit_Set_t *bitSet, UWB_Address_t neighborAddress)
{
  ASSERT(neighborAddress <= NEIGHBOR_ADDRESS_MAX);
  uint64_t prevBits = bitSet->bits;
  bitSet->bits |= (1ULL << neighborAddress);
  if (prevBits != bitSet->bits)
  {
    bitSet->size++;
  }
}

void neighborBitSetRemove(Neighbor_Bit_Set_t *bitSet, UWB_Address_t neighborAddress)
{
  ASSERT(neighborAddress <= NEIGHBOR_ADDRESS_MAX);
  uint64_t prevBits = bitSet->bits;
  bitSet->bits &= ~(1ULL << neighborAddress);
  if (prevBits != bitSet->bits)
  {
    bitSet->size--;
  }
}

void neighborBitSetClear(Neighbor_Bit_Set_t *bitSet)
{
  bitSet->bits = 0;
  bitSet->size = 0;
}

bool neighborBitSetHas(Neighbor_Bit_Set_t *bitSet, UWB_Address_t neighborAddress)
{
  ASSERT(neighborAddress <= NEIGHBOR_ADDRESS_MAX);
  return (bitSet->bits & (1ULL << neighborAddress)) != 0;
}

Neighbor_Set_t *getGlobalNeighborSet()
{
  return &neighborSet;
}

void neighborSetInit(Neighbor_Set_t *set)
{
  set->size = 0;
  set->mu = xSemaphoreCreateMutex();
  neighborBitSetInit(&set->oneHop);
  neighborBitSetInit(&set->twoHop);
  set->neighborNewHooks.hook = NULL;
  set->neighborNewHooks.next = NULL;
  set->neighborExpirationHooks.hook = NULL;
  set->neighborExpirationHooks.next = NULL;
  set->neighborTopologyChangeHooks.hook = NULL;
  set->neighborTopologyChangeHooks.next = NULL;
  for (UWB_Address_t neighborAddress = 0; neighborAddress <= NEIGHBOR_ADDRESS_MAX; neighborAddress++)
  {
    set->expirationTime[neighborAddress] = 0;
    neighborBitSetInit(&set->twoHopReachSets[neighborAddress]);
  }
}

bool neighborSetHas(Neighbor_Set_t *set, UWB_Address_t neighborAddress)
{
  ASSERT(neighborAddress <= NEIGHBOR_ADDRESS_MAX);
  return neighborBitSetHas(&set->oneHop, neighborAddress) || neighborBitSetHas(&set->twoHop, neighborAddress);
}

bool neighborSetHasOneHop(Neighbor_Set_t *set, UWB_Address_t neighborAddress)
{
  ASSERT(neighborAddress <= NEIGHBOR_ADDRESS_MAX);
  return neighborBitSetHas(&set->oneHop, neighborAddress);
}

bool neighborSetHasTwoHop(Neighbor_Set_t *set, UWB_Address_t neighborAddress)
{
  ASSERT(neighborAddress <= NEIGHBOR_ADDRESS_MAX);
  return neighborBitSetHas(&set->twoHop, neighborAddress);
}

void neighborSetAddOneHopNeighbor(Neighbor_Set_t *set, UWB_Address_t neighborAddress)
{
  ASSERT(neighborAddress <= NEIGHBOR_ADDRESS_MAX);
  bool isNewNeighbor = false;
  if (!neighborSetHas(set, neighborAddress))
  {
    isNewNeighbor = true;
  }
  /* If neighbor is previous two-hop neighbor, remove it from two-hop neighbor set. */
  if (neighborSetHasTwoHop(set, neighborAddress))
  {
    neighborSetRemoveNeighbor(set, neighborAddress);
  }
  /* Add one-hop neighbor. */
  if (!neighborSetHasOneHop(set, neighborAddress))
  {
    neighborBitSetAdd(&set->oneHop, neighborAddress);
    neighborSetUpdateExpirationTime(set, neighborAddress);
    neighborSetHooksInvoke(&set->neighborTopologyChangeHooks, neighborAddress);
  }
  set->size = set->oneHop.size + set->twoHop.size;
  if (isNewNeighbor)
  {
    neighborSetHooksInvoke(&set->neighborNewHooks, neighborAddress);
  }
}

void neighborSetAddTwoHopNeighbor(Neighbor_Set_t *set, UWB_Address_t neighborAddress)
{
  ASSERT(neighborAddress <= NEIGHBOR_ADDRESS_MAX);
  bool isNewNeighbor = false;
  if (!neighborSetHas(set, neighborAddress))
  {
    isNewNeighbor = true;
  }
  /* If neighbor is previous one-hop neighbor, remove it from one-hop neighbor set. */
  if (neighborSetHasOneHop(set, neighborAddress))
  {
    neighborSetRemoveNeighbor(set, neighborAddress);
  }
  if (!neighborSetHasTwoHop(set, neighborAddress))
  {
    /* Add two-hop neighbor. */
    neighborBitSetAdd(&set->twoHop, neighborAddress);
    neighborSetUpdateExpirationTime(set, neighborAddress);
    neighborSetHooksInvoke(&set->neighborTopologyChangeHooks, neighborAddress);
  }
  set->size = set->oneHop.size + set->twoHop.size;
  if (isNewNeighbor)
  {
    neighborSetHooksInvoke(&set->neighborNewHooks, neighborAddress);
  }
}

void neighborSetRemoveNeighbor(Neighbor_Set_t *set, UWB_Address_t neighborAddress)
{
  ASSERT(neighborAddress <= NEIGHBOR_ADDRESS_MAX);
  if (neighborSetHas(set, neighborAddress))
  {
    if (neighborSetHasOneHop(set, neighborAddress) && neighborSetHasTwoHop(set, neighborAddress))
    {
      ASSERT(0); // impossible
    }
    set->expirationTime[neighborAddress] = 0;
    if (neighborSetHasOneHop(set, neighborAddress))
    {
      neighborBitSetRemove(&set->oneHop, neighborAddress);
      /* Remove related path to two-hop neighbor */
      for (UWB_Address_t twoHopNeighbor = 0; twoHopNeighbor <= NEIGHBOR_ADDRESS_MAX; twoHopNeighbor++)
      {
        if (neighborSetHasRelation(set, neighborAddress, twoHopNeighbor))
        {
          neighborSetRemoveRelation(set, neighborAddress, twoHopNeighbor);
        }
      }
    }
    else if (neighborSetHasTwoHop(set, neighborAddress))
    {
      neighborBitSetRemove(&set->twoHop, neighborAddress);
      /* Clear related two-hop reach set */
      neighborBitSetClear(&set->twoHopReachSets[neighborAddress]);
    }
    else
    {
      ASSERT(0); // impossible
    }
    neighborSetHooksInvoke(&set->neighborTopologyChangeHooks, neighborAddress);
  }
  set->size = set->oneHop.size + set->twoHop.size;
}

bool neighborSetHasRelation(Neighbor_Set_t *set, UWB_Address_t from, UWB_Address_t to)
{
  ASSERT(from <= NEIGHBOR_ADDRESS_MAX);
  ASSERT(to <= NEIGHBOR_ADDRESS_MAX);
  return neighborBitSetHas(&set->twoHopReachSets[to], from);
}

void neighborSetAddRelation(Neighbor_Set_t *set, UWB_Address_t from, UWB_Address_t to)
{
  ASSERT(from <= NEIGHBOR_ADDRESS_MAX);
  ASSERT(to <= NEIGHBOR_ADDRESS_MAX);
  if (!neighborBitSetHas(&set->twoHopReachSets[to], from))
  {
    neighborBitSetAdd(&set->twoHopReachSets[to], from);
    neighborSetHooksInvoke(&set->neighborTopologyChangeHooks, from);
  }
}

void neighborSetRemoveRelation(Neighbor_Set_t *set, UWB_Address_t from, UWB_Address_t to)
{
  ASSERT(from <= NEIGHBOR_ADDRESS_MAX);
  ASSERT(to <= NEIGHBOR_ADDRESS_MAX);
  if (neighborBitSetHas(&set->twoHopReachSets[to], from))
  {
    neighborBitSetRemove(&set->twoHopReachSets[to], from);
    neighborSetHooksInvoke(&set->neighborTopologyChangeHooks, from);
  }
}

void neighborSetRegisterNewNeighborHook(Neighbor_Set_t *set, neighborSetHook hook)
{
  ASSERT(hook);
  Neighbor_Set_Hooks_t cur = {
      .hook = hook,
      .next = (struct Neighbor_Set_Hook_Node *)set->neighborNewHooks.hook};
  set->neighborNewHooks = cur;
}

void neighborSetRegisterExpirationHook(Neighbor_Set_t *set, neighborSetHook hook)
{
  ASSERT(hook);
  Neighbor_Set_Hooks_t cur = {
      .hook = hook,
      .next = (struct Neighbor_Set_Hook_Node *)set->neighborExpirationHooks.hook};
  set->neighborExpirationHooks = cur;
}

void neighborSetRegisterTopologyChangeHook(Neighbor_Set_t *set, neighborSetHook hook)
{
  ASSERT(hook);
  Neighbor_Set_Hooks_t cur = {
      .hook = hook,
      .next = (struct Neighbor_Set_Hook_Node *)set->neighborTopologyChangeHooks.hook};
  set->neighborTopologyChangeHooks = cur;
}

void neighborSetHooksInvoke(Neighbor_Set_Hooks_t *hooks, UWB_Address_t neighborAddress)
{
  neighborSetHook cur = hooks->hook;
  while (cur != NULL)
  {
    DEBUG_PRINT("neighborSetHooksInvoke: Invoke neighbor set hook.\n");
    cur(neighborAddress);
    cur = (neighborSetHook)hooks->next;
  }
}

void neighborSetUpdateExpirationTime(Neighbor_Set_t *set, UWB_Address_t neighborAddress)
{
  ASSERT(neighborAddress <= NEIGHBOR_ADDRESS_MAX);
  set->expirationTime[neighborAddress] = xTaskGetTickCount() + M2T(NEIGHBOR_SET_HOLD_TIME);
}

int neighborSetClearExpire(Neighbor_Set_t *set)
{
  Time_t curTime = xTaskGetTickCount();
  int evictionCount = 0;
  for (UWB_Address_t neighborAddress = 0; neighborAddress <= NEIGHBOR_ADDRESS_MAX; neighborAddress++)
  {
    if (neighborSetHas(set, neighborAddress) && set->expirationTime[neighborAddress] <= curTime)
    {
      evictionCount++;
      neighborSetRemoveNeighbor(set, neighborAddress);
      DEBUG_PRINT("neighborSetClearExpire: neighbor %u expire at %lu.\n", neighborAddress, curTime);
      neighborSetHooksInvoke(&set->neighborExpirationHooks, neighborAddress);
    }
  }
  return evictionCount;
}

// static void topologySensing(Ranging_Message_t *rangingMessage)
// {
//   //  DEBUG_PRINT("topologySensing: Received ranging message from neighbor %u.\n", rangingMessage->header.srcAddress);
//   UWB_Address_t neighborAddress = rangingMessage->header.srcAddress;
//   if (!neighborSetHasOneHop(&neighborSet, neighborAddress))
//   {
//     /* Add current neighbor to one-hop neighbor set. */
//     neighborSetAddOneHopNeighbor(&neighborSet, neighborAddress);
//   }
//   neighborSetUpdateExpirationTime(&neighborSet, neighborAddress);

//   /* Infer one-hop and tow-hop neighbors from received ranging message. */
//   uint8_t bodyUnitCount = (rangingMessage->header.msgLength - sizeof(Ranging_Message_Header_t)) / sizeof(Body_Unit_t);
//   for (int i = 0; i < bodyUnitCount; i++)
//   {
// #ifdef ROUTING_OLSR_ENABLE
//     if (rangingMessage->bodyUnits[i].address == uwbGetAddress())
//     {
//       /* If been selected as MPR, add neighbor to mpr selector set. */
//       if (rangingMessage->bodyUnits[i].flags.MPR)
//       {
//         if (!mprSelectorSetHas(getGlobalMPRSelectorSet(), neighborAddress))
//         {
//           mprSelectorSetAdd(getGlobalMPRSelectorSet(), neighborAddress);
//         }
//         mprSelectorSetUpdateExpirationTime(getGlobalMPRSelectorSet(), neighborAddress);
//       }
//       else
//       {
//         if (mprSelectorSetHas(getGlobalMPRSelectorSet(), neighborAddress))
//         {
//           mprSelectorSetRemove(getGlobalMPRSelectorSet(), neighborAddress);
//         }
//       }
//     }
// #endif
//     UWB_Address_t twoHopNeighbor = rangingMessage->bodyUnits[i].address;
//     if (twoHopNeighbor != uwbGetAddress() && !neighborSetHasOneHop(&neighborSet, twoHopNeighbor))
//     {
//       /* If it is not one-hop neighbor then it is now my two-hop neighbor, if new add it to neighbor set. */
//       if (!neighborSetHasTwoHop(&neighborSet, twoHopNeighbor))
//       {
//         neighborSetAddTwoHopNeighbor(&neighborSet, twoHopNeighbor);
//       }
//       if (!neighborSetHasRelation(&neighborSet, neighborAddress, twoHopNeighbor))
//       {
//         neighborSetAddRelation(&neighborSet, neighborAddress, twoHopNeighbor);
//       }
//       neighborSetUpdateExpirationTime(&neighborSet, twoHopNeighbor);
//     }
//   }
// }

static void neighborSetClearExpireTimerCallback(TimerHandle_t timer)
{
  xSemaphoreTake(neighborSet.mu, portMAX_DELAY);

  Time_t curTime = xTaskGetTickCount();
  DEBUG_PRINT("neighborSetClearExpireTimerCallback: Trigger expiration timer at %lu.\n", curTime);

  int evictionCount = neighborSetClearExpire(&neighborSet);
  if (evictionCount > 0)
  {
    DEBUG_PRINT("neighborSetClearExpireTimerCallback: Evict total %d neighbors.\n", evictionCount);
  }
  else
  {
    DEBUG_PRINT("neighborSetClearExpireTimerCallback: Evict none.\n");
  }

  xSemaphoreGive(neighborSet.mu);
}

void printRangingTable(Ranging_Table_t *table)
{
  DEBUG_PRINT("Rp = %u, Tr = %u, Rf = %u, \n",
              table->Rp.seqNumber,
              table->TrRrBuffer.candidates[table->TrRrBuffer.latest].Tr.seqNumber,
              table->Rf.seqNumber);
  DEBUG_PRINT("Tp = %u, Rr = %u, Tf = %u, Re = %u, \n",
              table->Tp.seqNumber,
              table->TrRrBuffer.candidates[table->TrRrBuffer.latest].Rr.seqNumber,
              table->Tf.seqNumber,
              table->Re.seqNumber);
  DEBUG_PRINT("\n");
}

void printRangingTableSet(Ranging_Table_Set_t *set)
{
  DEBUG_PRINT("neighbor\t distance\t period\t expire\t \n");
  for (int i = 0; i < set->size; i++)
  {
    if (set->tables[i].neighborAddress == UWB_DEST_EMPTY)
    {
      continue;
    }
    DEBUG_PRINT("%u\t %d\t %lu\t %lu\t \n",
                set->tables[i].neighborAddress,
                set->tables[i].distance,
                set->tables[i].period,
                set->tables[i].expirationTime);
  }
  DEBUG_PRINT("---\n");
}

void printRangingMessage(Ranging_Message_t *rangingMessage)
{
  // for (int i = 0; i < RANGING_MAX_Tr_UNIT; i++)
  // {
  //   DEBUG_PRINT("lastTxTimestamp %d seq=%u, lastTxTimestamp=%2x%8lx\n",
  //               i,
  //               rangingMessage->header.lastTxTimestamps[i].seqNumber,
  //               rangingMessage->header.lastTxTimestamps[i].timestamp.high8,
  //               rangingMessage->header.lastTxTimestamps[i].timestamp.low32);
  // }
  if (rangingMessage->header.msgLength - sizeof(Ranging_Message_Header_t) == 0)
  {
    return;
  }
  uint16_t body_unit_number = (rangingMessage->header.msgLength - sizeof(Ranging_Message_Header_t)) / sizeof(Body_Unit_t);
  if (body_unit_number >= RANGING_MAX_BODY_UNIT)
  {
    DEBUG_PRINT("printRangingMessage: malformed body unit number.\n");
    return;
  }
  for (int i = 0; i < body_unit_number; i++)
  {
    DEBUG_PRINT("unitAddress=%u, Seq=%u\n",
                rangingMessage->bodyUnits[i].address,
                rangingMessage->bodyUnits[i].seqNumber);
  }
}

void printNeighborBitSet(Neighbor_Bit_Set_t *bitSet)
{
  DEBUG_PRINT("%u has %u neighbors = ", uwbGetAddress(), bitSet->size);
  for (int neighborAddress = 0; neighborAddress <= NEIGHBOR_ADDRESS_MAX; neighborAddress++)
  {
    if (neighborBitSetHas(bitSet, neighborAddress))
    {
      DEBUG_PRINT("%u ", neighborAddress);
    }
  }
  DEBUG_PRINT("\n");
}

void printNeighborSet(Neighbor_Set_t *set)
{
  DEBUG_PRINT("%u has %u one hop neighbors, %u two hop neighbors, %u neighbors in total.\n",
              uwbGetAddress(),
              set->oneHop.size,
              set->twoHop.size,
              set->size);
  DEBUG_PRINT("one-hop neighbors = ");
  for (UWB_Address_t oneHopNeighbor = 0; oneHopNeighbor <= NEIGHBOR_ADDRESS_MAX; oneHopNeighbor++)
  {
    if (neighborBitSetHas(&set->oneHop, oneHopNeighbor))
    {
      DEBUG_PRINT("%u ", oneHopNeighbor);
    }
  }
  DEBUG_PRINT("\n");
  DEBUG_PRINT("two-hop neighbors = ");
  for (UWB_Address_t twoHopNeighbor = 0; twoHopNeighbor <= NEIGHBOR_ADDRESS_MAX; twoHopNeighbor++)
  {
    if (neighborBitSetHas(&set->twoHop, twoHopNeighbor))
    {
      DEBUG_PRINT("%u ", twoHopNeighbor);
    }
  }
  DEBUG_PRINT("\n");
  for (UWB_Address_t twoHopNeighbor = 0; twoHopNeighbor <= NEIGHBOR_ADDRESS_MAX; twoHopNeighbor++)
  {
    if (!neighborBitSetHas(&set->twoHop, twoHopNeighbor))
    {
      continue;
    }
    DEBUG_PRINT("to two-hop neighbor %u: ", twoHopNeighbor);
    for (UWB_Address_t oneHopNeighbor = 0; oneHopNeighbor <= NEIGHBOR_ADDRESS_MAX; oneHopNeighbor++)
    {
      if (neighborSetHasRelation(set, oneHopNeighbor, twoHopNeighbor))
      {
        DEBUG_PRINT("%u ", oneHopNeighbor);
      }
    }
    DEBUG_PRINT("\n");
  }
}

static int16_t computeDistance(Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp,
                               Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr,
                               Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf)
{

  bool isErrorOccurred = false;

  DEBUG_PRINT("Tp:%d,Rp:%d,Tr:%d,Rr:%d,Tf:%d,Rf:%d\n", Tp.seqNumber, Rp.seqNumber, Tr.seqNumber, Rr.seqNumber, Tf.seqNumber, Rf.seqNumber);
  if (Tp.seqNumber != Rp.seqNumber || Tr.seqNumber != Rr.seqNumber || Tf.seqNumber != Rf.seqNumber)
  {
    // DEBUG_PRINT("Tp:%d,Rp:%d,Tr:%d,Rr:%d,Tf:%d,Rf:%d\n", Tp.seqNumber, Rp.seqNumber, Tr.seqNumber, Rr.seqNumber, Tf.seqNumber, Rf.seqNumber);
    DEBUG_PRINT("Ranging Error: sequence number mismatch\n");
    isErrorOccurred = true;
  }

  if (Tp.seqNumber >= Tf.seqNumber || Rp.seqNumber >= Rf.seqNumber)
  {
    DEBUG_PRINT("Ranging Error: sequence number out of order\n");
    isErrorOccurred = true;
  }

  int64_t tRound1, tReply1, tRound2, tReply2, diff1, diff2, t;
  tRound1 = (Rr.timestamp.full - Tp.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
  tReply1 = (Tr.timestamp.full - Rp.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
  tRound2 = (Rf.timestamp.full - Tr.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
  tReply2 = (Tf.timestamp.full - Rr.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
  diff1 = tRound1 - tReply1;
  diff2 = tRound2 - tReply2;
  t = (diff1 * tReply2 + diff2 * tReply1 + diff2 * diff1) / (tRound1 + tRound2 + tReply1 + tReply2);
  int16_t distance = (int16_t)t * 0.4691763978616;
  DEBUG_PRINT("compute dist 1:%d\n", distance);
  if (distance < 0)
  {
    DEBUG_PRINT("Ranging Error: distance < 0\n");
    isErrorOccurred = true;
  }

  if (distance > 1000)
  {
    DEBUG_PRINT("Ranging Error: distance > 1000\n");
    isErrorOccurred = true;
  }

  if (isErrorOccurred)
  {
    return -1;
  }

  return distance;
}

static int16_t computeDistance2(Timestamp_Tuple_t Tx, Timestamp_Tuple_t Rx,
                                Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp,
                                Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr,Ranging_Table_t * rangingTable)
{
  rangingTable->TxRxHistory.Tx.seqNumber=0;
  rangingTable->TxRxHistory.Tx.timestamp.full=0;
  rangingTable->TxRxHistory.Rx.seqNumber=0;
  rangingTable->TxRxHistory.Rx.timestamp.full=0;
  statistic[rangingTable->neighborAddress].compute3num++;
  bool isErrorOccurred = false;
  DEBUG_PRINT("Tx:%d,Rx:%d,Tp:%d,Rp:%d,Tr:%d,Rr:%d\n", Tx.seqNumber, Rx.seqNumber, Tp.seqNumber, Rp.seqNumber, Tr.seqNumber, Rr.seqNumber);

  if(Tx.seqNumber==0 || Rx.seqNumber==0){
    return -1;
  }

  if (Tp.seqNumber != Rp.seqNumber || Tr.seqNumber != Rr.seqNumber || Tx.seqNumber != Rx.seqNumber)
  {
    DEBUG_PRINT("Ranging Error: sequence number mismatch\n");
    isErrorOccurred = true;
  }

  if (Tx.seqNumber >= Tr.seqNumber || Rx.seqNumber >= Rr.seqNumber)
  {
    DEBUG_PRINT("Ranging Error: sequence number out of order\n");
    isErrorOccurred = true;
  }

  int64_t tRound1, tReply1, tRound2, tReply2, diff1, diff2, t;
  tRound1 = (Rp.timestamp.full - Tx.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
  tReply1 = (Tp.timestamp.full - Rx.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
  tRound2 = (Rr.timestamp.full - Tp.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
  tReply2 = (Tr.timestamp.full - Rp.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
  diff1 = tRound1 - tReply1;
  diff2 = tRound2 - tReply2;
  t = (diff1 * tReply2 + diff2 * tReply1 + diff2 * diff1) / (tRound1 + tRound2 + tReply1 + tReply2);
  int16_t distance = (int16_t)t * 0.4691763978616;

  DEBUG_PRINT("compute dist 2:%d\n", distance);
  if (distance < 0)
  {
    DEBUG_PRINT("Ranging Error: distance < 0\n");
    isErrorOccurred = true;
  }

  if (distance > 1000)
  {
    DEBUG_PRINT("Ranging Error: distance > 1000\n");
    isErrorOccurred = true;
  }

  if (isErrorOccurred)
  {
    return -1;
  }

  return distance;
}

static void S1_Tf(Ranging_Table_t *rangingTable)
{
  RANGING_TABLE_STATE prevState = rangingTable->state;

  /* Don't update Tf here since sending message is an async action, we put all Tf in TfBuffer. */
  rangingTable->state = RANGING_STATE_S2;

  RANGING_TABLE_STATE curState = rangingTable->state;
  //  DEBUG_PRINT("S1_Tf: S%d -> S%d\n", prevState, curState);
}

static void S1_RX_NO_Rf(Ranging_Table_t *rangingTable)
{
  RANGING_TABLE_STATE prevState = rangingTable->state;

  rangingTable->state = RANGING_STATE_S1;

  RANGING_TABLE_STATE curState = rangingTable->state;
  //  DEBUG_PRINT("Invalid state transition occurs, just ignore\n");
  //  DEBUG_PRINT("S1_RX_NO_Rf: S%d -> S%d\n", prevState, curState);
}

static void S1_RX_Rf(Ranging_Table_t *rangingTable)
{
  RANGING_TABLE_STATE prevState = rangingTable->state;

  rangingTable->state = RANGING_STATE_S1;

  RANGING_TABLE_STATE curState = rangingTable->state;
  //  DEBUG_PRINT("Invalid state transition occurs, just ignore\n");
  //  DEBUG_PRINT("S1_RX_Rf: S%d -> S%d\n", prevState, curState);
}

static void S2_Tf(Ranging_Table_t *rangingTable)
{
  RANGING_TABLE_STATE prevState = rangingTable->state;

  /* Don't update Tf here since sending message is an async action, we put all Tf in TfBuffer. */
  rangingTable->state = RANGING_STATE_S2;

  RANGING_TABLE_STATE curState = rangingTable->state;
  //  DEBUG_PRINT("S2_Tf: S%d -> S%d\n", prevState, curState);
}

static void S2_RX_NO_Rf(Ranging_Table_t *rangingTable)
{
  RANGING_TABLE_STATE prevState = rangingTable->state;

  rangingTable->state = RANGING_STATE_S2;

  RANGING_TABLE_STATE curState = rangingTable->state;
  //  DEBUG_PRINT("Invalid state transition occurs, just ignore\n");
  //  DEBUG_PRINT("S2_RX_NO_Rf: S%d -> S%d\n", prevState, curState);
}

static void S2_RX_Rf(Ranging_Table_t *rangingTable)
{
  RANGING_TABLE_STATE prevState = rangingTable->state;

  /* Find corresponding Tf in TfBuffer, it is possible that can not find corresponding Tf. */
  rangingTable->Tf = findTfBySeqNumber(rangingTable->Rf.seqNumber);
  if (!rangingTable->Tf.timestamp.full)
  {
    DEBUG_PRINT("Cannot found corresponding Tf in Tf buffer, the ranging frequency may be too high or Tf buffer is in a small size.");
  }

  /* Shift ranging table
   * Rp <- Rf
   * Tp <- Tf  Rr <- Re
   */
  rangingTable->Rp = rangingTable->Rf;
  rangingTable->Tp = rangingTable->Tf;
  rangingTable->TrRrBuffer.candidates[rangingTable->TrRrBuffer.cur].Rr = rangingTable->Re;

  Timestamp_Tuple_t empty = {.timestamp.full = 0, .seqNumber = 0};
  rangingTable->Rf = empty;
  rangingTable->Tf = empty;
  rangingTable->Re = empty;

  rangingTable->state = RANGING_STATE_S3;

  RANGING_TABLE_STATE curState = rangingTable->state;
  //  DEBUG_PRINT("S2_RX_Rf: S%d -> S%d\n", prevState, curState);
}

static void S3_Tf(Ranging_Table_t *rangingTable)
{
  RANGING_TABLE_STATE prevState = rangingTable->state;

  /* Don't update Tf here since sending message is an async action, we put all Tf in TfBuffer. */
  rangingTable->state = RANGING_STATE_S4;

  RANGING_TABLE_STATE curState = rangingTable->state;
  //  DEBUG_PRINT("S3_Tf: S%d -> S%d\n", prevState, curState);
}

static void S3_RX_NO_Rf(Ranging_Table_t *rangingTable)

{
  DEBUG_PRINT("T1-");
  Ranging_Table_Tr_Rr_Candidate_t Tr_Rr_Candidate = rangingTableBufferGetLatest(&rangingTable->TrRrBuffer);
  int16_t distance = computeDistance2(rangingTable->TxRxHistory.Tx, rangingTable->TxRxHistory.Rx,
                                      rangingTable->Tp, rangingTable->Rp,
                                      Tr_Rr_Candidate.Tr, Tr_Rr_Candidate.Rr,rangingTable);
  if (distance > 0)
  {
    statistic[rangingTable->neighborAddress].compute2num++;
    rangingTable->distance = distance;
    setDistance(rangingTable->neighborAddress, distance, 2);
  }
  else
  {
    //    DEBUG_PRINT("distance is not updated since some error occurs\n");
  }

  RANGING_TABLE_STATE prevState = rangingTable->state;

  /* Shift ranging table
   * Rr <- Re
   */
  rangingTable->TrRrBuffer.candidates[rangingTable->TrRrBuffer.cur].Rr = rangingTable->Re;
  Timestamp_Tuple_t empty = {.timestamp.full = 0, .seqNumber = 0};
  rangingTable->Re = empty;

  rangingTable->state = RANGING_STATE_S3;

  RANGING_TABLE_STATE curState = rangingTable->state;
  //  DEBUG_PRINT("S3_RX_NO_Rf: S%d -> S%d\n", prevState, curState);
}

static void S3_RX_Rf(Ranging_Table_t *rangingTable)
{
  Ranging_Table_Tr_Rr_Candidate_t Tr_Rr_Candidate = rangingTableBufferGetLatest(&rangingTable->TrRrBuffer);
  int16_t distance = computeDistance2(rangingTable->TxRxHistory.Tx, rangingTable->TxRxHistory.Rx,
                                      rangingTable->Tp, rangingTable->Rp,
                                      Tr_Rr_Candidate.Tr, Tr_Rr_Candidate.Rr,rangingTable);
  if (distance > 0)
  {
    statistic[rangingTable->neighborAddress].compute2num++;
    rangingTable->distance = distance;
    setDistance(rangingTable->neighborAddress, distance, 2);
  }
  else
  {
    //    DEBUG_PRINT("distance is not updated since some error occurs\n");
  }

  RANGING_TABLE_STATE prevState = rangingTable->state;

  /* Shift ranging table
   * Rr <- Re
   */
  rangingTable->TrRrBuffer.candidates[rangingTable->TrRrBuffer.cur].Rr = rangingTable->Re;
  Timestamp_Tuple_t empty = {.timestamp.full = 0, .seqNumber = 0};
  rangingTable->Re = empty;

  rangingTable->state = RANGING_STATE_S3;

  RANGING_TABLE_STATE curState = rangingTable->state;
  //  DEBUG_PRINT("S3_RX_Rf: S%d -> S%d\n", prevState, curState);
}

static void S4_Tf(Ranging_Table_t *rangingTable)
{
  RANGING_TABLE_STATE prevState = rangingTable->state;

  /* Don't update Tf here since sending message is an async action, we put all Tf in TfBuffer. */
  rangingTable->state = RANGING_STATE_S4;

  RANGING_TABLE_STATE curState = rangingTable->state;
  //  DEBUG_PRINT("S4_Tf: S%d -> S%d\n", prevState, curState);
}

static void S4_RX_NO_Rf(Ranging_Table_t *rangingTable)
{

  DEBUG_PRINT("T2-");
  /*use history tx,rx to compute distance*/
  Ranging_Table_Tr_Rr_Candidate_t Tr_Rr_Candidate = rangingTableBufferGetLatest(&rangingTable->TrRrBuffer);
  int16_t distance = computeDistance2(rangingTable->TxRxHistory.Tx, rangingTable->TxRxHistory.Rx,
                                      rangingTable->Tp, rangingTable->Rp,
                                      Tr_Rr_Candidate.Tr, Tr_Rr_Candidate.Rr,rangingTable);
  if (distance > 0)
  {
    statistic[rangingTable->neighborAddress].compute2num++;
    rangingTable->distance = distance;
    setDistance(rangingTable->neighborAddress, distance, 2);
  }
  else
  {
    //    DEBUG_PRINT("distance is not updated since some error occurs\n");
  }

  RANGING_TABLE_STATE prevState = rangingTable->state;

  /* Shift ranging table
   * Rr <- Re
   */
  rangingTable->TrRrBuffer.candidates[rangingTable->TrRrBuffer.cur].Rr = rangingTable->Re;
  Timestamp_Tuple_t empty = {.timestamp.full = 0, .seqNumber = 0};
  rangingTable->Re = empty;

  rangingTable->state = RANGING_STATE_S4;

  RANGING_TABLE_STATE curState = rangingTable->state;
  //  DEBUG_PRINT("S4_RX_NO_Rf: S%d -> S%d\n", prevState, curState);
}

static void S4_RX_Rf(Ranging_Table_t *rangingTable)
{
  RANGING_TABLE_STATE prevState = rangingTable->state;

  /* Find corresponding Tf in TfBuffer, it is possible that can not find corresponding Tf. */
  rangingTable->Tf = findTfBySeqNumber(rangingTable->Rf.seqNumber);

  Ranging_Table_Tr_Rr_Candidate_t Tr_Rr_Candidate = rangingTableBufferGetCandidate(&rangingTable->TrRrBuffer,
                                                                                   rangingTable->Tf, rangingTable->Tp);

  //  printRangingTable(rangingTable);
  // DEBUG_PRINT("Tp:%d,Rf:%d\n", rangingTable->Tp.seqNumber, rangingTable->Rf.seqNumber);
  /* try to compute distance */
  int16_t distance = computeDistance(rangingTable->Tp, rangingTable->Rp,
                                     Tr_Rr_Candidate.Tr, Tr_Rr_Candidate.Rr,
                                     rangingTable->Tf, rangingTable->Rf);
  if (distance > 0)
  {
    statistic[rangingTable->neighborAddress].compute1num++;
    rangingTable->distance = distance;
    setDistance(rangingTable->neighborAddress, distance, 1);
    /* update history tx,rx
     * only success distance,update history
     */
    rangingTable->TxRxHistory.Tx = Tr_Rr_Candidate.Tr;
    rangingTable->TxRxHistory.Rx = Tr_Rr_Candidate.Rr;
  }
  else
  {
    //    DEBUG_PRINT("distance is not updated since some error occurs\n");
  }

  /* Shift ranging table
   * Rp <- Rf
   * Tp <- Tf  Rr <- Re
   */
  rangingTable->Rp = rangingTable->Rf;
  rangingTable->Tp = rangingTable->Tf;
  rangingTable->TrRrBuffer.candidates[rangingTable->TrRrBuffer.cur].Rr = rangingTable->Re;

  Timestamp_Tuple_t empty = {.timestamp.full = 0, .seqNumber = 0};
  rangingTable->Rf = empty;
  rangingTable->Tf = empty;
  rangingTable->Re = empty;

  // TODO: check if valid
  rangingTable->state = RANGING_STATE_S3;

  RANGING_TABLE_STATE curState = rangingTable->state;
  //  DEBUG_PRINT("S4_RX_Rf: S%d -> S%d\n", prevState, curState);
}

/* Don't call this handler function. */
static void S5_Tf(Ranging_Table_t *rangingTable)
{
  //  DEBUG_PRINT("S5_Tf: invalid handler invocation of temporary state RANGING_STATE_S5\n");
}

/* Don't call this handler function. */
static void S5_RX_NO_Rf(Ranging_Table_t *rangingTable)
{
  //  DEBUG_PRINT("S5_RX_NO_Rf: invalid handler invocation of temporary state RANGING_STATE_S5\n");
}

/* Don't call this handler function. */
static void S5_RX_Rf(Ranging_Table_t *rangingTable)
{
  //  DEBUG_PRINT("S5_RX_Rf: invalid handler invocation of temporary state RANGING_STATE_S5\n");
}

/* Don't call this handler function. */
static void RESERVED_STUB(Ranging_Table_t *rangingTable)
{
  //  DEBUG_PRINT("RESERVED_STUB: Error, been invoked unexpectedly\n");
}

static RangingTableEventHandler EVENT_HANDLER[RANGING_TABLE_STATE_COUNT][RANGING_TABLE_EVENT_COUNT] = {
    {RESERVED_STUB, RESERVED_STUB, RESERVED_STUB},
    {S1_Tf, S1_RX_NO_Rf, S1_RX_Rf},
    {S2_Tf, S2_RX_NO_Rf, S2_RX_Rf},
    {S3_Tf, S3_RX_NO_Rf, S3_RX_Rf},
    {S4_Tf, S4_RX_NO_Rf, S4_RX_Rf},
    /* RANGING_STATE_S5 is effectively a temporary state for distance calculation, never been invoked */
    {S5_Tf, S5_RX_NO_Rf, S5_RX_Rf}};

void rangingTableOnEvent(Ranging_Table_t *table, RANGING_TABLE_EVENT event)
{
  ASSERT(table->state < RANGING_TABLE_STATE_COUNT);
  ASSERT(event < RANGING_TABLE_EVENT_COUNT);
  EVENT_HANDLER[table->state][event](table);
}

void computeRealDistance(uint16_t neighborAddress, float x1, float y1, float z1, float x2, float y2, float z2)
{
  // 计算各坐标的差
  float dx = x2 - x1;
  float dy = y2 - y1;
  float dz = z2 - z1;

  // 计算距离的平方和再开方
  float distance = sqrt(dx * dx + dy * dy + dz * dz);

  distanceReal[neighborAddress] = distance;
}

/* Swarm Ranging */
static void processRangingMessage(Ranging_Message_With_Timestamp_t *rangingMessageWithTimestamp)
{
  Ranging_Message_t *rangingMessage = &rangingMessageWithTimestamp->rangingMessage;
  uint16_t neighborAddress = rangingMessage->header.srcAddress;
  int neighborIndex = rangingTableSetSearchTable(&rangingTableSet, neighborAddress);

  DEBUG_PRINT("seq:%d\n", rangingMessage->header.msgSequence);

  float posiX = logGetFloat(idX);
  float posiY = logGetFloat(idY);
  float posiZ = logGetFloat(idZ);
  computeRealDistance(neighborAddress, posiX, posiY, posiZ, rangingMessage->header.posiX, rangingMessage->header.posiY, rangingMessage->header.posiZ);

  statistic[neighborAddress].recvnum++;
  statistic[neighborAddress].recvSeq = rangingMessage->header.msgSequence;

  /* Handle new neighbor */
  if (neighborIndex == -1)
  {
    Ranging_Table_t table;
    rangingTableInit(&table, neighborAddress);
    /* Ranging table set is full, ignore this ranging message. */
    if (!rangingTableSetAddTable(&rangingTableSet, table))
    {
      DEBUG_PRINT("processRangingMessage: Ranging table is full = %d, cannot handle new neighbor %d.\n",
                  rangingTableSet.size,
                  neighborAddress);
      return;
    }
    else
    {
      neighborIndex = rangingTableSetSearchTable(&rangingTableSet, neighborAddress);
    }
  }

  Ranging_Table_t *neighborRangingTable = &rangingTableSet.tables[neighborIndex];
  /* Update Re */
  neighborRangingTable->Re.timestamp = rangingMessageWithTimestamp->rxTime;
  neighborRangingTable->Re.seqNumber = rangingMessage->header.msgSequence;
  /* Update latest received timestamp of this neighbor */
  neighborRangingTable->latestReceived = neighborRangingTable->Re;
  /* Update expiration time of this neighbor */
  neighborRangingTable->expirationTime = xTaskGetTickCount() + M2T(RANGING_TABLE_HOLD_TIME);

  /* Each ranging messages contains MAX_Tr_UNIT lastTxTimestamps, find corresponding
   * Tr according to Rr to get a valid Tr-Rr pair if possible, this approach may
   * help when experiencing continuous packet loss.
   */
  Ranging_Table_Tr_Rr_Buffer_t *neighborTrRrBuffer = &neighborRangingTable->TrRrBuffer;
  for (int i = 0; i < RANGING_MAX_Tr_UNIT; i++)
  {
    if (rangingMessage->header.lastTxTimestamps[i].timestamp.full && neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr.timestamp.full && rangingMessage->header.lastTxTimestamps[i].seqNumber == neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr.seqNumber)
    {
      rangingTableBufferUpdate(&neighborRangingTable->TrRrBuffer,
                               rangingMessage->header.lastTxTimestamps[i],
                               neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr);
      break;
    }
  }
  //  printRangingMessage(rangingMessage);

  /* Try to find corresponding Rf for MY_UWB_ADDRESS. */
  Timestamp_Tuple_t neighborRf = {.timestamp.full = 0, .seqNumber = 0};
  if (rangingMessage->header.filter & (1 << (uwbGetAddress() % 16)))
  {
    /* Retrieve body unit from received ranging message. */
    uint8_t bodyUnitCount = (rangingMessage->header.msgLength - sizeof(Ranging_Message_Header_t)) / sizeof(Body_Unit_t);
    for (int i = 0; i < bodyUnitCount; i++)
    {
      if (rangingMessage->bodyUnits[i].address == uwbGetAddress())
      {
        neighborRf.timestamp = rangingMessage->bodyUnits[i].timestamp;
        neighborRf.seqNumber = rangingMessage->bodyUnits[i].seqNumber;
        break;
      }
    }
  }
  Timestamp_Tuple_t Tf = findTfBySeqNumber(neighborRf.seqNumber);

  if (neighborRf.seqNumber != neighborRangingTable->Tp.seqNumber && Tf.timestamp.full)
  {
    neighborRangingTable->Rf = neighborRf;
    rangingTableOnEvent(neighborRangingTable, RANGING_EVENT_RX_Rf);
  }
  else
  {
    rangingTableOnEvent(neighborRangingTable, RANGING_EVENT_RX_NO_Rf);
  }
  // /* Trigger event handler according to Rf */
  // if (neighborRf.timestamp.full)
  // {
  //   neighborRangingTable->Rf = neighborRf;
  //   rangingTableOnEvent(neighborRangingTable, RANGING_EVENT_RX_Rf);
  // }
  // else
  // {
  //   DEBUG_PRINT("------");
  //   rangingTableOnEvent(neighborRangingTable, RANGING_EVENT_RX_NO_Rf);
  // }

#ifdef ENABLE_DYNAMIC_RANGING_PERIOD
  /* update period according to distance and velocity */
  neighborRangingTable->period = M2T(DYNAMIC_RANGING_COEFFICIENT * (neighborRangingTable->distance / rangingMessage->header.velocity));
  /* bound ranging period between RANGING_PERIOD_MIN and RANGING_PERIOD_MAX */
  neighborRangingTable->period = MAX(neighborRangingTable->period, M2T(RANGING_PERIOD_MIN));
  neighborRangingTable->period = MIN(neighborRangingTable->period, M2T(RANGING_PERIOD_MAX));
#endif
}

/* By default, we include each neighbor's latest rx timestamp to body unit in index order of ranging table, which
 * may cause ranging starvation, i.e. node 1 has many one-hop neighbors [2, 3, 4, 5, 6, 7, 8, 9, ..., 30], since
 * RANGING_MAX_BODY_UNIT < rangingTable.size, so each ranging message can only include a subset of it's one-hop
 * neighbor.
 * Say ranging table of node 1:
 *  index 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  ...
 *  id    2   7   5   3   8   11  15  6   16  17  23  24  25  26  4   18  ...
 * Then it is possible that ranging message N send by this node behaves like below (RANGING_MAX_BODY_UNIT = 4):
 *            ranging message 1: [2, 7, 5, 3]
 *            ranging message 2: [2, 7, 5, 3]
 *            ranging message 3: [2, 7, 5, 3]
 *            ranging message 4: [2, 7, 5, 3]
 *            ...
 * While the expected behavior is:
 *            ranging message 1: [2, 7, 5, 3]
 *            ranging message 2: [8, 11, 15, 6]
 *            ranging message 3: [16, 17, 23, 24]
 *            ranging message 4: [25, 26, 4, 18]
 *            ...
 * This behavior is lead by the fact that everytime we want to populate a new ranging message, the ranging tables
 * is traversed by index. If ENABLE_BUS_BOARDING_SCHEME, then the (table index, id) relationship is dynamically
 * changed according to the nextExpectedDeliveryTime of each table entry, which bypasses this issue implicitly.
 * Therefore, to solve this issue when BUS_BOARDING_SCHEME is not enabled, we should change the (table index, id)
 * relationship after populating new ranging message in the approach below:
 * Say the ranging tables of node 1:
 *  index 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  ...
 *  id    2   7   5   3   8   11  15  6   16  17  23  24  25  26  4   18  ...
 *            ranging message 1: [2, 7, 5, 3]
 * Then change the (index, id) relationship by moving the ranging table of included timestamp to last.
 *  index 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  ...
 *  id    11  15  6   16  17  23  24  25  26  4   18  2   7   5   3   8   ...
 *            ranging message 2: [11, 15, 6, 16]
 * Again:
 *  index 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  ...
 *  id    17  23  24  25  26  4   18  2   7   5   3   8   11  15  6   16  ...
 *            ranging message 3: [17, 23, 24, 25]
 * This makes the ranging table behaves like a cyclic array, the actual implementation have also considered the
 * nextExpectedDeliveryTime (only include timestamp with expected next delivery time less or equal than current
 * time) by sort the ranging table set by each timestamp's last send time.
 */
static Time_t generateRangingMessage(Ranging_Message_t *rangingMessage)
{
  int8_t bodyUnitNumber = 0;
  rangingSeqNumber++;
  int curSeqNumber = rangingSeqNumber;
  rangingMessage->header.filter = 0;
  Time_t curTime = xTaskGetTickCount();
  /* Using the default RANGING_PERIOD when DYNAMIC_RANGING_PERIOD is not enabled. */
  Time_t taskDelay = M2T(RANGING_PERIOD);
#ifdef ENABLE_BUS_BOARDING_SCHEME
  rangingTableSetRearrange(&rangingTableSet, COMPARE_BY_NEXT_EXPECTED_DELIVERY_TIME);
#else
  // rangingTableSetRearrange(&rangingTableSet, COMPARE_BY_LAST_SEND_TIME);
#endif

  /* Generate message body */
  for (int index = 0; index < rangingTableSet.size; index++)
  {
    if (bodyUnitNumber >= RANGING_MAX_BODY_UNIT)
    {
      break;
    }
    Ranging_Table_t *table = &rangingTableSet.tables[index];
    if (table->latestReceived.timestamp.full)
    {
      /* Only include timestamps with expected delivery time less or equal than current time. */
      if (table->nextExpectedDeliveryTime > curTime)
      {
        continue;
      }
      table->nextExpectedDeliveryTime = curTime + M2T(table->period);
      table->lastSendTime = curTime;

      /* It is possible that latestReceived is not the newest timestamp, because the newest may be in rxQueue
       * waiting to be handled.
       */
      rangingMessage->bodyUnits[bodyUnitNumber].timestamp = table->latestReceived.timestamp;
      rangingMessage->bodyUnits[bodyUnitNumber].seqNumber = table->latestReceived.seqNumber;
      rangingMessage->bodyUnits[bodyUnitNumber].address = table->neighborAddress;
      // table->latestReceived.seqNumber = 0;
      // table->latestReceived.timestamp.full = 0;
      // int randnum = rand() % 10;
      // if (randnum < 7)
      // {
      //   rangingMessage->bodyUnits[bodyUnitNumber].timestamp = table->latestReceived;
      // }
      // else
      // {
      //   Timestamp_Tuple_t empty = {.seqNumber = 0, .timestamp.full = 0};
      //   rangingMessage->bodyUnits[bodyUnitNumber].timestamp = empty;
      // }
      rangingMessage->header.filter |= 1 << (table->neighborAddress % 16);
      rangingTableOnEvent(table, RANGING_EVENT_TX_Tf);

#ifdef ENABLE_DYNAMIC_RANGING_PERIOD
      /* Change task delay dynamically, may increase packet loss rate since ranging period now is determined
       * by the minimum expected delivery time.
       */
      taskDelay = MIN(taskDelay, table->nextExpectedDeliveryTime - curTime);
      /* Bound the dynamic task delay between RANGING_PERIOD_MIN and RANGING_PERIOD */
      taskDelay = MAX(RANGING_PERIOD_MIN, taskDelay);
#endif

#ifdef ROUTING_OLSR_ENABLE
      // if (mprSetHas(getGlobalMPRSet(), table->neighborAddress))
      // {
      //   rangingMessage->bodyUnits[bodyUnitNumber].flags.MPR = true;
      // }
      // else
      // {
      //   rangingMessage->bodyUnits[bodyUnitNumber].flags.MPR = false;
      // }
#endif

      bodyUnitNumber++;
    }
  }
  /* Generate message header */
  rangingMessage->header.srcAddress = MY_UWB_ADDRESS;
  rangingMessage->header.msgLength = sizeof(Ranging_Message_Header_t) + sizeof(Body_Unit_t) * bodyUnitNumber;
  rangingMessage->header.msgSequence = curSeqNumber;
  // getLatestNTxTimestamps(rangingMessage->header.lastTxTimestamps, RANGING_MAX_Tr_UNIT);

  // xSemaphoreTake(TfBufferMutex, portMAX_DELAY);
  int startIndex = (TfBufferIndex + 1 - RANGING_MAX_Tr_UNIT + Tf_BUFFER_POOL_SIZE) % Tf_BUFFER_POOL_SIZE;
  for (int i = RANGING_MAX_Tr_UNIT - 1; i >= 0; i--)
  {
    rangingMessage->header.lastTxTimestamps[i].timestamp = TfBuffer[startIndex].timestamp;
    rangingMessage->header.lastTxTimestamps[i].seqNumber = TfBuffer[startIndex].seqNumber;
    startIndex = (startIndex + 1) % Tf_BUFFER_POOL_SIZE;
  }
  // xSemaphoreGive(TfBufferMutex);

  float velocityX = logGetFloat(idVelocityX);
  float velocityY = logGetFloat(idVelocityY);
  float velocityZ = logGetFloat(idVelocityZ);

  float posiX = logGetFloat(idX);
  float posiY = logGetFloat(idY);
  float posiZ = logGetFloat(idZ);
  DEBUG_PRINT("%f\n", posiX);

  rangingMessage->header.posiX = posiX;
  rangingMessage->header.posiY = posiY;
  rangingMessage->header.posiZ = posiZ;

  velocity = sqrt(pow(velocityX, 2) + pow(velocityY, 2) + pow(velocityZ, 2));
  /* velocity in cm/s */
  // rangingMessage->header.velocity = (short)(velocity * 100);
  //  DEBUG_PRINT("generateRangingMessage: ranging message size = %u with %u body units.\n",
  //              rangingMessage->header.msgLength,
  //              bodyUnitNumber
  //  );

  /* Keeps ranging table in order to perform binary search */
  rangingTableSetRearrange(&rangingTableSet, COMPARE_BY_ADDRESS);

  return taskDelay;
}

static void uwbRangingTxTask(void *parameters)
{
  systemWaitStart();

  /* velocity log variable id */
  idVelocityX = logGetVarId("stateEstimate", "vx");
  idVelocityY = logGetVarId("stateEstimate", "vy");
  idVelocityZ = logGetVarId("stateEstimate", "vz");

  idX = logGetVarId("lighthouse", "x");
  idY = logGetVarId("lighthouse", "y");
  idZ = logGetVarId("lighthouse", "z");

  UWB_Packet_t txPacketCache;
  txPacketCache.header.srcAddress = uwbGetAddress();
  txPacketCache.header.destAddress = UWB_DEST_ANY;
  txPacketCache.header.type = UWB_RANGING_MESSAGE;
  txPacketCache.header.length = 0;
  Ranging_Message_t *rangingMessage = (Ranging_Message_t *)&txPacketCache.payload;

  while (true)
  {
    xSemaphoreTake(rangingTableSet.mu, portMAX_DELAY);
    // xSemaphoreTake(neighborSet.mu, portMAX_DELAY);
    Time_t taskDelay = RANGING_PERIOD;
    generateRangingMessage(rangingMessage);
    txPacketCache.header.length = sizeof(UWB_Packet_Header_t) + rangingMessage->header.msgLength;
    uwbSendPacketBlock(&txPacketCache);
    //    printRangingTableSet(&rangingTableSet);
    //    printNeighborSet(&neighborSet);

    // xSemaphoreGive(neighborSet.mu);
    xSemaphoreGive(rangingTableSet.mu);
#ifdef ENABLE_OPTIMAL_RANGING_SCHEDULE
    int8_t time_Delay = temp_delay;
    temp_delay = 0;
    vTaskDelay(RANGING_PERIOD + time_Delay);
#else
    vTaskDelay(taskDelay);
#endif
  }
}

static void uwbRangingRxTask(void *parameters)
{
  systemWaitStart();

  Ranging_Message_With_Timestamp_t rxPacketCache;

  while (true)
  {
    if (xQueueReceive(rxQueue, &rxPacketCache, portMAX_DELAY))
    {
      int randnum = rand() % 20;
      // if (randnum < 17)
      {
        xSemaphoreTake(rangingTableSet.mu, portMAX_DELAY);
        // xSemaphoreTake(neighborSet.mu, portMAX_DELAY);

        processRangingMessage(&rxPacketCache);
        // topologySensing(&rxPacketCache.rangingMessage);

        // xSemaphoreGive(neighborSet.mu);
        xSemaphoreGive(rangingTableSet.mu);
      }
    }
    vTaskDelay(M2T(1));
  }
}

void rangingRxCallback(void *parameters)
{
  // DEBUG_PRINT("rangingRxCallback \n");

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  UWB_Packet_t *packet = (UWB_Packet_t *)parameters;

  dwTime_t rxTime;
  dwt_readrxtimestamp((uint8_t *)&rxTime.raw);

#ifdef ENABLE_OPTIMAL_RANGING_SCHEDULE
  rx_buffer_index++;
  rx_buffer_index %= NEIGHBOR_ADDRESS_MAX;
  rx_buffer[rx_buffer_index].seqNumber = rangingSeqNumber;
  rx_buffer[rx_buffer_index].timestamp = rxTime;
  predict_period_in_rx(rx_buffer_index);
#endif

  Ranging_Message_With_Timestamp_t rxMessageWithTimestamp;
  rxMessageWithTimestamp.rxTime = rxTime;
  Ranging_Message_t *rangingMessage = (Ranging_Message_t *)packet->payload;
  rxMessageWithTimestamp.rangingMessage = *rangingMessage;

  xQueueSendFromISR(rxQueue, &rxMessageWithTimestamp, &xHigherPriorityTaskWoken);
}

void rangingTxCallback(void *parameters)
{
  UWB_Packet_t *packet = (UWB_Packet_t *)parameters;
  Ranging_Message_t *rangingMessage = (Ranging_Message_t *)packet->payload;

  dwTime_t txTime;
  dwt_readtxtimestamp((uint8_t *)&txTime.raw);

  Timestamp_Tuple_t timestamp = {.timestamp = txTime, .seqNumber = rangingMessage->header.msgSequence};
  updateTfBuffer(timestamp);
}

void rangingInit()
{
  MY_UWB_ADDRESS = uwbGetAddress();
  srand(MY_UWB_ADDRESS);
  rxQueue = xQueueCreate(RANGING_RX_QUEUE_SIZE, RANGING_RX_QUEUE_ITEM_SIZE);
  // neighborSetInit(&neighborSet);
  // neighborSetEvictionTimer = xTimerCreate("neighborSetEvictionTimer",
  //                                         M2T(NEIGHBOR_SET_HOLD_TIME / 2),
  //                                         pdTRUE,
  //                                         (void *)0,
  //                                         neighborSetClearExpireTimerCallback);
  // xTimerStart(neighborSetEvictionTimer, M2T(0));
  rangingTableSetInit(&rangingTableSet);
  rangingTableSetEvictionTimer = xTimerCreate("rangingTableSetEvictionTimer",
                                              M2T(RANGING_TABLE_HOLD_TIME / 2),
                                              pdTRUE,
                                              (void *)0,
                                              rangingTableSetClearExpireTimerCallback);
  xTimerStart(rangingTableSetEvictionTimer, M2T(0));
  TfBufferMutex = xSemaphoreCreateMutex();

  listener.type = UWB_RANGING_MESSAGE;
  listener.rxQueue = NULL; // handle rxQueue in swarm_ranging.c instead of adhocdeck.c
  listener.rxCb = rangingRxCallback;
  listener.txCb = rangingTxCallback;
  uwbRegisterListener(&listener);

  idVelocityX = logGetVarId("stateEstimate", "vx");
  idVelocityY = logGetVarId("stateEstimate", "vy");
  idVelocityZ = logGetVarId("stateEstimate", "vz");

  statisticInit();

  xTaskCreate(uwbRangingTxTask, ADHOC_DECK_RANGING_TX_TASK_NAME, UWB_TASK_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbRangingTxTaskHandle);
  xTaskCreate(uwbRangingRxTask, ADHOC_DECK_RANGING_RX_TASK_NAME, UWB_TASK_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbRangingRxTaskHandle);
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
LOG_ADD(LOG_INT16, distTo11, distanceTowards + 11)
LOG_ADD(LOG_INT16, distTo12, distanceTowards + 12)
LOG_ADD(LOG_INT16, distTo13, distanceTowards + 13)
LOG_ADD(LOG_INT16, distTo14, distanceTowards + 14)
LOG_ADD(LOG_INT16, distTo15, distanceTowards + 15)
LOG_ADD(LOG_INT16, distTo16, distanceTowards + 16)
LOG_ADD(LOG_INT16, distTo17, distanceTowards + 17)
LOG_ADD(LOG_INT16, distTo18, distanceTowards + 18)
LOG_ADD(LOG_INT16, distTo19, distanceTowards + 19)
LOG_ADD(LOG_INT16, distTo20, distanceTowards + 20)
LOG_ADD(LOG_INT16, distTo21, distanceTowards + 21)
LOG_ADD(LOG_INT16, distTo22, distanceTowards + 22)
LOG_ADD(LOG_INT16, distTo23, distanceTowards + 23)
LOG_ADD(LOG_INT16, distTo24, distanceTowards + 24)
LOG_ADD(LOG_INT16, distTo25, distanceTowards + 25)
LOG_ADD(LOG_INT16, distTo26, distanceTowards + 26)
LOG_ADD(LOG_INT16, distTo27, distanceTowards + 27)
LOG_ADD(LOG_INT16, distTo28, distanceTowards + 28)
LOG_ADD(LOG_INT16, distTo29, distanceTowards + 29)
LOG_ADD(LOG_INT16, distTo30, distanceTowards + 30)

LOG_GROUP_STOP(Ranging)

LOG_GROUP_START(Statistic)
LOG_ADD(LOG_UINT16, recvSeq0, &statistic[0].recvSeq)
LOG_ADD(LOG_UINT16, recvNum0, &statistic[0].recvnum)
LOG_ADD(LOG_UINT16, compute1num0, &statistic[0].compute1num)
LOG_ADD(LOG_UINT16, compute2num0, &statistic[0].compute2num)
LOG_ADD(LOG_UINT16, compute3num0, &statistic[0].compute3num)
LOG_ADD(LOG_INT16, dist0, distanceTowards + 0)
LOG_ADD(LOG_UINT8, distSrc0, distanceSource + 0)
LOG_ADD(LOG_FLOAT, distReal0, distanceReal + 0)

LOG_ADD(LOG_UINT16, recvSeq1, &statistic[1].recvSeq)
LOG_ADD(LOG_UINT16, recvNum1, &statistic[1].recvnum)
LOG_ADD(LOG_UINT16, compute1num1, &statistic[1].compute1num)
LOG_ADD(LOG_UINT16, compute2num1, &statistic[1].compute2num)
LOG_ADD(LOG_UINT16, compute3num1, &statistic[1].compute3num)
LOG_ADD(LOG_INT16, dist1, distanceTowards + 1)
LOG_ADD(LOG_UINT8, distSrc1, distanceSource + 1)
LOG_ADD(LOG_FLOAT, distReal1, distanceReal + 1)

LOG_ADD(LOG_UINT16, recvSeq2, &statistic[2].recvSeq)
LOG_ADD(LOG_UINT16, recvNum2, &statistic[2].recvnum)
LOG_ADD(LOG_UINT16, compute1num2, &statistic[2].compute1num)
LOG_ADD(LOG_UINT16, compute2num2, &statistic[2].compute2num)
LOG_ADD(LOG_UINT16, compute3num2, &statistic[2].compute3num)
LOG_ADD(LOG_INT16, dist2, distanceTowards + 2)
LOG_ADD(LOG_UINT8, distSrc2, distanceSource + 2)
LOG_ADD(LOG_FLOAT, distReal2, distanceReal + 2)

LOG_ADD(LOG_UINT16, recvSeq3, &statistic[3].recvSeq)
LOG_ADD(LOG_UINT16, recvNum3, &statistic[3].recvnum)
LOG_ADD(LOG_UINT16, compute1num3, &statistic[3].compute1num)
LOG_ADD(LOG_UINT16, compute2num3, &statistic[3].compute2num)
LOG_ADD(LOG_UINT16, compute3num3, &statistic[3].compute3num)
LOG_ADD(LOG_INT16, dist3, distanceTowards + 3)
LOG_ADD(LOG_UINT8, distSrc3, distanceSource + 3)
LOG_ADD(LOG_FLOAT, distReal3, distanceReal + 3)

LOG_ADD(LOG_UINT16, recvSeq4, &statistic[4].recvSeq)
LOG_ADD(LOG_UINT16, recvNum4, &statistic[4].recvnum)
LOG_ADD(LOG_UINT16, compute1num4, &statistic[4].compute1num)
LOG_ADD(LOG_UINT16, compute2num4, &statistic[4].compute2num)
LOG_ADD(LOG_UINT16, compute3num4, &statistic[4].compute3num)
LOG_ADD(LOG_INT16, dist4, distanceTowards + 4)
LOG_ADD(LOG_UINT8, distSrc4, distanceSource + 4)
LOG_ADD(LOG_FLOAT, distReal4, distanceReal + 4)

LOG_ADD(LOG_UINT16, recvSeq5, &statistic[5].recvSeq)
LOG_ADD(LOG_UINT16, recvNum5, &statistic[5].recvnum)
LOG_ADD(LOG_UINT16, compute1num5, &statistic[5].compute1num)
LOG_ADD(LOG_UINT16, compute2num5, &statistic[5].compute2num)
LOG_ADD(LOG_UINT16, compute3num5, &statistic[5].compute3num)
LOG_ADD(LOG_INT16, dist5, distanceTowards + 5)
LOG_ADD(LOG_UINT8, distSrc5, distanceSource + 5)
LOG_ADD(LOG_FLOAT, distReal5, distanceReal + 5)

LOG_ADD(LOG_UINT16, recvSeq6, &statistic[6].recvSeq)
LOG_ADD(LOG_UINT16, recvNum6, &statistic[6].recvnum)
LOG_ADD(LOG_UINT16, compute1num6, &statistic[6].compute1num)
LOG_ADD(LOG_UINT16, compute2num6, &statistic[6].compute2num)
LOG_ADD(LOG_UINT16, compute3num6, &statistic[6].compute3num)
LOG_ADD(LOG_INT16, dist6, distanceTowards + 6)
LOG_ADD(LOG_UINT8, distSrc6, distanceSource + 6)
LOG_ADD(LOG_FLOAT, distReal6, distanceReal + 6)

LOG_ADD(LOG_UINT16, recvSeq7, &statistic[7].recvSeq)
LOG_ADD(LOG_UINT16, recvNum7, &statistic[7].recvnum)
LOG_ADD(LOG_UINT16, compute1num7, &statistic[7].compute1num)
LOG_ADD(LOG_UINT16, compute2num7, &statistic[7].compute2num)
LOG_ADD(LOG_UINT16, compute3num7, &statistic[7].compute3num)
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