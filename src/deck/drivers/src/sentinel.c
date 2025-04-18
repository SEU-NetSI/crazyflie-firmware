#include "sentinel.h"

#include "FreeRTOSConfig.h"
#include "timers.h"

#include "swarm_ranging.h"

static TimerHandle_t neighborStateTableSetEvictionTimer;
static Neighbor_State_Table_Set_t neighborStateTableSet;
static QueueHandle_t rxQueue;


void initNeighborStateTableSet(Neighbor_State_Table_Set_t *set) {
    set->mu = xSemaphoreCreateMutex();
    set->size = 0;
    set->head = -1;
    for (int i = 0; i < SNIFFER_SWARMRANGING_SIZE_MAX; i++) {
        set->isValid[i] = 0;
        set->prev[i] = -1;
        set->next[i] = -1;
        set->lastRecvNeighborTick[i] = 0;
    }
}

void neighborStateTableSetUpdate(Neighbor_State_Table_Set_t *set, UWB_Address_t neighborAddress, uint32_t tick){
    ASSERT(neighborAddress < SNIFFER_SWARMRANGING_SIZE_MAX);
    set->lastRecvNeighborTick[neighborAddress] = tick;
    // 原来就有，直接更新就可以
    if(set->isValid[neighborAddress] != 0){
        return;
    }
    // 现在需要插入
    set->size++;
    set->isValid[neighborAddress] = 1;
    // 1. 第一个元素，直接插入
    if(set->head == -1){
        set->head = neighborAddress;
        return;
    }
    // 2. 非第一个元素
    // 2.1 在head之前
    if(neighborAddress < set->head){
        set->prev[set->head] = neighborAddress;
        set->next[neighborAddress] = set->head;
        set->head = neighborAddress;
        return;
    }
    // 2.2 在head之后
    int prev = set->head;
    int next = set->head;
    while (next != -1 && next < neighborAddress)
    {
        prev = next;
        next = set->next[next];
    }
    // 此时的i就应该是next了

    set->next[neighborAddress] = next;
    set->prev[neighborAddress] = prev;

    if(next != -1){
        set->prev[next] = neighborAddress;
    }
    set->next[prev] = neighborAddress;
}

void neighborStateTableSetRemove(Neighbor_State_Table_Set_t *set, UWB_Address_t neighborAddress){
// 双向链表维护
    
    int16_t prev = set->prev[neighborAddress];
    int16_t next = set->next[neighborAddress];

    // 1. 前边没有节点
    if ( prev == -1) {
        // 说明它自己就是头节点，因此更新头节点
        set->head = next;
        // 2.1 后面有节点
        if (next != -1) {
            set->prev[next] = -1;  // 更新他的下一个节点的prev
        }
    } else {
        // 2. 前面一定有节点
        // 2.1 后面有节点
        set->next[prev] = next;
        if(next != -1){
            set->prev[next] = prev;
        }
    }
    // 删除这条数据
    set->size--;
    set->isValid[neighborAddress] = 0;
    set->prev[neighborAddress] = -1;
    set->next[neighborAddress] = -1;
}

static void neighborStateTableSetEvictionTimerCallback(TimerHandle_t timer) {
  Time_t curTime = xTaskGetTickCount();
  // 对过期时间进行处理
  for (int i = neighborStateTableSet.head; i != -1;) {
        if (neighborStateTableSet.lastRecvNeighborTick[i] + DOWN_AFTER_MILLISECOND < curTime) {  // 超时未收到消息
            int next = neighborStateTableSet.next[i];
            neighborStateTableSetRemove(&neighborStateTableSet,i);
            // 检查是否为leader，如果是leader则需要进行发起客观下线投票
            i = next;
        }else{
            i = neighborStateTableSet.next[i];
        }
    }
}


static void sentinelSnifferRxCallback(void *parameters) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(rxQueue, parameters, &xHigherPriorityTaskWoken);
}

static void sentinelSnifferTask(void *parameters) {
  systemWaitStart();
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  UWB_Packet_t packet;
  while (1) {
    if (xQueueReceive(rxQueue, &packet, portMAX_DELAY)) {
        Ranging_Message_t *rangingMessage = (Ranging_Message_t *)packet.payload;
        UWB_Address_t senderAddress = rangingMessage->header.srcAddress;
        // 更新该地址对应的时间戳
        uint32_t currTick = xTaskGetTickCount();
        // 将该地址对应数据更新或者插入
        neighborStateTableSetUpdate(&neighborStateTableSet,senderAddress,currTick);
    }
    vTaskDelay(1);
  }
}

void initSentinel() {
  neighborStateTableSetEvictionTimer = xTimerCreate(
      "rangingTableSetEvictionTimer", M2T(100), pdTRUE,
      (void *)0, neighborStateTableSetEvictionTimerCallback);
  xTimerStart(neighborStateTableSetEvictionTimer, M2T(0));
}
