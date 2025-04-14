#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"
#include "system.h"
#include "debug.h"

#include "autoflyPacketQueue.h"

#define CHECK_INTERVAL 5

void initAutoflyPacketQueue(Autofly_packet_Queue_t *queue){
    queue->front = 0;
    queue->tail = 0;
    queue->len = 0;
    queue->mutexWriteWrite = xSemaphoreCreateMutex();
    queue->mutexWriteRead = xSemaphoreCreateMutex();
    if(queue->mutexWriteWrite == NULL || queue->mutexWriteRead == NULL){
        DEBUG_PRINT("[initAutoflyPacketQueue]Create mutex failed\n");
    }
    for(int i = 0; i < AUTOFLY_PACKET_QUEUE_SIZE; i++){
        queue->data[i].header.sourceId = 0;
        queue->data[i].header.destinationId = 0;
        queue->data[i].header.packetType = 0;
        queue->data[i].header.length = 0;
        memset(queue->data[i].data, 0, AUTOFLY_PACKET_MTU);
    }
}
void pushAutoflyPacketQueue(Autofly_packet_Queue_t *queue, Autofly_packet_t* data){
    // DEBUG_PRINT("[pushAutoflyPacketQueue start]len = %d\n", queue->len);
    // 先占用写写信号量
    xSemaphoreTake(queue->mutexWriteWrite, portMAX_DELAY);
    // 死等队列空闲,待优化
    while (isAutoflyPacketQueueFull(queue))
    {
        vTaskDelay(M2T(CHECK_INTERVAL));
    }
    // DEBUG_PRINT("[pushAutoflyPacketQueue]Get WriteWriteLock\n");
    // 修改内容，占有写读信号量
    xSemaphoreTake(queue->mutexWriteRead, portMAX_DELAY);
    // DEBUG_PRINT("[pushAutoflyPacketQueue]Get WriteReadLock\n");
    memcpy(&queue->data[queue->tail], data, sizeof(Autofly_packet_t));
    queue->tail = (queue->tail + 1) % AUTOFLY_PACKET_QUEUE_SIZE;
    queue->len++;
    // DEBUG_PRINT("[pushAutoflyPacketQueue]len = %d\n", queue->len);
    // 释放信号量
    xSemaphoreGive(queue->mutexWriteRead);
    xSemaphoreGive(queue->mutexWriteWrite);
}
bool popAutoflyPacketQueue(Autofly_packet_Queue_t *queue, Autofly_packet_t* data){
    // 等待队列非空
    if(isAutoflyPacketQueueEmpty(queue)){
        return false;
    }
    xSemaphoreTake(queue->mutexWriteRead, portMAX_DELAY);
    memcpy(data, &queue->data[queue->front], sizeof(Autofly_packet_t));
    queue->front = (queue->front + 1) % AUTOFLY_PACKET_QUEUE_SIZE;
    queue->len--;
    // DEBUG_PRINT("[popAutoflyPacketQueue]len = %d\n", queue->len);
    xSemaphoreGive(queue->mutexWriteRead);
    return true;
}

bool isAutoflyPacketQueueEmpty(Autofly_packet_Queue_t *queue){
    return queue->len == 0;
}
bool isAutoflyPacketQueueFull(Autofly_packet_Queue_t *queue){
    return queue->len == AUTOFLY_PACKET_QUEUE_SIZE;
}