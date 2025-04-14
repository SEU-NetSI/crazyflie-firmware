#ifndef __AUTOFLY_PACKET_QUEUE_H__
#define __AUTOFLY_PACKET_QUEUE_H__

#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "autoflyPacket.h"

typedef struct{
    SemaphoreHandle_t mutexWriteWrite;
    SemaphoreHandle_t mutexWriteRead;
    
    uint8_t front;
    uint8_t tail;
    uint8_t len;
    Autofly_packet_t data[AUTOFLY_PACKET_QUEUE_SIZE];
}Autofly_packet_Queue_t;

void initAutoflyPacketQueue(Autofly_packet_Queue_t *queue);
void pushAutoflyPacketQueue(Autofly_packet_Queue_t *queue, Autofly_packet_t* data);
bool popAutoflyPacketQueue(Autofly_packet_Queue_t *queue, Autofly_packet_t* data);
bool isAutoflyPacketQueueEmpty(Autofly_packet_Queue_t *queue);
bool isAutoflyPacketQueueFull(Autofly_packet_Queue_t *queue);


#endif
