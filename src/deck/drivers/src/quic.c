#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "timers.h"
#include "debug.h"
#include "system.h"
#include "param.h"
#include "quic.h"

#ifndef QUIC_DEBUG_ENABLE
#undef DEBUG_PRINT
#define DEBUG_PRINT
#endif

#define ERROR 1
#define SUCCESS 0

static QueueHandle_t rxPacketQueue;
static QueueHandle_t txInitialOrHandshakeBufferQueue;
static QueueHandle_t txOneRTTBufferQueue;
static TaskHandle_t quicRxTaskHandle;

static void quicRxTask() {
    while(1) {
        vTaskDelay(M2T(1));
    }
}

void quicInit() {
    rxPacketQueue = XQueueCreate(QUIC_RX_PACKET_QUEUE_SIZE, QUIC_RX_PACKET_ITEM_SIZE);
    txInitialOrHandshakeBufferQueue = xQueueCreate(QUIC_TX_INITIAL_OR_HANDSHAKE_BUFFER_QUEUE_SIZE, QUIC_TX_INITIAL_OR_HANDSHAKE_BUFFER_QUEUE_ITEM_SIZE);
    txOneRTTBufferQueue = xQueueCreate(QUIC_TX_ONE_RTT_BUFFER_QUEUE_SIZE, QUIC_TX_ONE_RTT_BUFFER_QUEUE_ITEM_SIZE);

    UWB_Data_Packet_Listener_t listener = {
        .type = UWB_DATA_MESSAGE_QUIC,
        .rxQueue = rxPacketQueue
    };
    uwbRegisterDataPacketListener(&listener);

    xTaskCreate(quicRxTask, ADHOC_DECK_QUIC_RX_TASK_NAME, UWB_TASK_STACK_SIZE, NULL, ADHOC_DECK_TASK_PRI, &quicRxTaskHandle);
}

/* Generate Packet Operations */
int quicGenerateInitialPacket(Quic_Initial_or_Handshake_Packet_t *packet, uint16_t srcCID, uint16_t dstCID) {
    
}

/* Generate Frame Operations */
int quicGenerateOnlyTypeFrame(Quic_Only_Type_Frame_t *frame, QUIC_FRAME_TYPE type) {
    if(type > 3) {
        DEBUG_PRINT("Error in generate only type frame, type is compatibal.");
        return ERROR;
    }
    frame->type = type;
    return SUCCESS;
}