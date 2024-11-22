#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "timers.h"
#include "debug.h"
#include "system.h"
#include "param.h"
#include "quic.h"
#include "routing.h"
#include "tools.h"

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
static QUIC_Client_Node_t quicClientNode; // TODO: may change name
// static QUIC_Client_Conn_Item_t quicClientConnBuffer[QUIC_CONNECTION_BUFFER_MAX] = {0};
// static QUIC_Packet_Seq_Number_Tuple_t connPacketSeqTuple[QUIC_CONNECTION_NUMBER_MAX] = {0};
static uint16_t quicSrcConnId = 0;
static SemaphoreHandle_t quicConnIdMutex;
// static Map_t quicClientConnBufferMap;

/* Local functions */
static uint16_t getNextSrcConnId() {
    xSemaphoreTake(quicConnIdMutex, M2T(0));
    uint16_t nextId = quicSrcConnId++;
    xSemaphoreGive(quicConnIdMutex);
    return nextId;
}

static void quicRxTask() {
    while(1) {
        vTaskDelay(M2T(1));
    }
}

int quicConnInit(UWB_Address_t peer, uint16_t srcConnId, uint16_t dstConnId) {
    if(quicClientNode.conns.size + 1 > quicClientNode.conns.capacity) {
        DEBUG_PRINT("Connection is full, can not resolve this connection.");
        return ERROR;
    }
    int srcConnId = srcConnId;
    int dstConnId = dstConnId;
    QUIC_Client_Conn_Item_t connItem;
    memset(&connItem, 0, sizeof(QUIC_Client_Conn_Item_t));
    // for(int i = 0; i < QUIC_CONNECTION_BUFFER_MAX; i++) {
    //     if(quicClientConnBuffer[i].connId == 0) { /* when connId is 0, we consider this buffer slot is empty. */
    //         connItem = &quicClientConnBuffer[i];
    //         break;
    //     } else if(i == QUIC_CONNECTION_BUFFER_MAX - 1) { /* the buffer is full, we can adjust constant QUIC_CONNECTION_BUFFER_MAX */
    //         DEBUG_PRINT("Connection buffer is full, can not resolve this connection.");
    //         return ERROR;
    //     }
    // }
    connItem.connId = srcConnId;
    connItem.currentState = QUIC_CONN_STATE_INIT;
    connItem.peer = peer;
    /* since memset, packetSeqTuple is already set 0. */
    mapSet(&quicClientNode.conns.connItemsMap, srcConnId, &connItem, sizeof(QUIC_Client_Conn_Item_t));
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

    quicClientNode.me = uwbGetAddress();
    quicClientNode.conns.size = 0;
    quicClientNode.conns.capacity = QUIC_CONNECTION_NUMBER_MAX;
    // memset(quicClientNode.conns.items, 0, sizeof(QUIC_Client_Conn_Item_t) * QUIC_CONNECTION_NUMBER_MAX); // TODO: may debug
    // mapInit(&quicClientConnBufferMap, MAP_TYPE_QUIC_CONN, MAP_NOT_COPY_ADDR, QUIC_CONNECTION_BUFFER_MAX);
    mapInit(&quicClientNode.conns.connItemsMap, MAP_TYPE_QUIC_CONN, MAP_NOT_COPY_ADDR, QUIC_CONNECTION_NUMBER_MAX);

    xTaskCreate(quicRxTask, ADHOC_DECK_QUIC_RX_TASK_NAME, UWB_TASK_STACK_SIZE, NULL, ADHOC_DECK_TASK_PRI, &quicRxTaskHandle);
}

/* Generate Packet Operations */
int quicGenerateInitialPacket(Quic_Initial_or_Handshake_Packet_t *packet, uint16_t srcConnId, uint16_t dstConnId) {
    memset(packet, 0, sizeof(Quic_Initial_or_Handshake_Packet_t));
    packet->headerForm = 1;
    packet->fixedBit = 1;
    packet->longPacketType = 0;
    packet->srcConnId = srcConnId;
    packet->dstConnId = dstConnId;
    // packet->packetNumber = connPacketSeqTuple[dstConnId % QUIC_CONNECTION_NUMBER_MAX].dstConnId == dstConnId ? connPacketSeqTuple[dstConnId % QUIC_CONNECTION_NUMBER_MAX].initialSeqNumber : 0; // TODO: modify it
    // int itemIndex = 0;
    // for(; itemIndex < QUIC_CONNECTION_BUFFER_MAX; itemIndex++) { // TODO: add map func.
    //     // if(quicClientNode.conns.items[itemIndex].connId == srcConnId) packet->packetNumber = 
    // }
    QUIC_Client_Conn_Item_t *connItem = (QUIC_Client_Conn_Item_t *) mapGet(&quicClientNode.conns.connItemsMap, srcConnId);
    packet->packetNumber = connItem->packetSeqTuple.initialSeqNumber;
    connItem->packetSeqTuple.initialSeqNumber++;
    packet->length = 0;

    return SUCCESS;
}

/* Generate Frame Operations */
int quicFromInitialOrHandshakeGenerateOnlyTypeFrame(Quic_Initial_or_Handshake_Packet_t *packet, QUIC_FRAME_TYPE type) {
    if(type > 3) {
        DEBUG_PRINT("Error in generate only type frame, type is compatibal.");
        return ERROR;
    }
    if(packet->length + sizeof(Quic_Only_Type_Frame_t) >= QUIC_INITIAL_OR_HANDSHAKE_PACKET_PAYLOAD_SIZE_MAX * sizeof(uint8_t)) {
        if(packet->longPacketType == 0) DEBUG_PRINT("Error in Initial packet add frame, payload overflow.");
        else DEBUG_PRINT("Error in handshake packet add frame, payload overflow.");
        return ERROR;
    }
    Quic_Only_Type_Frame_t *frame = (Quic_Only_Type_Frame_t *) (packet->packetPayload + packet->length);
    frame->type = type;
    ASSERT((unsigned long)packet->length + sizeof(Quic_Only_Type_Frame_t) < QUIC_INITIAL_OR_HANDSHAKE_PACKET_PAYLOAD_SIZE_MAX * sizeof(uint8_t));
    packet->length += sizeof(Quic_Only_Type_Frame_t);
    return SUCCESS;
}

/* Packet Interact Operations */
int quicSendInitialPacket(Quic_Initial_or_Handshake_Packet_t *initialPacket, UWB_Address_t peer) {
    UWB_Data_Packet_t dataTxPacket;
    dataTxPacket.header.type = UWB_DATA_MESSAGE_QUIC;
    dataTxPacket.header.srcAddress = quicClientNode.me;
    dataTxPacket.header.destAddress = peer;
    dataTxPacket.header.ttl = 10;
    // dataTxPacket.header.length = sizeof(UWB_Data_Packet_Header_t) + sizeof(Quic_Initial_or_Handshake_Packet_t) - (sizeof(uint8_t) * QUIC_INITIAL_OR_HANDSHAKE_PACKET_PAYLOAD_SIZE_MAX) + (unsigned long)initialPacket->length;

    int srcConnId = getNextSrcConnId();
    int dstConnId = 0;
    Quic_Initial_or_Handshake_Packet_t *initialPacket = (Quic_Initial_or_Handshake_Packet_t *) &dataTxPacket.payload;
    quicConnInit(peer, srcConnId, dstConnId); /* initialize local connection parameters */
    quicGenerateInitialPacket(initialPacket, srcConnId, dstConnId);
    // Quic_Only_Type_Frame_t *helloFrame = (Quic_Only_Type_Frame_t *) initialPacket->packetPayload;
    // quicGenerateOnlyTypeFrame(&helloFrame, QUIC_FRAME_HELLO);
    // quicInitialOrHandshakePacketAddOnlyTypeFrame(&initialPacket, &helloFrame);
    quicFromInitialOrHandshakeGenerateOnlyTypeFrame(initialPacket, QUIC_FRAME_HELLO);

    DEBUG_PRINT("quicSendInitialPacket: "); /* add parameters according to the debugging situation. */
    uwbSendDataPacketBlock(&dataTxPacket);
}

// TODO: Write send<--->receive and state machine
