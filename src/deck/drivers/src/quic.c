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

#define ERROR -1
#define SUCCESS 0

static QueueHandle_t rxPacketQueue;
static QueueHandle_t txInitialOrHandshakeBufferQueue;
static QueueHandle_t txOneRTTBufferQueue;
static TaskHandle_t quicRxTaskHandle;
static QUIC_Node_t quicClientNode; // TODO: may change name
static QUIC_Node_t quicServerNode;
// static QUIC_Client_Conn_Item_t quicClientConnBuffer[QUIC_CONNECTION_BUFFER_MAX] = {0};
// static QUIC_Packet_Seq_Number_Tuple_t connPacketSeqTuple[QUIC_CONNECTION_NUMBER_MAX] = {0};
static uint16_t quicSrcConnId = 1;
static uint16_t quicTempSrcConnId = 0; /* There can only be one at the same time */
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
    systemWaitStart();

    UWB_Data_Packet_t dataRxPacket;
    while(1) {
        if(uwbReceiveDataPacketBlock(UWB_DATA_MESSAGE_QUIC, &dataRxPacket)) {
            /* We may receive packet as client or server */
            // TODO: may add mutex
            UWB_Address_t peer = dataRxPacket.header.srcAddress;
            uint16_t payloadLen = dataRxPacket.header.length - sizeof(UWB_Data_Packet_Header_t);
            uint16_t curPosLen = 0;
            uint8_t peerStatus = 0;
            uint16_t peerDstConnId = 0;
            while(curPosLen < payloadLen) {
                uint8_t headerForm = dataRxPacket.payload[curPosLen] >> 7;
                int packetOffset = 0;
                if(headerForm == QUIC_LONG_HEADER) {
                    Quic_Long_Packet_t *packet = (Quic_Long_Packet_t *) &dataRxPacket.payload[curPosLen];
                    uint8_t type = packet->header.longPacketType;
                    peerStatus = packet->header.status;
                    peerDstConnId = packet->header.dstConnId;
                    switch(type) {
                        case QUIC_INITIAL_PACKET:
                            packetOffset = quicProcessInitialPacket(packet, peer);
                            break;
                        case QUIC_ZERO_RTT_PACKET:
                            break;
                        case QUIC_HANDSHAKE_PACKET:
                            packetOffset = quicProcessHandshakePacket(packet, peer);
                            break;
                        default:
                            DEBUG_PRINT("Error in quicRxTask, long packet type is not compatibal.");
                            break;
                    }
                } else if(headerForm == QUIC_SHORT_HEADER){
                    Quic_One_RTT_Packet_t *packet = (Quic_One_RTT_Packet_t *) &dataRxPacket.payload[curPosLen];
                }
                curPosLen += packetOffset;
            }
            /* Send integrated packets to peer node */
            if(peerStatus == QUIC_CLIENT) {
                QUIC_Server_Conn_Item_t *connItem = (QUIC_Server_Conn_Item_t *) mapGet(&quicServerNode.conns.connItemsMap, peerDstConnId);
                if(connItem == NULL) { /* When server receive connection request message */
                    connItem = (QUIC_Server_Conn_Item_t *) mapGet(&quicServerNode.conns.connItemsMap, quicTempSrcConnId);
                }
                ASSERT(connItem != NULL); /* If connection is still null, then connection is not exist */
                switch(connItem->currentState) {
                    case QUIC_SERVER_CONN_STATE_INIT:
                        quicServerSendConnReply(peer, quicTempSrcConnId);
                        break;
                    case QUIC_SERVER_CONN_STATE_HANDSHAKE_LISTEN:
                        quicServerSendConnDone(peer, peerDstConnId);
                        break;
                    case QUIC_SERVER_CONN_STATE_OPEN:
                        // TODO: Send 1-RTT packet
                        break;
                    default:
                        break;
                }
            } else if(peerStatus == QUIC_SERVER) {
                QUIC_Client_Conn_Item_t *connItem = (QUIC_Client_Conn_Item_t *) mapGet(&quicClientNode.conns.connItemsMap, peerDstConnId);
                ASSERT(connItem != NULL);
                switch(connItem->currentState) {
                    case QUIC_CLIENT_CONN_STATE_INIT:
                        quicClientSendConnRequest(peer);
                        break;
                    case QUIC_CLIENT_CONN_STATE_HANDSHAKE_REPLY:
                        quicClientSendConnReply(peer, connItem->connId);
                        break;
                    case QUIC_CLIENT_CONN_STATE_OPEN:
                        // TODO: Send 1-RTT packet
                        break;
                    default:
                        break;
                }
            }
        }
        vTaskDelay(M2T(1));
    }
}

int quicConnInit(UWB_Address_t peer, uint16_t srcConnId, uint16_t dstConnId) {
    if(quicClientNode.conns.size + 1 > quicClientNode.conns.capacity) {
        DEBUG_PRINT("Connection is full, can not resolve this connection.");
        return ERROR;
    }
    // int srcConnId = srcConnId;
    // int dstConnId = dstConnId;
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
    connItem.dstConnId = dstConnId;
    connItem.currentState = QUIC_CLIENT_CONN_STATE_INIT;
    connItem.peer = peer;
    /* since memset, packetSeqTuple is already set 0. */
    mapSet(&quicClientNode.conns.connItemsMap, srcConnId, &connItem, sizeof(QUIC_Client_Conn_Item_t));
    quicClientNode.conns.size++;
    return SUCCESS;
}

void quicInit() {
    rxPacketQueue = XQueueCreate(QUIC_RX_PACKET_QUEUE_SIZE, QUIC_RX_PACKET_ITEM_SIZE);
    txInitialOrHandshakeBufferQueue = xQueueCreate(QUIC_TX_LONG_BUFFER_QUEUE_SIZE, QUIC_TX_LONG_BUFFER_QUEUE_ITEM_SIZE);
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
    mapInit(&quicClientNode.conns.connItemsMap, MAP_TYPE_QUIC_CLIENT_CONN, MAP_NOT_COPY_ADDR, QUIC_CONNECTION_NUMBER_MAX);

    quicServerNode.me = uwbGetAddress();
    quicServerNode.conns.size = 0;
    quicServerNode.conns.capacity = QUIC_CONNECTION_NUMBER_MAX;
    mapInit(&quicServerNode.conns.connItemsMap, MAP_TYPE_QUIC_SERVER_CONN, MAP_NOT_COPY_ADDR, QUIC_CONNECTION_NUMBER_MAX);

    xTaskCreate(quicRxTask, ADHOC_DECK_QUIC_RX_TASK_NAME, UWB_TASK_STACK_SIZE, NULL, ADHOC_DECK_TASK_PRI, &quicRxTaskHandle);
}

/* Frame Operations */
int quicGenerateTypeFrame(Quic_Long_Packet_t *packet, uint16_t framePos, QUIC_FRAME_TYPE type) {
    if(type >= QUIC_LONG_PACKET_TYPE_COUNT) {
        DEBUG_PRINT("Error in generate only type frame, type is compatibal.");
        return ERROR;
    }
    if(packet->header.length + sizeof(Quic_Type_Frame_t) >= QUIC_LONG_PACKET_PAYLOAD_SIZE_MAX) {
        if(packet->header.longPacketType == QUIC_INITIAL_PACKET) DEBUG_PRINT("Error in Initial packet add frame, payload overflow.");
        else if(packet->header.longPacketType == QUIC_HANDSHAKE_PACKET) DEBUG_PRINT("Error in handshake packet add frame, payload overflow.");
        return ERROR;
    }
    Quic_Type_Frame_t *frame = (Quic_Type_Frame_t *) (packet->packetPayload[framePos]);
    frame->type = type;
    ASSERT((unsigned long)packet->header.length + sizeof(Quic_Type_Frame_t) < QUIC_LONG_PACKET_PAYLOAD_SIZE_MAX);
    packet->header.length += sizeof(Quic_Type_Frame_t);
    /* If handshake done frame */
    if(type == QUIC_FRAME_HANDSHAKE_DONE) {
        QUIC_Server_Conn_Item_t *connItem = (QUIC_Server_Conn_Item_t *) mapGet(&quicServerNode.conns.connItemsMap, packet->header.srcConnId);
        ASSERT(connItem != NULL);
        connItem->currentState = QUIC_SERVER_CONN_STATE_OPEN;
    }
    return sizeof(Quic_Type_Frame_t);
}

int quicHandleHelloFrame(Quic_Long_Packet_t *packet, int pos, UWB_Address_t peer) {
    ASSERT(packet->header.status == QUIC_CLIENT);
    DEBUG_PRINT("quicHandleHelloFrame: Hello frame received.\n");
    
    /* ADD-TODO: receive dupicated connection request */

    /* Create new connection for client */
    QUIC_Server_Conn_Item_t connItem;
    memset(&connItem, 0, sizeof(QUIC_Server_Conn_Item_t));
    connItem.peer = peer;
    connItem.connId = getNextSrcConnId();
    connItem.currentState = QUIC_SERVER_CONN_STATE_INIT;
    connItem.dstConnId = packet->header.srcConnId;
    connItem.packetSeqTuple.initialSeqNumber = 0;
    connItem.packetSeqTuple.zeroRTTSeqNumber = 0;
    connItem.packetSeqTuple.handshakeSeqNumber = 0;
    connItem.packetSeqTuple.oneRTTSeqNumber = 0;
    mapSet(&quicServerNode.conns.connItemsMap, connItem.connId, &connItem, sizeof(QUIC_Server_Conn_Item_t));
    quicServerNode.conns.size++;

    quicTempSrcConnId = connItem.connId; /* Will be used when server reply */
    /* Calculate length and return */
    return sizeof(Quic_Type_Frame_t);
}

int quicHandleHandshakeDoneFrame(Quic_Long_Packet_t *packet, int pos, UWB_Address_t peer) {
    ASSERT(packet->header.status == QUIC_CLIENT);
    DEBUG_PRINT("quicHandleHandshakeDoneFrame: Handshake done frame received.\n");
    QUIC_Client_Conn_Item_t *connItem = (QUIC_Client_Conn_Item_t *) mapGet(&quicClientNode.conns.connItemsMap, packet->header.dstConnId);
    ASSERT(connItem != NULL);
    connItem->currentState = QUIC_CLIENT_CONN_STATE_OPEN;
    return sizeof(Quic_Type_Frame_t);
}

int quicGenerateACKFrame(Quic_Long_Packet_t *packet, uint16_t framePos, void *connItem) {
    if(packet->header.length + sizeof(Quic_ACK_Frame_t) >= QUIC_LONG_PACKET_PAYLOAD_SIZE_MAX) {
        if(packet->header.longPacketType == QUIC_INITIAL_PACKET) DEBUG_PRINT("Error in Initial packet add ACK frame, payload overflow.");
        else if(packet->header.longPacketType == QUIC_HANDSHAKE_PACKET) DEBUG_PRINT("Error in handshake packet add ACK frame, payload overflow.");
        return ERROR;
    }
    /* Get connection states and write ACK frame*/
    Quic_ACK_Frame_t *frame = (Quic_ACK_Frame_t *) (packet->packetPayload[framePos]);
    frame->header.type = QUIC_FRAME_ACK;
    if(packet->header.status == QUIC_CLIENT) {
        QUIC_Client_Conn_Item_t *connItem = (QUIC_Client_Conn_Item_t *) connItem;
        ASSERT(connItem != NULL);
        frame->header.largestACK = connItem->packetReceiveWindow->largestACK;
        frame->header.ACKDelay = packet->header.headerForm == QUIC_LONG_HEADER ? 0 : QUIC_ACK_DELAY_MAX; // TODO: set timer and calculate delay;
        frame->header.ACKRangeCount = connItem->packetReceiveWindow->ackRangeCount; 
        frame->header.firstACKRange = connItem->packetReceiveWindow->largestACK - connItem->packetReceiveWindow->minimumACK;
        // TODO: set ACK ranges
        for(int i = 0; i < frame->header.ACKRangeCount; i++) {
            frame->ACKRange[i].gap = 0;
            frame->ACKRange[i].ackRangeLength = 0;
        }
    } else if (packet->header.status == QUIC_SERVER)
    {
        QUIC_Server_Conn_Item_t *connItem = (QUIC_Server_Conn_Item_t *) connItem;
        ASSERT(connItem != NULL);
        frame->header.largestACK = connItem->packetReceiveWindow->largestACK;
        frame->header.ACKDelay = packet->header.headerForm == QUIC_LONG_HEADER ? 0 : QUIC_ACK_DELAY_MAX; // TODO: set timer and calculate delay;
        frame->header.ACKRangeCount = connItem->packetReceiveWindow->ackRangeCount; 
        frame->header.firstACKRange = connItem->packetReceiveWindow->largestACK - connItem->packetReceiveWindow->minimumACK;
        // TODO: set ACK ranges
        for(int i = 0; i < frame->header.ACKRangeCount; i++) {
            frame->ACKRange[i].gap = 0;
            frame->ACKRange[i].ackRangeLength = 0;
        }
    }
    
    ASSERT((unsigned long)packet->header.length + sizeof(Quic_ACK_Frame_t) < QUIC_LONG_PACKET_PAYLOAD_SIZE_MAX);
    packet->header.length += sizeof(Quic_ACK_Frame_t) - ((QUIC_ACK_FRAME_RANGE_SIZE_MAX - frame->header.ACKRangeCount) * sizeof(Quic_ACK_Range_t));
    return sizeof(Quic_ACK_Frame_t);
}

int quicHandleACKFrame(Quic_Long_Packet_t *packet, int pos, UWB_Address_t peer) {
    int packetType = packet->header.longPacketType;
    int windowType = 0;
    switch(packetType) {
        case QUIC_INITIAL_PACKET:
            windowType = QUIC_INITIAL_PACKET_ACK;
        case QUIC_ZERO_RTT_PACKET:
            windowType = QUIC_ZERO_RTT_PACKET_ACK;
        case QUIC_HANDSHAKE_PACKET:
            windowType = QUIC_HANDSHAKE_PACKET_ACK;
        default:
            DEBUG_PRINT("Error in quicHandleACKFrame, packet type is not compatibal.");
            return ERROR;
    }
    Quic_ACK_Frame_t *frame = (Quic_ACK_Frame_t *) (packet->packetPayload[pos]);
    if(packet->header.status == QUIC_SERVER) {
        QUIC_Client_Conn_Item_t *connItem = (QUIC_Client_Conn_Item_t *) mapGet(&quicClientNode.conns.connItemsMap, packet->header.dstConnId);
        ASSERT(connItem != NULL);
        if(connItem->packetSendWindow[windowType].largestACK == frame->header.largestACK) {
            connItem->packetSendWindow[windowType].largestACK++;
            connItem->packetSendWindow[windowType].minimumACK++;
            connItem->packetSendWindow[windowType].ackRangeCount = 0;
        } else {
            // TODO: retransmission
        }
    } else if(packet->header.status == QUIC_CLIENT) {
        QUIC_Server_Conn_Item_t *connItem = (QUIC_Server_Conn_Item_t *) mapGet(&quicServerNode.conns.connItemsMap, packet->header.dstConnId);
        ASSERT(connItem != NULL);
        if(connItem->packetSendWindow[windowType].largestACK == frame->header.largestACK) {
            connItem->packetSendWindow[windowType].largestACK++;
            connItem->packetSendWindow[windowType].minimumACK++;
            connItem->packetSendWindow[windowType].ackRangeCount = 0;
        } else {
            // TODO: retransmission
        }
    }

    return sizeof(Quic_ACK_Frame_Header_t) + (frame->header.ACKRangeCount * sizeof(Quic_ACK_Range_t));
} //TODO: coding

int quicGenerateParameterFrame(Quic_Long_Packet_t *packet, uint16_t framePos) {
    if(packet->header.length + sizeof(Quic_Parameter_Frame_t) >= QUIC_LONG_PACKET_PAYLOAD_SIZE_MAX) {
        if(packet->header.longPacketType == QUIC_INITIAL_PACKET) DEBUG_PRINT("Error in Initial packet add parameter frame, payload overflow.");
        else if(packet->header.longPacketType == QUIC_HANDSHAKE_PACKET) DEBUG_PRINT("Error in handshake packet add parameter frame, payload overflow.");
        return ERROR;
    }
    Quic_Parameter_Frame_t *frame = (Quic_Parameter_Frame_t *) (packet->packetPayload[framePos]);
    frame->type = QUIC_FRAME_PARAMETER;
    /* Add parameters */
    int parameterPos = 0;
    int padding = 2 * sizeof(uint8_t);
    /* Max Idle Timeout */
    Quic_Parameter_Frame_Item_t *parameter = (Quic_Parameter_Frame_Item_t *) &frame->parameters[parameterPos];
    parameter->parameterID = QUIC_MAX_IDLE_TIMEOUT;
    parameter->parameterLength = sizeof(uint16_t);
    *((uint16_t *)&parameter->parameterValue) = QUIC_MAX_IDLE_TIMEOUT_DEFAULT;
    parameterPos += padding + parameter->parameterLength;
    frame->size++;
    /* Max routing payload size */
    parameter = (Quic_Parameter_Frame_Item_t *) &frame->parameters[parameterPos];
    parameter->parameterID = QUIC_MAX_ROUTE_PAYLOAD_SIZE;
    parameter->parameterLength = sizeof(uint16_t);
    *((uint16_t *)&parameter->parameterValue) = QUIC_MAX_ROUTE_PAYLOAD_SIZE_DEFAULT;
    parameterPos += padding + parameter->parameterLength;
    frame->size++;
    /* Initial max data */
    parameter = (Quic_Parameter_Frame_Item_t *) &frame->parameters[parameterPos];
    parameter->parameterID = QUIC_INITIAL_MAX_DATA;
    parameter->parameterLength = sizeof(uint16_t);
    *((uint16_t *)&parameter->parameterValue) = QUIC_INITIAL_MAX_DATA_DEFAULT;
    parameterPos += padding + parameter->parameterLength;
    frame->size++;
    /* Initial max stream data uni */
    parameter = (Quic_Parameter_Frame_Item_t *) &frame->parameters[parameterPos];
    parameter->parameterID = QUIC_INITIAL_MAX_STREAM_DATA_UNI;
    parameter->parameterLength = sizeof(uint16_t);
    *((uint16_t *)&parameter->parameterValue) = QUIC_INITIAL_MAX_STREAM_DATA_UNI_DEFAULT;
    parameterPos += padding + parameter->parameterLength;
    frame->size++;
    /* Initial max streams uni */
    parameter = (Quic_Parameter_Frame_Item_t *) &frame->parameters[parameterPos];
    parameter->parameterID = QUIC_INITIAL_MAX_STREAMS_UNI;
    parameter->parameterLength = sizeof(uint8_t);
    *((uint8_t *)&parameter->parameterValue) = QUIC_INITIAL_MAX_STREAMS_UNI_DEFAULT;
    parameterPos += padding + parameter->parameterLength;
    frame->size++;
    /* ACK delay exponent */
    parameter = (Quic_Parameter_Frame_Item_t *) &frame->parameters[parameterPos];
    parameter->parameterID = QUIC_ACK_DELAY_EXPONENT;
    parameter->parameterLength = sizeof(uint16_t);
    *((uint16_t *)&parameter->parameterValue) = QUIC_ACK_DELAY_EXPONENT_DEFAULT;
    parameterPos += padding + parameter->parameterLength;
    frame->size++;
    /* Max ACK delay */
    parameter = (Quic_Parameter_Frame_Item_t *) &frame->parameters[parameterPos];
    parameter->parameterID = QUIC_MAX_ACK_DELAY;
    parameter->parameterLength = sizeof(uint16_t);
    *((uint16_t *)&parameter->parameterValue) = QUIC_ACK_DELAY_MAX_DEFAULT;
    parameterPos += padding + parameter->parameterLength;
    frame->size++;
    /* Active connection ID limit */
    parameter = (Quic_Parameter_Frame_Item_t *) &frame->parameters[parameterPos];
    parameter->parameterID = QUIC_ACTIVE_CONNECTION_ID_LIMIT;
    parameter->parameterLength = sizeof(uint8_t);
    *((uint8_t *)&parameter->parameterValue) = QUIC_CONNECTION_NUMBER_MAX;
    parameterPos += padding + parameter->parameterLength;
    frame->size++;

    ASSERT((unsigned long)packet->header.length + 2 * sizeof(uint8_t) + parameterPos < QUIC_LONG_PACKET_PAYLOAD_SIZE_MAX);
    return parameterPos;
}

int quicHandleParameterFrame(Quic_Long_Packet_t *packet, int pos, UWB_Address_t peer) {
    if(packet->header.status != QUIC_SERVER) {
        DEBUG_PRINT("Error in quicHandleParameterFrame, peer status not right.");
        return ERROR;
    }
    QUIC_Client_Conn_Item_t *connItem = (QUIC_Client_Conn_Item_t *) mapGet(&quicClientNode.conns.connItemsMap, packet->header.dstConnId);
    Quic_Parameter_Frame_t *frame = (Quic_Parameter_Frame_t *) (packet->packetPayload[pos]);
    int parameterPos = 0;
    for(int i = 0; i < frame->size; i++) {
        Quic_Parameter_Frame_Item_t *parameter = (Quic_Parameter_Frame_Item_t *) &frame->parameters[parameterPos];
        // TODO: handle parameters
        switch(parameter->parameterID) {
            case QUIC_MAX_IDLE_TIMEOUT:
                break;
            case QUIC_MAX_ROUTE_PAYLOAD_SIZE:
                break;
            case QUIC_INITIAL_MAX_DATA:
                break;
            case QUIC_INITIAL_MAX_STREAM_DATA_UNI:
                break;
            case QUIC_INITIAL_MAX_STREAMS_UNI:
                break;
            case QUIC_ACK_DELAY_EXPONENT:
                break;
            case QUIC_MAX_ACK_DELAY:
                break;
            case QUIC_ACTIVE_CONNECTION_ID_LIMIT:
                break;
            default:
                DEBUG_PRINT("Error in quicHandleParameterFrame, parameter ID is not compatibal.");
                return ERROR;
        }
        parameterPos += 2 * sizeof(uint8_t) + parameter->parameterLength;
    }

    connItem->currentState = QUIC_CLIENT_CONN_STATE_HANDSHAKE_REPLY;

    return 2 * sizeof(uint8_t) + parameterPos;
}

/* Packet Operations */
int quicGenerateInitialPacket(Quic_Long_Packet_t *packet, uint16_t srcConnId, uint16_t dstConnId, void *connItem) {
    memset(packet, 0, sizeof(Quic_Long_Packet_Header_t));
    packet->header.headerForm = 1;
    packet->header.fixedBit = 1;
    packet->header.longPacketType = QUIC_INITIAL_PACKET;
    packet->header.srcConnId = srcConnId;
    packet->header.dstConnId = dstConnId;
    
    if(packet->header.status == QUIC_CLIENT) {
        QUIC_Client_Conn_Item_t *clientConnItem = (QUIC_Client_Conn_Item_t *) connItem;
        packet->header.packetNumber = clientConnItem->packetSeqTuple.initialSeqNumber;
        clientConnItem->packetSeqTuple.initialSeqNumber++;
    } else if(packet->header.status == QUIC_SERVER) {
        QUIC_Server_Conn_Item_t *serverConnItem = (QUIC_Server_Conn_Item_t *) connItem;
        packet->header.packetNumber = serverConnItem->packetSeqTuple.initialSeqNumber;
        serverConnItem->packetSeqTuple.initialSeqNumber++;
    }
    
    packet->header.length = sizeof(Quic_Long_Packet_Header_t);

    return sizeof(Quic_Long_Packet_Header_t);
}

int quicProcessInitialPacket(Quic_Long_Packet_t *initialPacket, UWB_Address_t peer) {
    uint16_t payloadLen = initialPacket->header.length - sizeof(Quic_Long_Packet_Header_t);
    uint16_t curPosLen = 0;
    while(curPosLen < payloadLen) {
        uint8_t type = initialPacket->packetPayload[curPosLen];
        int frameOffset = 0;
        switch(type) {
        case QUIC_FRAME_HELLO:
            frameOffset = quicHandleHelloFrame(initialPacket, curPosLen, peer);
            break;
        case QUIC_FRAME_ACK:
            frameOffset = quicHandleACKFrame(initialPacket, curPosLen, peer);
            break;
        default:
            DEBUG_PRINT("Error in process initial packet, frame type is not compatibal.");
            return ERROR;
        }
        curPosLen += frameOffset;
    }

    /* Initial packet receive ACK handle and ConnID handle */
    if(initialPacket->header.status == QUIC_CLIENT) {
        uint16_t connId = initialPacket->header.dstConnId == 0 ? quicTempSrcConnId : initialPacket->header.dstConnId;
        QUIC_Server_Conn_Item_t *connItem = (QUIC_Server_Conn_Item_t *) mapGet(&quicServerNode.conns.connItemsMap, connId);
        /* ConnID Handle */
        connItem->dstConnId = initialPacket->header.srcConnId;
        /* Receive ACK */
        ASSERT(connItem != NULL);
        connItem->packetReceiveWindow[QUIC_INITIAL_PACKET_ACK].largestACK = initialPacket->header.packetNumber;
        connItem->packetReceiveWindow[QUIC_INITIAL_PACKET_ACK].minimumACK = initialPacket->header.packetNumber;
    } else if(initialPacket->header.status == QUIC_SERVER) {
        QUIC_Client_Conn_Item_t *connItem = (QUIC_Client_Conn_Item_t *) mapGet(&quicClientNode.conns.connItemsMap, initialPacket->header.dstConnId);
        /* ConnID Handle */
        connItem->dstConnId = initialPacket->header.srcConnId;
        /* Receive ACK */
        ASSERT(connItem != NULL);
        connItem->packetReceiveWindow[QUIC_INITIAL_PACKET_ACK].largestACK = initialPacket->header.packetNumber;
        connItem->packetReceiveWindow[QUIC_INITIAL_PACKET_ACK].minimumACK = initialPacket->header.packetNumber;
    }
    
    return initialPacket->header.length;
}

int quicGenerateHandshakePacket(Quic_Long_Packet_t *packet, uint16_t srcConnId, uint16_t dstConnId, void *connItem) {
    memset(packet, 0, sizeof(Quic_Long_Packet_Header_t));
    packet->header.headerForm = 1;
    packet->header.fixedBit = 1;
    packet->header.longPacketType = QUIC_HANDSHAKE_PACKET;
    packet->header.srcConnId = srcConnId;
    packet->header.dstConnId = dstConnId;
    
    if(packet->header.status == QUIC_CLIENT) {
        QUIC_Client_Conn_Item_t *clientConnItem = (QUIC_Client_Conn_Item_t *) connItem;
        packet->header.packetNumber = clientConnItem->packetSeqTuple.initialSeqNumber;
        clientConnItem->packetSeqTuple.initialSeqNumber++;
    } else if(packet->header.status == QUIC_SERVER) {
        QUIC_Server_Conn_Item_t *serverConnItem = (QUIC_Server_Conn_Item_t *) connItem;
        packet->header.packetNumber = serverConnItem->packetSeqTuple.initialSeqNumber;
        serverConnItem->packetSeqTuple.initialSeqNumber++;
    }
    
    packet->header.length = sizeof(Quic_Long_Packet_Header_t);

    return sizeof(Quic_Long_Packet_Header_t);
}

int quicProcessHandshakePacket(Quic_Long_Packet_t *handshakePacket, UWB_Address_t peer) {
    uint16_t payloadLen = handshakePacket->header.length - sizeof(Quic_Long_Packet_Header_t);
    uint16_t curPosLen = 0;
    while(curPosLen < payloadLen) {
        uint8_t type = handshakePacket->packetPayload[curPosLen];
        int frameOffset = 0;
        switch(type) {
        case QUIC_FRAME_ACK:
            frameOffset = quicHandleACKFrame(handshakePacket, curPosLen, peer);
            break;
        case QUIC_FRAME_PARAMETER:
            frameOffset = quicHandleParameterFrame(handshakePacket, curPosLen, peer);
            break;
        case QUIC_FRAME_HANDSHAKE_DONE:
            frameOffset = quicHandleHandshakeDoneFrame(handshakePacket, curPosLen, peer);
            break;
        default:
            DEBUG_PRINT("Error in process handshake packet, frame type is not compatibal.");
            return ERROR;
        }
        curPosLen += frameOffset;
    }
}

/* Message Operations */
int quicClientSendConnRequest(UWB_Address_t peer) {
    UWB_Data_Packet_t dataTxPacket;
    dataTxPacket.header.type = UWB_DATA_MESSAGE_QUIC;
    dataTxPacket.header.srcAddress = quicClientNode.me;
    dataTxPacket.header.destAddress = peer;
    dataTxPacket.header.ttl = 10;
    dataTxPacket.header.length = sizeof(UWB_Data_Packet_Header_t);

    int srcConnId = getNextSrcConnId();
    int dstConnId = 0;
    /* Initialize local connection */
    quicConnInit(peer, srcConnId, dstConnId); 
    /* Generate packets */
    int packetPos = 0;
    QUIC_Client_Conn_Item_t *connItem = (QUIC_Client_Conn_Item_t *) mapGet(&quicClientNode.conns.connItemsMap, srcConnId);
    /* Generate initial packet */
    Quic_Long_Packet_t *initialPacket = (Quic_Long_Packet_t *) &dataTxPacket.payload[packetPos];
    initialPacket->header.status = QUIC_CLIENT;
    packetPos += quicGenerateInitialPacket(initialPacket, srcConnId, dstConnId, (void *)connItem);
    /* Generate frames */
    int framePos = 0;
    /* Generate Hello frame */
    framePos += quicGenerateTypeFrame(initialPacket, framePos, QUIC_FRAME_HELLO);
    packetPos += framePos;

    dataTxPacket.header.length += packetPos;

    DEBUG_PRINT("quicClientSendConnRequest: "); /* add parameters according to the debugging situation. */
    uwbSendDataPacketBlock(&dataTxPacket);
}

int quicServerSendConnReply(UWB_Address_t peer, uint16_t connId) {
    UWB_Data_Packet_t dataTxPacket;
    dataTxPacket.header.type = UWB_DATA_MESSAGE_QUIC;
    dataTxPacket.header.srcAddress = quicServerNode.me;
    dataTxPacket.header.destAddress = peer;
    dataTxPacket.header.ttl = 10;
    dataTxPacket.header.length = sizeof(UWB_Data_Packet_Header_t);
    /* Find connection with peer */
    QUIC_Server_Conn_Item_t *connItem = (QUIC_Server_Conn_Item_t *) mapGet(&quicServerNode.conns.connItemsMap, connId);
    /* Generate packets */
    int packetPos = 0;
    /* Generate initial packet */
    Quic_Long_Packet_t *initialPacket = (Quic_Long_Packet_t *) &dataTxPacket.payload[packetPos];
    initialPacket->header.status = QUIC_SERVER;
    packetPos += quicGenerateInitialPacket(initialPacket, connItem->connId, connItem->dstConnId, connItem);
    /* Generate fames */
    int framePos = 0;
    /* Generate ACK frame */
    framePos += quicGenerateACKFrame(initialPacket, framePos, (void *)connItem);
    packetPos += framePos;
    /* Generate handshake packet */
    Quic_Long_Packet_t *handshakePacket = (Quic_Long_Packet_t *) &dataTxPacket.payload[packetPos];
    packetPos += quicGenerateHandshakePacket(handshakePacket, connItem->connId, connItem->dstConnId, (void *)connItem);
    /* Generate frames */
    framePos = 0;
    /* Generate parameter frame */
    framePos += quicGenerateParameterFrame(handshakePacket, framePos);
    packetPos += framePos;

    dataTxPacket.header.length += packetPos;

    DEBUG_PRINT("quicServerSendConnReply: "); /* add parameters according to the debugging situation. */
    uwbSendDataPacketBlock(&dataTxPacket);
}

int quicClientSendConnReply(UWB_Address_t peer, uint16_t connId) {
    UWB_Data_Packet_t dataTxPacket;
    dataTxPacket.header.type = UWB_DATA_MESSAGE_QUIC;
    dataTxPacket.header.srcAddress = quicClientNode.me;
    dataTxPacket.header.destAddress = peer;
    dataTxPacket.header.ttl = 10;
    dataTxPacket.header.length = sizeof(UWB_Data_Packet_Header_t);
    /* Find connection with peer */
    QUIC_Client_Conn_Item_t *connItem = (QUIC_Client_Conn_Item_t *) mapGet(&quicClientNode.conns.connItemsMap, connId);
    /* Generate packets */
    int packetPos = 0;
    /* Generate initial packet */
    Quic_Long_Packet_t *initialPacket = (Quic_Long_Packet_t *) &dataTxPacket.payload[packetPos];
    initialPacket->header.status = QUIC_CLIENT;
    packetPos += quicGenerateInitialPacket(initialPacket, connItem->connId, connItem->dstConnId, (void *)connItem);
    /* Generate frames */
    int framePos = 0;
    /* Generate ACK frame */
    framePos += quicGenerateACKFrame(initialPacket, framePos, (void *)connItem);
    packetPos += framePos;
    /* Generate handshake packet */
    Quic_Long_Packet_t *handshakePacket = (Quic_Long_Packet_t *) &dataTxPacket.payload[packetPos];
    handshakePacket->header.status = QUIC_CLIENT;
    packetPos += quicGenerateHandshakePacket(handshakePacket, connItem->connId, connItem->dstConnId, (void *)connItem);
    /* Generate frames */
    int framePos = 0;
    /* Generate ACK frame */
    framePos += quicGenerateACKFrame(handshakePacket, framePos, (void *)connItem);
    packetPos += framePos;
    /* Generate 1-RTT packet */
    // TODO: Add 1-RTT packet

    dataTxPacket.header.length += packetPos;

    DEBUG_PRINT("quicClientSendConnReply: "); /* add parameters according to the debugging situation. */
    uwbSendDataPacketBlock(&dataTxPacket);
}

int quicServerSendConnDone(UWB_Address_t peer, uint16_t connId) {
    UWB_Data_Packet_t dataTxPacket;
    dataTxPacket.header.type = UWB_DATA_MESSAGE_QUIC;
    dataTxPacket.header.srcAddress = quicServerNode.me;
    dataTxPacket.header.destAddress = peer;
    dataTxPacket.header.ttl = 10;
    dataTxPacket.header.length = sizeof(UWB_Data_Packet_Header_t);
    /* Find connection with peer */
    QUIC_Server_Conn_Item_t *connItem = (QUIC_Server_Conn_Item_t *) mapGet(&quicServerNode.conns.connItemsMap, connId);
    /* Generate packets */
    int packetPos = 0;
    /* Generate handshake packet */
    Quic_Long_Packet_t *handshakePacket = (Quic_Long_Packet_t *) &dataTxPacket.payload[packetPos];
    handshakePacket->header.status = QUIC_SERVER;
    packetPos += quicGenerateHandshakePacket(handshakePacket, connItem->connId, connItem->dstConnId, (void *)connItem);
    /* Generate frames */
    int framePos = 0;
    /* Generate handshake done frame */
    framePos += quicGenerateTypeFrame(handshakePacket, framePos, QUIC_FRAME_HANDSHAKE_DONE);
    packetPos += framePos;
    /* Generate ACK frame */
    framePos += quicGenerateACKFrame(handshakePacket, framePos, (void *)connItem);
    packetPos += framePos;
    /* Generate 1-RTT packet */

    dataTxPacket.header.length += packetPos;

    DEBUG_PRINT("quicServerSendConnDone: "); /* add parameters according to the debugging situation. */
    uwbSendDataPacketBlock(&dataTxPacket);
}

/* Quic Interaction Operations */
