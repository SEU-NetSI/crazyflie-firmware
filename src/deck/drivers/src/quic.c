#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "debug.h"
#include "system.h"
#include "quic.h"

#include "routing.h"
#include "quicTools.h"

#ifndef QUIC_DEBUG_ENABLE
#undef DEBUG_PRINT
#define DEBUG_PRINT
#endif

/* Test */
#define QUIC_TEST_CLIENT
#define QUIC_TEST_SERVER

#define ERROR (-1)
#define SUCCESS 0

static QueueHandle_t rxPacketQueue;
static TaskHandle_t quicRxTaskHandle;
static TaskHandle_t quicTxTaskHandle;
static QUIC_Node_t quicClientNode;
static QUIC_Node_t quicServerNode;
static uint16_t quicSrcConnId = 1;
static uint16_t quicTempSrcConnId = 0; /* There can only be one at the same time */
static SemaphoreHandle_t quicConnIdMutex;

/* Local functions */
static uint16_t getNextSrcConnId() {
    xSemaphoreTake(quicConnIdMutex, M2T(0));
    const uint16_t nextId = quicSrcConnId++;
    xSemaphoreGive(quicConnIdMutex);
    return nextId;
}

static int quicStateTransport(const uint16_t connId, const QUIC_NODE_STATUS status) {
    /* Transport state */
    if (status == QUIC_CLIENT) {
        QUIC_Client_Conn_Item_t *clientConnItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, connId);
        const QUIC_CLIENT_CONN_STATE_TYPE state = clientConnItem->currentState;
        switch(state) {
            case QUIC_CLIENT_CONN_STATE_INITIAL:
                clientConnItem->currentState = QUIC_CLIENT_CONN_STATE_HELLO_SENT;
                break;
            case QUIC_CLIENT_CONN_STATE_HELLO_SENT:
                if (clientConnItem->packetReceiveWindow[QUIC_INITIAL_PACKET_ACK].largestACK == 1 && clientConnItem->packetReceiveWindow[QUIC_HANDSHAKE_PACKET_ACK].largestACK == 1) {
                    clientConnItem->currentState = QUIC_CLIENT_CONN_STATE_PARAMETER_HANDLED;
                } else {
                    DEBUG_PRINT("In quicStateTransport: current client connection state is hello sent, but transport condition is not right.\n");
                }
                break;
            case QUIC_CLIENT_CONN_STATE_PARAMETER_HANDLED:
                clientConnItem->currentState = QUIC_CLIENT_CONN_STATE_ACKNOWLEDGE_SENT;
                break;
            case QUIC_CLIENT_CONN_STATE_ACKNOWLEDGE_SENT:
                if (clientConnItem->packetReceiveWindow[QUIC_INITIAL_PACKET_ACK].largestACK == 1 && clientConnItem->packetReceiveWindow[QUIC_HANDSHAKE_PACKET_ACK].largestACK == 2) {
                    clientConnItem->currentState = QUIC_CLIENT_CONN_STATE_OPEN;
                } else {
                    DEBUG_PRINT("In quicStateTransport: current client connection state is acknowledge sent, but transport condition is not right.\n");
                }
                break;
            case QUIC_CLIENT_CONN_STATE_OPEN:
                DEBUG_PRINT("In quicStateTransport: current client connection state is open.\n");
                break;
            default:
                break;
        }
    }
    if (status == QUIC_SERVER) {
        QUIC_Server_Conn_Item_t *serverConnItem = quicServerNode.conns.connItemGet(&quicServerNode.conns.connItemsMap, connId);
        const QUIC_SERVER_CONN_STATE_TYPE state = serverConnItem->currentState;
        switch(state) {
            case QUIC_SERVER_CONN_STATE_INITIAL:
                if (serverConnItem->packetReceiveWindow[QUIC_INITIAL_PACKET_ACK].largestACK == 1) {
                    serverConnItem->currentState = QUIC_SERVER_CONN_STATE_HELLO_HANDLED;
                } else {
                    DEBUG_PRINT("In quicStateTransport: current server connection state is initial, but transport condition is not right.\n");
                }
                break;
            case QUIC_SERVER_CONN_STATE_HELLO_HANDLED:
                serverConnItem->currentState = QUIC_SERVER_CONN_STATE_PARAMETER_SENT;
                break;
            case QUIC_SERVER_CONN_STATE_PARAMETER_SENT:
                if (serverConnItem->packetReceiveWindow[QUIC_INITIAL_PACKET_ACK].largestACK == 2 && serverConnItem->packetReceiveWindow[QUIC_HANDSHAKE_PACKET_ACK].largestACK == 1) {
                    serverConnItem->currentState = QUIC_SERVER_CONN_STATE_ACKNOWLEDGE_HANDLED;
                } else {
                    DEBUG_PRINT("In quicStateTransport: current server connection state is parameter sent, but transport condition is not right.\n");
                }
                break;
            case QUIC_SERVER_CONN_STATE_ACKNOWLEDGE_HANDLED:
                serverConnItem->currentState = QUIC_SERVER_CONN_STATE_OPEN;
                break;
            case QUIC_SERVER_CONN_STATE_OPEN:
                DEBUG_PRINT("In quicStateTransport: current server connection state is open.\n");
                break;
            default:
                break;
        }
    }
    return SUCCESS;
}

static void quicConnItemSet(Map_t *connItemsMap, const uint16_t connId, void *connItem, const QUIC_NODE_STATUS status) {
    char mapKey[10] = {0};
    itoa(connId, mapKey, 10);
    if (status == QUIC_CLIENT) {
        mapSet(connItemsMap, mapKey, connItem, sizeof(QUIC_Client_Conn_Item_t));
    }
    if (status == QUIC_SERVER) {
        mapSet(connItemsMap, mapKey, connItem, sizeof(QUIC_Server_Conn_Item_t));
    }
}

static void *quicConnItemGet(const Map_t *connItemsMap, const uint16_t connId) {
    char mapKey[10] = {0};
    itoa(connId, mapKey, 10);
    return mapGet(connItemsMap, mapKey);
}

static void quicConnItemsMapInit(QUIC_Conn_t *conn, const QUIC_NODE_STATUS status) {
    Map_t *connItemMap = &conn->connItemsMap;
    if (status == QUIC_CLIENT) {
        mapInit(connItemMap, MAP_TYPE_QUIC_CLIENT_CONN, MAP_COPY_ADDR, QUIC_CONNECTION_NUMBER_MAX, sizeof(QUIC_Client_Conn_Item_t));
    }
    if (status == QUIC_SERVER) {
        mapInit(connItemMap, MAP_TYPE_QUIC_SERVER_CONN, MAP_COPY_ADDR, QUIC_CONNECTION_NUMBER_MAX, sizeof(QUIC_Server_Conn_Item_t));
    }
    conn->connItemSet = quicConnItemSet;
    conn->connItemGet = quicConnItemGet;
}

static void quicTxTask() {
    systemWaitStart();

    while(true)
    {
#ifdef QUIC_TEST_CLIENT
        quicClientSendConnRequest(0);
#endif
        vTaskDelay(M2T(3000));
    }
}

static void quicRxTask() {
    systemWaitStart();

    UWB_Data_Packet_t dataRxPacket;
    while(true) {
        if(uwbReceiveDataPacketBlock(UWB_DATA_MESSAGE_QUIC, &dataRxPacket)) {
            /* We may receive packet as client or server */
            DEBUG_PRINT("Received QUIC packet from %u\n", dataRxPacket.header.srcAddress);
            const UWB_Address_t peer = dataRxPacket.header.srcAddress;
            const uint16_t payloadLen = dataRxPacket.header.length - sizeof(UWB_Data_Packet_Header_t);
            uint16_t curPosLen = 0;
            uint8_t peerStatus = 0;
            uint16_t peerDstConnId = 0;
            bool error = false;
            while(curPosLen < payloadLen) {
                const uint8_t headerForm = dataRxPacket.payload[curPosLen] >> 7;
                if (headerForm != QUIC_LONG_HEADER && headerForm != QUIC_SHORT_HEADER) {
                    DEBUG_PRINT("Error in quicRxTask, header form is not exist, packet analyse error, packet abandoned.\n");
                    break;
                }
                int packetOffset = 0;
                if(headerForm == QUIC_LONG_HEADER) {
                    Quic_Long_Packet_t *packet = (Quic_Long_Packet_t *) &dataRxPacket.payload[curPosLen];
                    const uint8_t type = packet->header.longPacketType;
                    peerStatus = packet->header.status;
                    peerDstConnId = packet->header.dstConnId;
                    switch(type) {
                        case QUIC_INITIAL_PACKET:
                            DEBUG_PRINT("In quicRxTask: Initial packet received.\n");
                            packetOffset = quicProcessInitialPacket(packet, peer);
                            break;
                        case QUIC_ZERO_RTT_PACKET:
                            break;
                        case QUIC_HANDSHAKE_PACKET:
                            DEBUG_PRINT("In quicRxTask: Handshake packet received.\n");
                            packetOffset = quicProcessHandshakePacket(packet, peer);
                            break;
                        default:
                            DEBUG_PRINT("Error in quicRxTask, long packet type is not exist.\n");
                            break;
                    }
                }
                if (headerForm == QUIC_SHORT_HEADER){
                    // Quic_One_RTT_Packet_t *packet = (Quic_One_RTT_Packet_t *) &dataRxPacket.payload[curPosLen];
                }
                error = packetOffset == ERROR;
                if (error) break;
                curPosLen += packetOffset;
            }
            /* State transport */
            if(peerStatus == QUIC_CLIENT && !error && peerDstConnId == 0) error = quicStateTransport(quicTempSrcConnId, QUIC_SERVER);
            else if(peerStatus == QUIC_CLIENT && !error && peerDstConnId != 0) error = quicStateTransport(peerDstConnId, QUIC_SERVER);
            else if(peerStatus == QUIC_SERVER && !error && peerDstConnId != 0) error = quicStateTransport(peerDstConnId, QUIC_CLIENT);
#ifdef QUIC_TEST_SERVER
            /* Send integrated packets to peer node */
            if(peerStatus == QUIC_CLIENT && !error) {
                const QUIC_Server_Conn_Item_t *connItem = quicServerNode.conns.connItemGet(&quicServerNode.conns.connItemsMap, peerDstConnId);
                if (connItem == NULL) connItem = quicServerNode.conns.connItemGet(&quicServerNode.conns.connItemsMap, quicTempSrcConnId); /* When server receive connection request message */
                if (connItem == NULL) {
                    DEBUG_PRINT("Error in server quicRxTask, connection item is not exist.\n");
                    break;
                }
                ASSERT(connItem != NULL); /* If connection is still null, then connection is not exist */
                switch(connItem->currentState) {
                    case QUIC_SERVER_CONN_STATE_HELLO_HANDLED:
                        quicServerSendConnReply(peer, quicTempSrcConnId);
                        break;
                    case QUIC_SERVER_CONN_STATE_ACKNOWLEDGE_HANDLED:
                        quicServerSendConnDone(peer, peerDstConnId);
                        break;
                    case QUIC_SERVER_CONN_STATE_OPEN:
                        // TODO: Send 1-RTT packet
                        DEBUG_PRINT("In quicRxTask: Send 1-RTT packet.\n");
                        break;
                    default:
                        break;
                }
            } else if(peerStatus == QUIC_SERVER && !error) {
                const QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, peerDstConnId);
                if (connItem == NULL) {
                    DEBUG_PRINT("Error in client quicRxTask, connection item is not exist.\n");
                    break;
                }
                ASSERT(connItem != NULL);
                switch(connItem->currentState) {
                    case QUIC_CLIENT_CONN_STATE_PARAMETER_HANDLED:
                        quicClientSendConnReply(peer, connItem->connId);
                        break;
                    case QUIC_CLIENT_CONN_STATE_OPEN:
                        // TODO: Send 1-RTT packet
                        DEBUG_PRINT("In quicRxTask: Send 1-RTT packet.\n");
                        break;
                    default:
                        break;
                }
            }
#endif
        }
        vTaskDelay(M2T(1));
    }
}

void quicInit() {
    rxPacketQueue = xQueueCreate(QUIC_RX_PACKET_QUEUE_SIZE, QUIC_RX_PACKET_ITEM_SIZE);
    quicConnIdMutex = xSemaphoreCreateMutex();
    UWB_Data_Packet_Listener_t listener = {
        .type = UWB_DATA_MESSAGE_QUIC,
        .rxQueue = rxPacketQueue
    };
    uwbRegisterDataPacketListener(&listener);

    quicClientNode.me = uwbGetAddress();
    quicClientNode.mu = xSemaphoreCreateMutex();
    quicClientNode.conns.size = 0;
    quicClientNode.conns.capacity = QUIC_CONNECTION_NUMBER_MAX;
    quicConnItemsMapInit(&quicClientNode.conns, QUIC_CLIENT);

    quicServerNode.me = uwbGetAddress();
    quicServerNode.mu = xSemaphoreCreateMutex();
    quicServerNode.conns.size = 0;
    quicServerNode.conns.capacity = QUIC_CONNECTION_NUMBER_MAX;
    quicConnItemsMapInit(&quicServerNode.conns, QUIC_SERVER);

    xTaskCreate(quicRxTask, ADHOC_DECK_QUIC_RX_TASK_NAME, UWB_TASK_STACK_SIZE, NULL, ADHOC_DECK_TASK_PRI, &quicRxTaskHandle);
    /* Test */
    xTaskCreate(quicTxTask, ADHOC_DECK_QUIC_TX_TASK_NAME, UWB_TASK_STACK_SIZE, NULL, ADHOC_DECK_TASK_PRI, &quicTxTaskHandle);
}

/* Frame Operations */
int quicGenerateTypeFrame(Quic_Long_Packet_t *packet, const uint16_t framePos, const QUIC_FRAME_TYPE type) {
    if(type >= QUIC_FRAME_PARAMETER) {
        DEBUG_PRINT("Error in generate only type frame, type is not exist.");
        return ERROR;
    }
    if(packet->header.length + sizeof(Quic_Type_Frame_t) >= QUIC_LONG_PACKET_PAYLOAD_SIZE_MAX) {
        if(packet->header.longPacketType == QUIC_INITIAL_PACKET) DEBUG_PRINT("Error in Initial packet add frame, payload overflow.");
        else if(packet->header.longPacketType == QUIC_HANDSHAKE_PACKET) DEBUG_PRINT("Error in handshake packet add frame, payload overflow.");
        return ERROR;
    }
    Quic_Type_Frame_t *frame = (Quic_Type_Frame_t *) &packet->packetPayload[framePos];
    memset(frame, 0, sizeof(Quic_Type_Frame_t));
    frame->type = type;
    ASSERT(packet->header.length + sizeof(Quic_Type_Frame_t) < QUIC_LONG_PACKET_PAYLOAD_SIZE_MAX);
    packet->header.length += sizeof(Quic_Type_Frame_t);
    // /* If handshake done frame */
    // if(type == QUIC_FRAME_HANDSHAKE_DONE) {
    //     QUIC_Server_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, packet->header.srcConnId);
    //     ASSERT(connItem != NULL);
    // }
    return sizeof(Quic_Type_Frame_t);
}

int quicHandleHelloFrame(const Quic_Long_Packet_t *packet, const UWB_Address_t peer) {
    ASSERT(packet->header.status == QUIC_CLIENT);
    /* receive duplicated connection request */
    if (quicServerNode.conns.connItemGet(&quicServerNode.conns.connItemsMap, packet->header.dstConnId) != NULL) {
        DEBUG_PRINT("Error in quicHandleHelloFrame, duplicated connection request.");
        return ERROR;
    }
    /* Create new connection for client */
    QUIC_Server_Conn_Item_t connItem = {0};
    connItem.peer = peer;
    connItem.connId = getNextSrcConnId();
    connItem.currentState = QUIC_SERVER_CONN_STATE_INITIAL;
    connItem.dstConnId = packet->header.srcConnId;
    quicServerNode.conns.connItemSet(&quicServerNode.conns.connItemsMap, connItem.connId, &connItem, QUIC_SERVER);
    quicServerNode.conns.size++;

    quicTempSrcConnId = connItem.connId; /* Will be used when server reply */
    /* Calculate length and return */
    return sizeof(Quic_Type_Frame_t);
}

int quicHandleHandshakeDoneFrame(const Quic_Long_Packet_t *packet, int pos, UWB_Address_t peer) {
    ASSERT(packet->header.status == QUIC_SERVER);
    DEBUG_PRINT("quicHandleHandshakeDoneFrame: Handshake done frame received.\n");
    // QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, packet->header.dstConnId);
    // ASSERT(connItem != NULL);
    // connItem->stateTransport(connItem, QUIC_CLIENT);
    return sizeof(Quic_Type_Frame_t);
}

int quicGenerateACKFrame(Quic_Long_Packet_t *packet, const int framePos, const void *connItem_) {
    if (connItem_ == NULL) {
        DEBUG_PRINT("Error in generate ACK frame, connection item is not exist.");
        return ERROR;
    }
    /* Get connection states and write ACK frame*/
    Quic_ACK_Frame_t *frame = (Quic_ACK_Frame_t *) &packet->packetPayload[framePos];
    memset(frame, 0, sizeof(Quic_ACK_Frame_Header_t));
    frame->header.type = QUIC_FRAME_ACK;
    /* Select ACK packet type */
    QUIC_PACKET_ACK_TYPE ackType = 0;
    switch (packet->header.longPacketType) {
        case QUIC_INITIAL_PACKET:
            ackType = QUIC_INITIAL_PACKET_ACK;
            break;
        case QUIC_HANDSHAKE_PACKET:
            ackType = QUIC_HANDSHAKE_PACKET_ACK;
            break;
        case QUIC_ZERO_RTT_PACKET:
            ackType = QUIC_ZERO_RTT_PACKET_ACK;
            break;
        default:
            break;
    }
    /* Generate ACK frame */
    if(packet->header.status == QUIC_CLIENT) {
        const QUIC_Client_Conn_Item_t *connItem = connItem_;
        ASSERT(connItem != NULL);
        frame->header.largestACK = connItem->packetReceiveWindow[ackType].largestACK;
        frame->header.ACKDelay = packet->header.headerForm == QUIC_LONG_HEADER ? 0 : QUIC_ACK_DELAY_MAX; // TODO: set timer and calculate delay;
        frame->header.ACKRangeCount = connItem->packetReceiveWindow[ackType].ackRangeCount;
        frame->header.firstACKRange = connItem->packetReceiveWindow[ackType].largestACK - connItem->packetReceiveWindow[ackType].minimumACK;
        // TODO: set ACK ranges when flow control is implemented
        for(int i = 0; i < frame->header.ACKRangeCount; i++) {
            frame->ACKRange[i].gap = 0;
            frame->ACKRange[i].ackRangeLength = 0;
        }
    }
    if (packet->header.status == QUIC_SERVER) {
        const QUIC_Server_Conn_Item_t *connItem = connItem_;
        ASSERT(connItem != NULL);
        frame->header.largestACK = connItem->packetReceiveWindow[ackType].largestACK;
        frame->header.ACKDelay = packet->header.headerForm == QUIC_LONG_HEADER ? 0 : QUIC_ACK_DELAY_MAX; // TODO: set timer and calculate delay;
        frame->header.ACKRangeCount = connItem->packetReceiveWindow[ackType].ackRangeCount;
        frame->header.firstACKRange = connItem->packetReceiveWindow[ackType].largestACK - connItem->packetReceiveWindow[ackType].minimumACK;
        // TODO: set ACK ranges when flow control is implemented
        for(int i = 0; i < frame->header.ACKRangeCount; i++) {
            frame->ACKRange[i].gap = 0;
            frame->ACKRange[i].ackRangeLength = 0;
        }
    }
    packet->header.length += sizeof(Quic_ACK_Frame_Header_t) + frame->header.ACKRangeCount * sizeof(Quic_ACK_Range_t);
    if(packet->header.length >= QUIC_LONG_PACKET_PAYLOAD_SIZE_MAX) {
        if(packet->header.longPacketType == QUIC_INITIAL_PACKET) DEBUG_PRINT("Error in Initial packet add ACK frame, payload overflow.\n");
        else if(packet->header.longPacketType == QUIC_HANDSHAKE_PACKET) DEBUG_PRINT("Error in handshake packet add ACK frame, payload overflow.\n");
        return ERROR;
    }
    ASSERT(packet->header.length < QUIC_LONG_PACKET_PAYLOAD_SIZE_MAX);
    // DEBUG_PRINT("In quicGenerateACKFrame: largestACK: %u, ACKDelay: %u, ACKRangeCount: %u, firstACKRange: %u.\n", frame->header.largestACK, frame->header.ACKDelay, frame->header.ACKRangeCount, frame->header.firstACKRange);
    return (int)sizeof(Quic_ACK_Frame_Header_t) + (int)(frame->header.ACKRangeCount * sizeof(Quic_ACK_Range_t));
}

int quicHandleACKFrame(const Quic_Long_Packet_t *packet, const int pos, UWB_Address_t peer) {
    const int packetType = packet->header.longPacketType;
    int windowType = 0;
    switch(packetType) {
        case QUIC_INITIAL_PACKET:
            windowType = QUIC_INITIAL_PACKET_ACK;
            break;
        case QUIC_ZERO_RTT_PACKET:
            windowType = QUIC_ZERO_RTT_PACKET_ACK;
            break;
        case QUIC_HANDSHAKE_PACKET:
            windowType = QUIC_HANDSHAKE_PACKET_ACK;
            break;
        default:
            DEBUG_PRINT("Error in quicHandleACKFrame, packet type is not exist.\n");
            return ERROR;
    }
    const Quic_ACK_Frame_t *frame = (Quic_ACK_Frame_t *) &packet->packetPayload[pos];
    if(packet->header.status == QUIC_SERVER) {
        const QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, packet->header.dstConnId);
        if (connItem == NULL) {
            DEBUG_PRINT("Error in quicHandleACKFrame, connection item is not exist.\n");
            return ERROR;
        }
        ASSERT(connItem != NULL);
        if(connItem->packetSendWindow[windowType].largestACK != frame->header.largestACK) {
            // TODO: retransmission last packet
            DEBUG_PRINT("In quicHandleACKFrame: retransmission last packet.\n");
        }
    } else if(packet->header.status == QUIC_CLIENT) {
        const QUIC_Server_Conn_Item_t *connItem = quicServerNode.conns.connItemGet(&quicServerNode.conns.connItemsMap, packet->header.dstConnId);
        if (connItem == NULL) {
            DEBUG_PRINT("Error in quicHandleACKFrame, connection item is not exist.\n");
            return ERROR;
        }
        ASSERT(connItem != NULL);
        if(connItem->packetSendWindow[windowType].largestACK != frame->header.largestACK) {
            // TODO: retransmission last packet
            DEBUG_PRINT("In quicHandleACKFrame: retransmission last packet.\n");
        }
    }

    return (int) sizeof(Quic_ACK_Frame_Header_t) + frame->header.ACKRangeCount * (int) sizeof(Quic_ACK_Range_t);
}

int quicGenerateParameterFrame(Quic_Long_Packet_t *packet, const uint16_t framePos) {
    if(packet->header.length + sizeof(Quic_Parameter_Frame_t) >= QUIC_LONG_PACKET_PAYLOAD_SIZE_MAX) {
        if(packet->header.longPacketType == QUIC_INITIAL_PACKET) DEBUG_PRINT("Error in Initial packet add parameter frame, payload overflow.");
        else if(packet->header.longPacketType == QUIC_HANDSHAKE_PACKET) DEBUG_PRINT("Error in handshake packet add parameter frame, payload overflow.");
        return ERROR;
    }
    Quic_Parameter_Frame_t *frame = (Quic_Parameter_Frame_t *) &packet->packetPayload[framePos];
    frame->type = QUIC_FRAME_PARAMETER;
    frame->size = 0;
    /* Add parameters */
    int parameterPos = 0;
    const int padding = 2 * sizeof(uint8_t);
    /* Max Idle Timeout */
    Quic_Parameter_Frame_Item_t *parameter = (Quic_Parameter_Frame_Item_t *) &frame->parameters[parameterPos];
    parameter->parameterID = QUIC_MAX_IDLE_TIMEOUT;
    parameter->parameterLength = sizeof(uint16_t);
    *(uint16_t *)&parameter->parameterValue = QUIC_MAX_IDLE_TIMEOUT_DEFAULT;
    parameterPos += padding + parameter->parameterLength;
    frame->size++;
    /* Max routing payload size */
    parameter = (Quic_Parameter_Frame_Item_t *) &frame->parameters[parameterPos];
    parameter->parameterID = QUIC_MAX_ROUTE_PAYLOAD_SIZE;
    parameter->parameterLength = sizeof(uint16_t);
    *(uint16_t *)&parameter->parameterValue = QUIC_MAX_ROUTE_PAYLOAD_SIZE_DEFAULT;
    parameterPos += padding + parameter->parameterLength;
    frame->size++;
    /* Initial max data */
    parameter = (Quic_Parameter_Frame_Item_t *) &frame->parameters[parameterPos];
    parameter->parameterID = QUIC_INITIAL_MAX_DATA;
    parameter->parameterLength = sizeof(uint16_t);
    *(uint16_t *)&parameter->parameterValue = QUIC_INITIAL_MAX_DATA_DEFAULT;
    parameterPos += padding + parameter->parameterLength;
    frame->size++;
    /* Initial max stream data uni */
    parameter = (Quic_Parameter_Frame_Item_t *) &frame->parameters[parameterPos];
    parameter->parameterID = QUIC_INITIAL_MAX_STREAM_DATA_UNI;
    parameter->parameterLength = sizeof(uint16_t);
    *(uint16_t *)&parameter->parameterValue = QUIC_INITIAL_MAX_STREAM_DATA_UNI_DEFAULT;
    parameterPos += padding + parameter->parameterLength;
    frame->size++;
    /* Initial max streams uni */
    parameter = (Quic_Parameter_Frame_Item_t *) &frame->parameters[parameterPos];
    parameter->parameterID = QUIC_INITIAL_MAX_STREAMS_UNI;
    parameter->parameterLength = sizeof(uint8_t);
    *(uint8_t *)&parameter->parameterValue = QUIC_INITIAL_MAX_STREAMS_UNI_DEFAULT;
    parameterPos += padding + parameter->parameterLength;
    frame->size++;
    /* ACK delay exponent */
    parameter = (Quic_Parameter_Frame_Item_t *) &frame->parameters[parameterPos];
    parameter->parameterID = QUIC_ACK_DELAY_EXPONENT;
    parameter->parameterLength = sizeof(uint16_t);
    *(uint16_t *)&parameter->parameterValue = QUIC_ACK_DELAY_EXPONENT_DEFAULT;
    parameterPos += padding + parameter->parameterLength;
    frame->size++;
    /* Max ACK delay */
    parameter = (Quic_Parameter_Frame_Item_t *) &frame->parameters[parameterPos];
    parameter->parameterID = QUIC_MAX_ACK_DELAY;
    parameter->parameterLength = sizeof(uint16_t);
    *(uint16_t *)&parameter->parameterValue = QUIC_ACK_DELAY_MAX_DEFAULT;
    parameterPos += padding + parameter->parameterLength;
    frame->size++;
    /* Active connection ID limit */
    parameter = (Quic_Parameter_Frame_Item_t *) &frame->parameters[parameterPos];
    parameter->parameterID = QUIC_ACTIVE_CONNECTION_ID_LIMIT;
    parameter->parameterLength = sizeof(uint8_t);
    *(uint8_t *)&parameter->parameterValue = QUIC_CONNECTION_NUMBER_MAX;
    parameterPos += padding + parameter->parameterLength;
    frame->size++;

    ASSERT(packet->header.length + 2 * sizeof(uint8_t) + parameterPos < QUIC_LONG_PACKET_PAYLOAD_SIZE_MAX);
    packet->header.length += 2 * sizeof(uint8_t) + parameterPos;
    return parameterPos + 2 * (int)sizeof(uint8_t);
}

int quicHandleParameterFrame(Quic_Long_Packet_t *packet, const int pos, UWB_Address_t peer) {
    if(packet->header.status != QUIC_SERVER) {
        DEBUG_PRINT("Error in quicHandleParameterFrame, peer status not right.");
        return ERROR;
    }
    QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, packet->header.dstConnId);
    Quic_Parameter_Frame_t *frame = (Quic_Parameter_Frame_t *) &packet->packetPayload[pos];
    int parameterPos = 0;
    const int padding = 2 * sizeof(uint8_t);
    for(int i = 0; i < frame->size; i++) {
        const Quic_Parameter_Frame_Item_t *parameter = (Quic_Parameter_Frame_Item_t *) &frame->parameters[parameterPos];
        // DEBUG_PRINT("In quicHandleParameterFrame: parameter ID: %u, parameter length: %u, \n", parameter->parameterID, parameter->parameterLength);
        switch(parameter->parameterID) {
            case QUIC_MAX_IDLE_TIMEOUT:
                connItem->transportParamsTuple.parameters[QUIC_MAX_IDLE_TIMEOUT] = *(uint16_t *)&parameter->parameterValue;
                // DEBUG_PRINT("parameter value: %u.\n", connItem->transportParamsTuple.parameters[QUIC_MAX_IDLE_TIMEOUT]);
                break;
            case QUIC_MAX_ROUTE_PAYLOAD_SIZE:
                connItem->transportParamsTuple.parameters[QUIC_MAX_ROUTE_PAYLOAD_SIZE] = *(uint16_t *)&parameter->parameterValue;
                // DEBUG_PRINT("parameter value: %u.\n", connItem->transportParamsTuple.parameters[QUIC_MAX_ROUTE_PAYLOAD_SIZE]);
                break;
            case QUIC_INITIAL_MAX_DATA:
                connItem->transportParamsTuple.parameters[QUIC_INITIAL_MAX_DATA] = *(uint16_t *)&parameter->parameterValue;
                // DEBUG_PRINT("parameter value: %u.\n", connItem->transportParamsTuple.parameters[QUIC_INITIAL_MAX_DATA]);
                break;
            case QUIC_INITIAL_MAX_STREAM_DATA_UNI:
                connItem->transportParamsTuple.parameters[QUIC_INITIAL_MAX_STREAM_DATA_UNI] = *(uint16_t *)&parameter->parameterValue;
                // DEBUG_PRINT("parameter value: %u.\n", connItem->transportParamsTuple.parameters[QUIC_INITIAL_MAX_STREAM_DATA_UNI]);
                break;
            case QUIC_INITIAL_MAX_STREAMS_UNI:
                connItem->transportParamsTuple.parameters[QUIC_INITIAL_MAX_STREAMS_UNI] = *(uint8_t *)&parameter->parameterValue;
                // DEBUG_PRINT("parameter value: %u.\n", connItem->transportParamsTuple.parameters[QUIC_INITIAL_MAX_STREAMS_UNI]);
                break;
            case QUIC_ACK_DELAY_EXPONENT:
                connItem->transportParamsTuple.parameters[QUIC_ACK_DELAY_EXPONENT] = *(uint16_t *)&parameter->parameterValue;
                // DEBUG_PRINT("parameter value: %u.\n", connItem->transportParamsTuple.parameters[QUIC_ACK_DELAY_EXPONENT]);
                break;
            case QUIC_MAX_ACK_DELAY:
                connItem->transportParamsTuple.parameters[QUIC_MAX_ACK_DELAY] = *(uint16_t *)&parameter->parameterValue;
                // DEBUG_PRINT("parameter value: %u.\n", connItem->transportParamsTuple.parameters[QUIC_MAX_ACK_DELAY]);
                break;
            case QUIC_ACTIVE_CONNECTION_ID_LIMIT:
                connItem->transportParamsTuple.parameters[QUIC_ACTIVE_CONNECTION_ID_LIMIT] = *(uint8_t *)&parameter->parameterValue;
                // DEBUG_PRINT("parameter value: %u.\n", connItem->transportParamsTuple.parameters[QUIC_ACTIVE_CONNECTION_ID_LIMIT]);
                break;
            default:
                DEBUG_PRINT("Error in quicHandleParameterFrame, parameter ID is not exist.");
                return ERROR;
        }
        parameterPos += padding + parameter->parameterLength;
    }

    connItem->transportParamsTuple.size = frame->size;

    return padding + parameterPos;
}

/* Packet Operations */
int quicGenerateInitialPacket(Quic_Long_Packet_t *packet, const uint16_t srcConnId, const uint16_t dstConnId, void *connItem, const QUIC_NODE_STATUS status) {
    memset(packet, 0, sizeof(Quic_Long_Packet_Header_t));
    packet->header.headerForm = 1;
    packet->header.fixedBit = 1;
    packet->header.longPacketType = QUIC_INITIAL_PACKET;
    packet->header.srcConnId = srcConnId;
    packet->header.dstConnId = dstConnId;
    packet->header.status = status;

    /* Send ACK handle and packet number handle */
    if(packet->header.status == QUIC_CLIENT) {
        QUIC_Client_Conn_Item_t *clientConnItem = connItem;
        clientConnItem->packetSendWindow[QUIC_INITIAL_PACKET_ACK].largestACK++;
        clientConnItem->packetSendWindow[QUIC_INITIAL_PACKET_ACK].minimumACK++;
        clientConnItem->packetSendWindow[QUIC_INITIAL_PACKET_ACK].ackRangeCount = 0;
        clientConnItem->packetSendWindow[QUIC_INITIAL_PACKET_ACK].ackRanges = NULL;
        packet->header.packetNumber = clientConnItem->packetSendWindow[QUIC_INITIAL_PACKET_ACK].largestACK;
    } else if(packet->header.status == QUIC_SERVER) {
        QUIC_Server_Conn_Item_t *serverConnItem = connItem;
        serverConnItem->packetSendWindow[QUIC_INITIAL_PACKET_ACK].largestACK++;
        serverConnItem->packetSendWindow[QUIC_INITIAL_PACKET_ACK].minimumACK++;
        serverConnItem->packetSendWindow[QUIC_INITIAL_PACKET_ACK].ackRangeCount = 0;
        serverConnItem->packetSendWindow[QUIC_INITIAL_PACKET_ACK].ackRanges = NULL;
        packet->header.packetNumber = serverConnItem->packetSendWindow[QUIC_INITIAL_PACKET_ACK].largestACK;
    }
    
    packet->header.length = sizeof(Quic_Long_Packet_Header_t);

    return sizeof(Quic_Long_Packet_Header_t);
}

int quicProcessInitialPacket(const Quic_Long_Packet_t *initialPacket, const UWB_Address_t peer) {
    const uint16_t payloadLen = initialPacket->header.length - sizeof(Quic_Long_Packet_Header_t);
    uint16_t curPosLen = 0;
    while(curPosLen < payloadLen) {
        const uint8_t type = initialPacket->packetPayload[curPosLen];
        int frameOffset = 0;
        switch(type) {
        case QUIC_FRAME_HELLO:
            DEBUG_PRINT("In quicProcessInitialPacket: handle Hello frame.\n");
            frameOffset = quicHandleHelloFrame(initialPacket, peer);
            break;
        case QUIC_FRAME_ACK:
            DEBUG_PRINT("In quicProcessInitialPacket: handle ack frame.\n");
            frameOffset = quicHandleACKFrame(initialPacket, curPosLen, peer);
            break;
        default:
            DEBUG_PRINT("Error in process initial packet, frame type is not exist.\n");
            return ERROR;
        }
        if (frameOffset == ERROR) {
            return ERROR;
        }
        curPosLen += frameOffset;
    }

    /* Initial packet receive ACK handle and ConnID handle */
    if(initialPacket->header.status == QUIC_CLIENT) {
        const uint16_t connId = initialPacket->header.dstConnId == 0 ? quicTempSrcConnId : initialPacket->header.dstConnId;
        QUIC_Server_Conn_Item_t *connItem = quicServerNode.conns.connItemGet(&quicServerNode.conns.connItemsMap, connId);
        /* ConnID Handle */
        connItem->dstConnId = initialPacket->header.srcConnId;
        /* Receive ACK */
        // MARK
        ASSERT(connItem != NULL);
        connItem->packetReceiveWindow[QUIC_INITIAL_PACKET_ACK].largestACK = initialPacket->header.packetNumber;
        connItem->packetReceiveWindow[QUIC_INITIAL_PACKET_ACK].minimumACK = initialPacket->header.packetNumber;
        connItem->packetReceiveWindow[QUIC_INITIAL_PACKET_ACK].ackRangeCount = 0;
        connItem->packetReceiveWindow[QUIC_INITIAL_PACKET_ACK].ackRanges = NULL;
    } else if(initialPacket->header.status == QUIC_SERVER) {
        QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, initialPacket->header.dstConnId);
        /* ConnID Handle */
        connItem->dstConnId = initialPacket->header.srcConnId;
        /* Receive ACK */
        ASSERT(connItem != NULL);
        connItem->packetReceiveWindow[QUIC_INITIAL_PACKET_ACK].largestACK = initialPacket->header.packetNumber;
        connItem->packetReceiveWindow[QUIC_INITIAL_PACKET_ACK].minimumACK = initialPacket->header.packetNumber;
        connItem->packetReceiveWindow[QUIC_INITIAL_PACKET_ACK].ackRangeCount = 0;
        connItem->packetReceiveWindow[QUIC_INITIAL_PACKET_ACK].ackRanges = NULL;
    }
    
    return (int) initialPacket->header.length;
}

int quicGenerateHandshakePacket(Quic_Long_Packet_t *packet, const uint16_t srcConnId, const uint16_t dstConnId, void *connItem, QUIC_NODE_STATUS status) {
    memset(packet, 0, sizeof(Quic_Long_Packet_Header_t));
    packet->header.headerForm = 1;
    packet->header.fixedBit = 1;
    packet->header.longPacketType = QUIC_HANDSHAKE_PACKET;
    packet->header.srcConnId = srcConnId;
    packet->header.dstConnId = dstConnId;
    packet->header.status = status;

    /* Handshake packet send ACK handle and packet number handle */
    if(packet->header.status == QUIC_CLIENT) {
        QUIC_Client_Conn_Item_t *clientConnItem = connItem;
        clientConnItem->packetSendWindow[QUIC_HANDSHAKE_PACKET_ACK].largestACK++;
        clientConnItem->packetSendWindow[QUIC_HANDSHAKE_PACKET_ACK].minimumACK++;
        clientConnItem->packetSendWindow[QUIC_HANDSHAKE_PACKET_ACK].ackRangeCount = 0;
        clientConnItem->packetSendWindow[QUIC_HANDSHAKE_PACKET_ACK].ackRanges = NULL;
        packet->header.packetNumber = clientConnItem->packetSendWindow[QUIC_HANDSHAKE_PACKET_ACK].largestACK;
    } else if(packet->header.status == QUIC_SERVER) {
        QUIC_Server_Conn_Item_t *serverConnItem = connItem;
        serverConnItem->packetSendWindow[QUIC_HANDSHAKE_PACKET_ACK].largestACK++;
        serverConnItem->packetSendWindow[QUIC_HANDSHAKE_PACKET_ACK].minimumACK++;
        serverConnItem->packetSendWindow[QUIC_HANDSHAKE_PACKET_ACK].ackRangeCount = 0;
        serverConnItem->packetSendWindow[QUIC_HANDSHAKE_PACKET_ACK].ackRanges = NULL;
        packet->header.packetNumber = serverConnItem->packetSendWindow[QUIC_HANDSHAKE_PACKET_ACK].largestACK;
    }
    
    packet->header.length = sizeof(Quic_Long_Packet_Header_t);

    return sizeof(Quic_Long_Packet_Header_t);
}

int quicProcessHandshakePacket(Quic_Long_Packet_t *handshakePacket, const UWB_Address_t peer) {
    const uint16_t payloadLen = handshakePacket->header.length - sizeof(Quic_Long_Packet_Header_t);
    uint16_t curPosLen = 0;
    while(curPosLen < payloadLen) {
        const uint8_t type = handshakePacket->packetPayload[curPosLen];
        int frameOffset = 0;
        switch(type) {
        case QUIC_FRAME_ACK:
            DEBUG_PRINT("In quicProcessHandshakePacket: handle ack frame.\n");
            frameOffset = quicHandleACKFrame(handshakePacket, curPosLen, peer);
            break;
        case QUIC_FRAME_PARAMETER:
            DEBUG_PRINT("In quicProcessHandshakePacket: handle parameter frame.\n");
            frameOffset = quicHandleParameterFrame(handshakePacket, curPosLen, peer);
            break;
        case QUIC_FRAME_HANDSHAKE_DONE:
            DEBUG_PRINT("In quicProcessHandshakePacket: handle handshake done frame.\n");
            frameOffset = quicHandleHandshakeDoneFrame(handshakePacket, curPosLen, peer);
            break;
        default:
            DEBUG_PRINT("Error in process handshake packet, frame type is not exist.\n");
            return ERROR;
        }
        if (frameOffset == ERROR) {
            return ERROR;
        }
        curPosLen += frameOffset;
    }

    /* Handshake packet receive ACK handle and ConnID handle */
    if(handshakePacket->header.status == QUIC_CLIENT) {
        QUIC_Server_Conn_Item_t *connItem = quicServerNode.conns.connItemGet(&quicServerNode.conns.connItemsMap, handshakePacket->header.dstConnId);
        if (connItem == NULL) {
            DEBUG_PRINT("Error in process handshake packet, connection item is not exist.");
            return ERROR;
        }
        /* Receive ACK */
        ASSERT(connItem != NULL);
        connItem->packetReceiveWindow[QUIC_HANDSHAKE_PACKET_ACK].largestACK = handshakePacket->header.packetNumber;
        connItem->packetReceiveWindow[QUIC_HANDSHAKE_PACKET_ACK].minimumACK = handshakePacket->header.packetNumber;
        connItem->packetReceiveWindow[QUIC_HANDSHAKE_PACKET_ACK].ackRangeCount = 0;
        connItem->packetReceiveWindow[QUIC_HANDSHAKE_PACKET_ACK].ackRanges = NULL;
    } else if(handshakePacket->header.status == QUIC_SERVER) {
        QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, handshakePacket->header.dstConnId);
        if (connItem == NULL) {
            DEBUG_PRINT("Error in process handshake packet, connection item is not exist.");
            return ERROR;
        }
        /* ConnID Handle */
        connItem->dstConnId = handshakePacket->header.srcConnId;
        /* Receive ACK */
        ASSERT(connItem != NULL);
        connItem->packetReceiveWindow[QUIC_HANDSHAKE_PACKET_ACK].largestACK = handshakePacket->header.packetNumber;
        connItem->packetReceiveWindow[QUIC_HANDSHAKE_PACKET_ACK].minimumACK = handshakePacket->header.packetNumber;
        connItem->packetReceiveWindow[QUIC_HANDSHAKE_PACKET_ACK].ackRangeCount = 0;
        connItem->packetReceiveWindow[QUIC_HANDSHAKE_PACKET_ACK].ackRanges = NULL;
    }

    return (int) handshakePacket->header.length;
}

/* Message Operations */
int quicClientSendConnRequest(const UWB_Address_t peer) {
    /* Steps:
     * 1. Generate initial packet
     * 2. Generate Hello frame
     * 3. Send packet
     */
    /* Generate data packet */
    UWB_Data_Packet_t dataTxPacket;
    dataTxPacket.header.type = UWB_DATA_MESSAGE_QUIC;
    dataTxPacket.header.srcAddress = quicClientNode.me;
    dataTxPacket.header.destAddress = peer;
    dataTxPacket.header.ttl = 10;
    dataTxPacket.header.length = sizeof(UWB_Data_Packet_Header_t);

    const int srcConnId = getNextSrcConnId();
    const int dstConnId = 0;
    /* Initialize local connection */
    /* Because we issued a connection request, we need to be prepared to maintain the connection. */
    if(quicClientNode.conns.size + 1 > quicClientNode.conns.capacity) {
        DEBUG_PRINT("Connection is full, can not resolve this connection.\n");
        return ERROR;
    }
    QUIC_Client_Conn_Item_t connItem_ = {0};
    connItem_.connId = srcConnId;
    connItem_.dstConnId = dstConnId;
    connItem_.currentState = QUIC_CLIENT_CONN_STATE_INITIAL;
    connItem_.peer = peer;
    /* since memset, packetTuples are already set 0. */
    quicClientNode.conns.connItemSet(&quicClientNode.conns.connItemsMap, srcConnId, &connItem_, QUIC_CLIENT);
    quicClientNode.conns.size++;
    /* Find connection with peer */
    QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, srcConnId);
    /* Generate packets */
    int packetPos = 0;
    /* Generate initial packet */
    Quic_Long_Packet_t *initialPacket = (Quic_Long_Packet_t *) &dataTxPacket.payload[packetPos];
    packetPos += quicGenerateInitialPacket(initialPacket, srcConnId, dstConnId, connItem, QUIC_CLIENT);
    /* Generate frames */
    int framePos = 0;
    /* Generate Hello frame */
    framePos += quicGenerateTypeFrame(initialPacket, framePos, QUIC_FRAME_HELLO);
    packetPos += framePos;

    dataTxPacket.header.length += packetPos;

    DEBUG_PRINT("In quicClientSendConnRequest: initial packet send.\n"); /* add parameters according to the debugging situation. */
    uwbSendDataPacketBlock(&dataTxPacket);
    quicStateTransport(srcConnId, QUIC_CLIENT);

    return SUCCESS;
}

int quicServerSendConnReply(const UWB_Address_t peer, const uint16_t connId) {
    /* Steps:
     * 1. Generate initial packet
     * 2. Generate ack frame
     * 3. Generate handshake packet
     * 4. Generate parameter frame
     * 5. Send packet
     */
    UWB_Data_Packet_t dataTxPacket;
    dataTxPacket.header.type = UWB_DATA_MESSAGE_QUIC;
    dataTxPacket.header.srcAddress = quicServerNode.me;
    dataTxPacket.header.destAddress = peer;
    dataTxPacket.header.ttl = 10;
    dataTxPacket.header.length = sizeof(UWB_Data_Packet_Header_t);
    /* Find connection with peer */
    QUIC_Server_Conn_Item_t *connItem = quicServerNode.conns.connItemGet(&quicServerNode.conns.connItemsMap, connId);
    if (connItem == NULL) return ERROR;
    /* Generate packets */
    int packetPos = 0;
    /* Generate initial packet */
    Quic_Long_Packet_t *initialPacket = (Quic_Long_Packet_t *) &dataTxPacket.payload[packetPos];
    packetPos += quicGenerateInitialPacket(initialPacket, connItem->connId, connItem->dstConnId, connItem, QUIC_SERVER);
    /* Generate frames */
    int framePos = 0;
    /* Generate ACK frame */
    framePos += quicGenerateACKFrame(initialPacket, framePos, connItem);
    packetPos += framePos;
    /* Generate handshake packet */
    Quic_Long_Packet_t *handshakePacket = (Quic_Long_Packet_t *) &dataTxPacket.payload[packetPos];
    packetPos += quicGenerateHandshakePacket(handshakePacket, connItem->connId, connItem->dstConnId, connItem, QUIC_SERVER);
    /* Generate frames */
    framePos = 0;
    /* Generate parameter frame */
    framePos += quicGenerateParameterFrame(handshakePacket, framePos);
    packetPos += framePos;

    dataTxPacket.header.length += packetPos;

    DEBUG_PRINT("In quicServerSendConnReply: initial and handshake packets send.\n"); /* add parameters according to the debugging situation. */
    uwbSendDataPacketBlock(&dataTxPacket);
    quicStateTransport(connId, QUIC_SERVER);

    return SUCCESS;
}

int quicClientSendConnReply(const UWB_Address_t peer, const uint16_t connId) {
    /* Steps:
     * 1. Generate initial packet
     * 2. Generate ack frame
     * 3. Generate handshake packet
     * 4. Generate ack frame
     * 5. Generate 1-RTT packet
     * 6. Generate stream frame
     * 7. Send packet
     */
    UWB_Data_Packet_t dataTxPacket;
    dataTxPacket.header.type = UWB_DATA_MESSAGE_QUIC;
    dataTxPacket.header.srcAddress = quicClientNode.me;
    dataTxPacket.header.destAddress = peer;
    dataTxPacket.header.ttl = 10;
    dataTxPacket.header.length = sizeof(UWB_Data_Packet_Header_t);
    /* Find connection with peer */
    QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, connId);
    /* Generate packets */
    int packetPos = 0;
    /* Generate initial packet */
    Quic_Long_Packet_t *initialPacket = (Quic_Long_Packet_t *) &dataTxPacket.payload[packetPos];
    packetPos += quicGenerateInitialPacket(initialPacket, connItem->connId, connItem->dstConnId, connItem, QUIC_CLIENT);
    /* Generate frames */
    int framePos = 0;
    /* Generate ACK frame */
    framePos += quicGenerateACKFrame(initialPacket, framePos, connItem);
    packetPos += framePos;
    /* Generate handshake packet */
    Quic_Long_Packet_t *handshakePacket = (Quic_Long_Packet_t *) &dataTxPacket.payload[packetPos];
    packetPos += quicGenerateHandshakePacket(handshakePacket, connItem->connId, connItem->dstConnId, connItem, QUIC_CLIENT);
    /* Generate frames */
    framePos = 0;
    /* Generate ACK frame */
    framePos += quicGenerateACKFrame(handshakePacket, framePos, connItem);
    packetPos += framePos;
    /* Generate 1-RTT packet */
    // TODO: Add 1-RTT packet

    dataTxPacket.header.length += packetPos;

    DEBUG_PRINT("In quicClientSendConnReply: initial, handshake and 1-RTT packets send.\n"); /* add parameters according to the debugging situation. */
    uwbSendDataPacketBlock(&dataTxPacket);
    quicStateTransport(connId, QUIC_CLIENT);

    return SUCCESS;
}

int quicServerSendConnDone(const UWB_Address_t peer, const uint16_t connId) {
    /* Steps:
     * 1. Generate handshake packet
     * 2. Generate handshake done frame
     * 3. Generate ack frame
     * 4. Generate 1-RTT packet
     * 5. Generate stream frame
     * 6. Send packet
     */
    UWB_Data_Packet_t dataTxPacket;
    dataTxPacket.header.type = UWB_DATA_MESSAGE_QUIC;
    dataTxPacket.header.srcAddress = quicServerNode.me;
    dataTxPacket.header.destAddress = peer;
    dataTxPacket.header.ttl = 10;
    dataTxPacket.header.length = sizeof(UWB_Data_Packet_Header_t);
    /* Find connection with peer */
    QUIC_Server_Conn_Item_t *connItem = quicServerNode.conns.connItemGet(&quicServerNode.conns.connItemsMap, connId);
    /* Generate packets */
    int packetPos = 0;
    /* Generate handshake packet */
    Quic_Long_Packet_t *handshakePacket = (Quic_Long_Packet_t *) &dataTxPacket.payload[packetPos];
    packetPos += quicGenerateHandshakePacket(handshakePacket, connItem->connId, connItem->dstConnId, connItem, QUIC_SERVER);
    /* Generate frames */
    int framePos = 0;
    /* Generate handshake done frame */
    framePos += quicGenerateTypeFrame(handshakePacket, framePos, QUIC_FRAME_HANDSHAKE_DONE);
    /* Generate ACK frame */
    framePos += quicGenerateACKFrame(handshakePacket, framePos, connItem);
    packetPos += framePos;
    /* Generate 1-RTT packet */
    // TODO: Add 1-RTT packet

    dataTxPacket.header.length += packetPos;

    DEBUG_PRINT("In quicServerSendConnDone: handshake and 1-RTT packets send.\n"); /* add parameters according to the debugging situation. */
    uwbSendDataPacketBlock(&dataTxPacket);
    quicStateTransport(connId, QUIC_SERVER);

    return SUCCESS;
}

/* Quic Interaction Operations */
/* TODO:
 * 1. 当报文超过负载了怎么办？
 * 2. 重传有两种，一种是超时重传，一种是接收到ACK后重传。
 * 3. 考虑发送失败的纠错机制。
 * 4. 状态转移的判断还需要改进。
 */