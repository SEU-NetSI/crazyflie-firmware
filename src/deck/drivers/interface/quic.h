#ifndef __QUIC_H__
#define __QUIC_H__

#include <stdint.h>
#include <quicTools.h>
#include "routing.h"

#define QUIC_DEBUG_ENABLE

/* Tools */
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

/* Queue Constants */
#define QUIC_RX_PACKET_QUEUE_SIZE 5
#define QUIC_RX_PACKET_ITEM_SIZE sizeof(UWB_Packet_t)

/* QUIC Constants */
#define QUIC_LONG_PACKET_PAYLOAD_SIZE_MAX (ROUTING_DATA_PACKET_PAYLOAD_SIZE_MAX - 14)
#define QUIC_ONE_RTT_PACKET_PAYLOAD_SIZE_MAX (ROUTING_DATA_PACKET_PAYLOAD_SIZE_MAX - 8)
#define QUIC_ACK_FRAME_RANGE_SIZE_MAX ((MAX(QUIC_LONG_PACKET_PAYLOAD_SIZE_MAX, QUIC_ONE_RTT_PACKET_PAYLOAD_SIZE_MAX) - 18) / sizeof(Quic_ACK_Range_t))
#define QUIC_PARAMETER_FRAME_VALUE_LENGTH_MAX 4
#define QUIC_PARAMETER_FRAME_ITEM_SIZE_MAX 10 * QUIC_PARAMETER_FRAME_VALUE_LENGTH_MAX
#define QUIC_CONNECTION_NUMBER_MAX 16
#define QUIC_ACK_DELAY_MAX 2000 // 2 seconds

/* QUIC Parameters */
#define QUIC_MAX_IDLE_TIMEOUT_DEFAULT 3000 // 3 seconds
#define QUIC_MAX_ROUTE_PAYLOAD_SIZE_DEFAULT ROUTING_DATA_PACKET_PAYLOAD_SIZE_MAX
#define QUIC_INITIAL_MAX_DATA_DEFAULT 1000 // 1KB
#define QUIC_INITIAL_MAX_STREAM_DATA_UNI_DEFAULT 1000 // 1KB
#define QUIC_INITIAL_MAX_STREAMS_UNI_DEFAULT 10
#define QUIC_ACK_DELAY_EXPONENT_DEFAULT 3
#define QUIC_ACK_DELAY_MAX_DEFAULT 25 // 25 milliseconds

/* QUIC Local Structs */
/* QUIC State Machine */
typedef enum {
    QUIC_CLIENT_CONN_STATE_INIT,
    QUIC_CLIENT_CONN_STATE_HANDSHAKE_REPLY,
    QUIC_CLIENT_CONN_STATE_OPEN,
    QUIC_CLIENT_CONN_STATE_CLOSE,
} QUIC_CLIENT_CONN_STATE_TYPE;

typedef enum {
    QUIC_SERVER_CONN_STATE_INIT,
    QUIC_SERVER_CONN_STATE_HANDSHAKE_LISTEN,
    QUIC_SERVER_CONN_STATE_OPEN,
    QUIC_SERVER_CONN_STATE_CLOSE,
} QUIC_SERVER_CONN_STATE_TYPE;

typedef enum {
    QUIC_INITIAL_PACKET_ACK,
    QUIC_ZERO_RTT_PACKET_ACK,
    QUIC_HANDSHAKE_PACKET_ACK,
    QUIC_ONE_RTT_PACKET_ACK,
    QUIC_PACKET_ACK_COUNT
} QUIC_PACKET_ACK_TYPE;

typedef struct {
    uint32_t largestACK;
    uint32_t minimumACK;
    uint16_t ackRangeCount;
    void *ackRanges; /* For initial and handshake packets, this value is null. */
} QUIC_Packet_ACK_Window_t;

typedef struct {
    uint32_t initialSeqNumber;
    uint32_t zeroRTTSeqNumber;
    uint32_t handshakeSeqNumber;
    uint32_t oneRTTSeqNumber;
} QUIC_Packet_Seq_Number_Tuple_t;

typedef struct {
    // TODO: add parameters
} QUIC_Transport_Params_Tuple_t;

typedef struct {
    UWB_Address_t peer;
    uint16_t connId;
    uint16_t dstConnId;
    QUIC_CLIENT_CONN_STATE_TYPE currentState;
    QUIC_Packet_Seq_Number_Tuple_t packetSeqTuple;
    QUIC_Transport_Params_Tuple_t transportParamsTuple;
    QUIC_Packet_ACK_Window_t packetSendWindow[QUIC_PACKET_ACK_COUNT];
    QUIC_Packet_ACK_Window_t packetReceiveWindow[QUIC_PACKET_ACK_COUNT];
} QUIC_Client_Conn_Item_t;

typedef struct {
    UWB_Address_t peer;
    uint16_t connId;
    uint16_t dstConnId;
    QUIC_SERVER_CONN_STATE_TYPE currentState;
    QUIC_Packet_Seq_Number_Tuple_t packetSeqTuple;
    QUIC_Packet_ACK_Window_t packetSendWindow[QUIC_PACKET_ACK_COUNT];
    QUIC_Packet_ACK_Window_t packetReceiveWindow[QUIC_PACKET_ACK_COUNT];
} QUIC_Server_Conn_Item_t;

typedef struct {
    uint16_t size;
    uint16_t capacity;
    // QUIC_Client_Conn_Item_t items[QUIC_CONNECTION_NUMBER_MAX];
    Map_t connItemsMap; /* Hash table, to store conn parameters, type is QUIC_Client_Conn_Item_t*/
} QUIC_Conn_t; // TODO: coding

typedef struct {
    UWB_Address_t me;
    QUIC_Conn_t conns;
} QUIC_Node_t; // TODO: coding

/* Node Status */
typedef enum {
    QUIC_CLIENT,
    QUIC_SERVER,
} QUIC_NODE_STATUS;

/* QUIC Packets */
typedef enum {
    QUIC_LONG_HEADER,
    QUIC_SHORT_HEADER
} QUIC_HEADER_FORM;

/* Long Header Packets */
typedef enum {
    QUIC_INITIAL_PACKET,
    QUIC_ZERO_RTT_PACKET,
    QUIC_HANDSHAKE_PACKET,
    QUIC_LONG_PACKET_TYPE_COUNT,
} QUIC_LONG_PACKET_TYPE;

typedef struct {
    uint8_t headerForm : 1;
    uint8_t fixedBit : 1;
    uint8_t longPacketType : 2;
    uint8_t status : 1;
    uint8_t reservedBits : 3;
    uint16_t dstConnId;
    uint16_t srcConnId;
    uint32_t packetNumber;
    uint32_t length; /* Total packet length */
} __attribute__((packed)) Quic_Long_Packet_Header_t;

typedef struct {
    Quic_Long_Packet_Header_t header;
    uint8_t packetPayload[QUIC_LONG_PACKET_PAYLOAD_SIZE_MAX];
} __attribute__((packed)) Quic_Long_Packet_t;

/* Short Header Packets */
typedef struct {
    uint8_t headerForm : 1;
    uint8_t fixedBit : 1;
    uint8_t status : 1;
    uint8_t reservedBits : 5;
    uint16_t dstConnId;
    uint32_t packetNumber;
} __attribute__((packed)) Quic_Short_Packet_Header_t;

typedef struct {
    Quic_Short_Packet_Header_t header;
    uint8_t packetPayload[QUIC_ONE_RTT_PACKET_PAYLOAD_SIZE_MAX];
} __attribute__((packed)) Quic_One_RTT_Packet_t;

/* QUIC Frames */
typedef enum{
    QUIC_FRAME_PADDING,
    QUIC_FRAME_PING,
    QUIC_FRAME_HELLO,
    QUIC_FRAME_HANDSHAKE_DONE,
    QUIC_FRAME_ACK,
    QUIC_FRAME_ACK_ECN,
    QUIC_FRAME_PARAMETER
} QUIC_FRAME_TYPE;

typedef struct {
    uint8_t type;
} __attribute__((packed)) Quic_Type_Frame_t; /* Such as Padding, Ping, Hello and Handshake_Done frame */

typedef struct {
    uint16_t gap;
    uint16_t ackRangeLength;
} __attribute__((packed)) Quic_ACK_Range_t;

typedef struct {
    uint8_t type;
    uint32_t largestACK;
    uint32_t ACKDelay;
    uint16_t ACKRangeCount;
    uint16_t firstACKRange;
} __attribute__((packed)) Quic_ACK_Frame_Header_t;

typedef struct {
    Quic_ACK_Frame_Header_t header;
    Quic_ACK_Range_t ACKRange[QUIC_ACK_FRAME_RANGE_SIZE_MAX]; // TODO: need to judge over range in the shorter packet
    //TODO: ECN
} __attribute__((packed)) Quic_ACK_Frame_t;

typedef enum {
    QUIC_ORIGINAL_DESTINATION_CONNECTION_ID,
    QUIC_MAX_IDLE_TIMEOUT,
    QUIC_MAX_ROUTE_PAYLOAD_SIZE,
    QUIC_INITIAL_MAX_DATA,
    QUIC_INITIAL_MAX_STREAM_DATA_UNI,
    QUIC_INITIAL_MAX_STREAMS_UNI,
    QUIC_ACK_DELAY_EXPONENT,
    QUIC_MAX_ACK_DELAY,
    QUIC_ACTIVE_CONNECTION_ID_LIMIT,
    QUIC_INITIAL_SOURCE_CONNECTION_ID,
    QUIC_PARAMETER_TYPE_COUNT,
} QUIC_PARAMETER_TYPE;

typedef struct {
    uint8_t parameterID;
    uint8_t parameterLength;
    uint8_t parameterValue[QUIC_PARAMETER_FRAME_VALUE_LENGTH_MAX];
} __attribute__((packed)) Quic_Parameter_Frame_Item_t;

typedef struct {
    uint8_t type;
    uint8_t size;
    uint8_t parameters[QUIC_PARAMETER_FRAME_ITEM_SIZE_MAX]; //TODO: calculate number of parameter size
} __attribute__((packed)) Quic_Parameter_Frame_t;

/* Quic Operations */
/* Connection Operations */
int quicConnInit(UWB_Address_t peer, uint16_t srcConnId, uint16_t dstConnId); // TODO: coding
/* Packet Operations */
int quicGenerateInitialPacket(Quic_Long_Packet_t *packet, uint16_t srcConnId, uint16_t dstConnId, void *connItem);
int quicProcessInitialPacket(Quic_Long_Packet_t *initialPacket, UWB_Address_t peer);
int quicGenerateHandshakePacket(Quic_Long_Packet_t *packet, uint16_t srcConnId, uint16_t dstConnId, void *connItem);
int quicProcessHandshakePacket(Quic_Long_Packet_t *handshakePacket, UWB_Address_t peer);
/* Frame Operations */
int quicGenerateTypeFrame(Quic_Long_Packet_t *packet, uint16_t framePos, int type); /* Enter the contents of the frame in the packet */
int quicHandleHelloFrame(Quic_Long_Packet_t *packet, int pos, UWB_Address_t peer);
int quicHandleHandshakeDoneFrame(Quic_Long_Packet_t *packet, int pos, UWB_Address_t peer);
int quicGenerateACKFrame(Quic_Long_Packet_t *packet, uint16_t framePos, void *connItem_);
int quicHandleACKFrame(Quic_Long_Packet_t *packet, int pos, UWB_Address_t peer);
int quicGenerateParameterFrame(Quic_Long_Packet_t *packet, uint16_t framePos);
int quicHandleParameterFrame(Quic_Long_Packet_t *packet, int pos, UWB_Address_t peer);
/* Message Operations */
int quicClientSendConnRequest(UWB_Address_t peer);
int quicServerSendConnReply(UWB_Address_t peer, uint16_t connId);
int quicClientSendConnReply(UWB_Address_t peer, uint16_t connId);
int quicServerSendConnDone(UWB_Address_t peer, uint16_t connId);
/* Interaction Operations */
void quicInit();
int quicConnect(); // TODO: coding
int quicSend(); // TODO: coding
int quicClose(); // TODO: coding

#endif