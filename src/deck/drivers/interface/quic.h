#ifndef __QUIC_H__
#define __QUIC_H__

#include <stdint.h>
#include <tools.h>
#include "routing.h"

#define QUIC_DEBUG_ENABLE

/* Tools */
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

/* Queue Constants */
#define QUIC_RX_PACKET_QUEUE_SIZE 5
#define QUIC_RX_PACKET_ITEM_SIZE sizeof(UWB_Packet_t)
#define QUIC_TX_INITIAL_OR_HANDSHAKE_BUFFER_QUEUE_SIZE 5
#define QUIC_TX_INITIAL_OR_HANDSHAKE_BUFFER_QUEUE_ITEM_SIZE sizeof(Quic_Initial_or_Handshake_Packet_t)
#define QUIC_TX_ONE_RTT_BUFFER_QUEUE_SIZE 5
#define QUIC_TX_ONE_RTT_BUFFER_QUEUE_ITEM_SIZE sizeof(Quic_One_RTT_Packet_t)

/* QUIC Constants */
#define QUIC_INITIAL_OR_HANDSHAKE_PACKET_PAYLOAD_SIZE_MAX (ROUTING_DATA_PACKET_PAYLOAD_SIZE_MAX - 14)
#define QUIC_ONE_RTT_PACKET_PAYLOAD_SIZE_MAX (ROUTING_DATA_PACKET_PAYLOAD_SIZE_MAX - 8)
#define QUIC_ACK_FRAME_RANGE_SIZE_MAX ((MAX(QUIC_INITIAL_OR_HANDSHAKE_PACKET_PAYLOAD_SIZE_MAX, QUIC_ONE_RTT_PACKET_PAYLOAD_SIZE_MAX) - 18) / (sizeof(uint32_t) / sizeof(uint8_t)))
#define QUIC_PARAMETER_FRAME_VALUE_LENGTH_MAX 8
#define QUIC_PARAMETER_FRAME_ITEM_SIZE_MAX 10
#define QUIC_CONNECTION_NUMBER_MAX 16
// #define QUIC_CONNECTION_BUFFER_MAX 8

/* QUIC Local Structs */
// TODO: node states machine
/* QUIC Client State Machine */
typedef enum {
    QUIC_CONN_STATE_INIT,
    QUIC_CONN_STATE_HANDSHAKE_REPLY,
    QUIC_CONN_STATE_OPEN,
    QUIC_CONN_STATE_CLOSE,
} QUIC_CLIENT_CONN_STATE_TYPE;

typedef struct {
    // uint16_t dstConnId; // TODO: delete this field
    uint32_t initialSeqNumber;
    uint32_t zeroRTTSeqNumber;
    uint32_t handshakeSeqNumber;
    uint32_t oneRTTSeqNumber;
} QUIC_Packet_Seq_Number_Tuple_t;

typedef struct {
    UWB_Address_t peer;
    uint16_t connId;
    QUIC_CLIENT_CONN_STATE_TYPE currentState;
    QUIC_Packet_Seq_Number_Tuple_t packetSeqTuple;
} QUIC_Client_Conn_Item_t; // TODO: combine packet seq tuple

typedef struct {
    uint16_t size;
    uint16_t capacity;
    // QUIC_Client_Conn_Item_t items[QUIC_CONNECTION_NUMBER_MAX];
    Map_t connItemsMap; /* Hash table, to store conn parameters */
} QUIC_Client_Conn_t; // TODO: coding

typedef struct {
    UWB_Address_t me;
    QUIC_Client_Conn_t conns;
} QUIC_Client_Node_t; // TODO: coding

/* QUIC Packets */
typedef enum {
    QUIC_LONG_HEADER,
    QUIC_SHORT_HEADER
} QUIC_HEADER_FORM;

/* Long Header Packets */
typedef enum {
    QUIC_INITIAL_PACKET,
    QUIC_0RTT_PACKET,
    QUIC_HANDSHAKE_PACKET,
} QUIC_LONG_PACKET_TYPE;

typedef struct {
    uint8_t headerForm : 1;
    uint8_t fixedBit : 1;
    uint8_t longPacketType : 2;
    uint8_t reservedBits : 4;
    uint16_t dstConnId;
    uint16_t srcConnId;
    uint32_t packetNumber;
    uint32_t length;
    uint8_t packetPayload[QUIC_INITIAL_OR_HANDSHAKE_PACKET_PAYLOAD_SIZE_MAX];
} __attribute__((packed)) Quic_Initial_or_Handshake_Packet_t;

/* Short Header Packets */
typedef struct {
    uint8_t headerForm : 1;
    uint8_t fixedBit : 1;
    uint8_t reservedBits : 6;
    uint16_t dstConnId;
    uint32_t packetNumber;
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
} __attribute__((packed)) Quic_Only_Type_Frame_t; /* Such as Padding, Ping, Hello and Handshake_Done frame */

typedef struct {
    uint8_t type;
    uint32_t largestACK;
    uint64_t ACKDelay;
    uint16_t ACKRangeCount;
    uint16_t firstACKRange;
    uint32_t ACKRange[QUIC_ACK_FRAME_RANGE_SIZE_MAX]; // TODO: need to judge over range in the shorter packet
    //TODO: ECN
} __attribute__((packed)) Quic_ACK_Frame_t;

typedef struct {
    uint8_t parameterID;
    uint8_t parameterLength;
    uint8_t parameterValue[QUIC_PARAMETER_FRAME_VALUE_LENGTH_MAX];
} __attribute__((packed)) Quic_Parameter_Frame_Item_t;

typedef struct {
    uint8_t type;
    Quic_Parameter_Frame_Item_t parameters[QUIC_PARAMETER_FRAME_ITEM_SIZE_MAX]; //TODO: calculate number of parameter size
} __attribute__((packed)) Quic_Parameter_Frame_t;

/* Quic Operations */
/* Connection Operations */
int quicConnInit(UWB_Address_t peer, uint16_t srcConnId, uint16_t dstConnId); // TODO: coding
/* Packet Operations */
int quicGenerateInitialPacket(Quic_Initial_or_Handshake_Packet_t *packet, uint16_t srcConnId, uint16_t dstConnId);
int quicSendInitialPacket(Quic_Initial_or_Handshake_Packet_t *initialPacket, UWB_Address_t peer); // TODO: coding
/* Frame Operations */
int quicFromInitialOrHandshakeGenerateOnlyTypeFrame(Quic_Initial_or_Handshake_Packet_t *packet, QUIC_FRAME_TYPE type);
/* Interaction Operations */
void quicInit();
int quicConnect(); // TODO: coding
int quicSend(); // TODO: coding
int quicClose(); // TODO: coding

#endif
