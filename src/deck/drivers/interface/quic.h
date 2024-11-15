#ifndef __QUIC_H__
#define __QUIC_H__

#include <stdint.h>
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
#define QUIC_INITIAL_OR_HANDSHAKE_PACKET_PAYLOAD_SIZE_MAX ((ROUTING_DATA_PACKET_PAYLOAD_SIZE_MAX - 14) / 8)
#define QUIC_ONE_RTT_PACKET_PAYLOAD_SIZE_MAX ((ROUTING_DATA_PACKET_PAYLOAD_SIZE_MAX - 8) / 8)
#define QUIC_ACK_FRAME_RANGE_SIZE_MAX ((MAX(QUIC_INITIAL_OR_HANDSHAKE_PACKET_PAYLOAD_SIZE_MAX, QUIC_ONE_RTT_PACKET_PAYLOAD_SIZE_MAX) - 18) / 32)
#define QUIC_PARAMETER_FRAME_VALUE_LENGTH_MAX 8
#define QUIC_PARAMETER_FRAME_ITEM_SIZE_MAX 10

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
    uint16_t destinationCID;
    uint16_t sourceCID;
    uint32_t packetNumber;
    uint32_t length;
    uint8_t packetPayload[QUIC_INITIAL_OR_HANDSHAKE_PACKET_PAYLOAD_SIZE_MAX];
} __attribute__((packed)) Quic_Initial_or_Handshake_Packet_t;

/* Short Header Packets */
typedef struct {
    uint8_t headerForm : 1;
    uint8_t fixedBit : 1;
    uint8_t reservedBits : 6;
    uint16_t destinationCID;
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

/* Quic Server Operations */
/* Generate Packet Operations */
int quicGenerateInitialPacket(Quic_Initial_or_Handshake_Packet_t *packet, uint16_t srcCID, uint16_t dstCID);
/* Generate Frame Operations */
int quicGenerateOnlyTypeFrame(Quic_Only_Type_Frame_t *frame, QUIC_FRAME_TYPE type);
/* Interaction Operations */
void quicInit();
int quicSendInitialPacket(Quic_Initial_or_Handshake_Packet_t *initialPacket, UWB_Address_t peerAddress);

/* Quic Client Operations */

#endif
