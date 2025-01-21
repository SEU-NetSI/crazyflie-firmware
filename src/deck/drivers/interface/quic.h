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
#define QUIC_ONE_RTT_PACKET_PAYLOAD_SIZE_MAX (ROUTING_DATA_PACKET_PAYLOAD_SIZE_MAX - sizeof(Quic_Short_Packet_Header_t))
#define QUIC_ACK_FRAME_RANGE_SIZE_MAX ((MAX(QUIC_LONG_PACKET_PAYLOAD_SIZE_MAX, QUIC_ONE_RTT_PACKET_PAYLOAD_SIZE_MAX) - 18) / sizeof(Quic_ACK_Range_t))
#define QUIC_PARAMETER_FRAME_VALUE_LENGTH_MAX 4
#define QUIC_PARAMETER_FRAME_ITEM_SIZE_MAX 10 * QUIC_PARAMETER_FRAME_VALUE_LENGTH_MAX
#define QUIC_CONNECTION_NUMBER_MAX 16
#define QUIC_ACK_DELAY_MAX 2000 // 2 seconds
#define QUIC_STREAM_FRAME_PAYLOAD_SIZE_MAX (QUIC_ONE_RTT_PACKET_PAYLOAD_SIZE_MAX - sizeof(Quic_Stream_Frame_Header_t))

/* QUIC Parameters */
#define QUIC_PARAMETER_NUMBER 10
#define QUIC_MAX_IDLE_TIMEOUT_DEFAULT 3000 // 3 seconds
#define QUIC_MAX_ROUTE_PAYLOAD_SIZE_DEFAULT ROUTING_DATA_PACKET_PAYLOAD_SIZE_MAX
#define QUIC_INITIAL_MAX_DATA_DEFAULT 1000 // 1KB
#define QUIC_INITIAL_MAX_STREAM_DATA_UNI_DEFAULT 1000 // 1KB
#define QUIC_INITIAL_MAX_STREAMS_UNI_DEFAULT 10
#define QUIC_ACK_DELAY_EXPONENT_DEFAULT 3
#define QUIC_ACK_DELAY_MAX_DEFAULT 25 // 25 milliseconds

typedef uint16_t UWB_Connection_ID ;
/* QUIC Local Structs */
/* QUIC State Machine */
typedef enum {
    QUIC_CLIENT_CONN_STATE_INITIAL,
    QUIC_CLIENT_CONN_STATE_HELLO_SENT,
    QUIC_CLIENT_CONN_STATE_PARAMETER_HANDLED,
    QUIC_CLIENT_CONN_STATE_ACKNOWLEDGE_SENT,
    QUIC_CLIENT_CONN_STATE_OPEN,
    QUIC_CLIENT_CONN_STATE_CLOSE,
} QUIC_CLIENT_CONN_STATE_TYPE;

typedef enum {
    QUIC_SERVER_CONN_STATE_INITIAL,
    QUIC_SERVER_CONN_STATE_HELLO_HANDLED,
    QUIC_SERVER_CONN_STATE_PARAMETER_SENT,
    QUIC_SERVER_CONN_STATE_ACKNOWLEDGE_HANDLED,
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

/* Node Status */
typedef enum {
    QUIC_CLIENT,
    QUIC_SERVER,
} QUIC_NODE_STATUS;

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
    BlockList_t pendingBlockList;
    BlockList_t unackedBlockList;
    uint32_t sendOffset; // offset has been sent and acked
    uint32_t maxSendOffset; // max send window
    DataBlock_t *freeBlocks;
    bool sentFin;
} QUIC_Stream_Send_Buffer_t;

typedef struct {
    BlockList_t receiveBlockList;
    uint32_t readOffset; // the offset that has been submitted to the application
    uint32_t consumedOffset;
    uint32_t maxReceiveOffset; // max receive window
    DataBlock_t *freeBlocks;
    bool receivedFin;
} QUIC_Stream_Receive_Buffer_t;

typedef struct {
    uint8_t* data;
    uint32_t length;
    uint32_t offset;
    uint32_t packetNumber;
    bool isFin;
} QUIC_Stream_Sending_Data_t;

typedef struct {
    uint16_t streamId;
    QUIC_Stream_Send_Buffer_t dataBuffer;
    int (*writeSendBuffer)(QUIC_Stream_Send_Buffer_t *streamBuffer, const uint8_t *data, uint32_t dataLength);
    int (*readSendBuffer)(QUIC_Stream_Send_Buffer_t *streamBuffer, QUIC_Stream_Sending_Data_t *sendingData, uint32_t dataLength, uint32_t packetNumber);
    // TODO: add functions
} QUIC_Send_Stream_Item_t;

typedef struct {
    uint16_t streamId;
    QUIC_Stream_Receive_Buffer_t dataBuffer;
    int (*writeReceiveBuffer)(QUIC_Stream_Receive_Buffer_t *streamBuffer, const uint8_t *data, const uint32_t dataLength, const uint32_t offset);
    int (*readReceiveBuffer)(QUIC_Stream_Receive_Buffer_t *streamBuffer, const uint8_t *data, const uint32_t dataLength);
    // TODO: add functions
} QUIC_Read_Stream_Item_t;

typedef struct {
    uint16_t size;
    uint16_t capacity;
    Map_t streamsMap;
    /* Functions */
    void *(*streamItemSet)(Map_t *streamsMap, void *streamItem, uint16_t streamId);
    void *(*streamItemGet)(Map_t *streamsMap, uint16_t streamId);
} QUIC_Stream_t;

/* Packet Info, relative to ack, and the buffer */
typedef struct {
    uint32_t packetNumber;
    uint16_t streamId;
    uint32_t offset;
    uint32_t length;
    bool isAcked;
    bool isLost;
    uint8_t retransmitCount;
    DataBlock_t *dataBlock;
    // TODO: can add more info
} QUIC_Packet_Info_Node_t;

typedef struct {
    RBRoot_t *packetInfoRBTree; // the packet number is the key
    uint16_t packetInfoNodeCount; // the packet node number count
    uint32_t largestPacketNumber; // the largest packet number
    uint32_t minimumPacketUnackedNumber; // the minimum unacked packet number
    /* Functions */
    int (*packetInfoNodeInsert)(struct QUIC_Packet_Info_Manager_t *packetInfoManager, QUIC_Packet_Info_Node_t *packetInfoNode);
    int (*packetInfoNodeDelete)(const struct QUIC_Packet_Info_Manager_t *packetInfoManager, uint32_t packetNumber, struct QUIC_Client_Conn_Item_t *clientConnItem);
} QUIC_Packet_Info_Manager_t;

/* HEAD[| gap | acked len |] --> TAIL[| gap | acked len |] --> NULL */
typedef struct {
    uint16_t gap; // this is first
    uint16_t ackRangeLength; // this is second
    struct QUIC_ACK_Ranges_Block_t *next;
} QUIC_ACK_Ranges_Block_t;

typedef struct {
    QUIC_ACK_Ranges_Block_t *head; // get from head, old data
    QUIC_ACK_Ranges_Block_t *tail; // put to tail, new data
    uint16_t ackRangeCount; // range block numbers
} QUIC_ACK_Ranges_List_t;

typedef struct {
    QUIC_ACK_Ranges_List_t ackRangesList;
    QUIC_ACK_Ranges_Block_t *freeBlocks;
    uint16_t ackRangeCapacity; // total block numbers
} QUIC_ACK_Ranges_t;

typedef struct {
    uint32_t largestACK; // largest packet number, but not largest acked number
    uint32_t minimumACK; // minimum acked number
    QUIC_ACK_Ranges_t *ackRanges; /* For initial and handshake packets, this value is null. */
} QUIC_Packet_ACK_Window_t;

typedef struct {
    int size;
    uint16_t parameters[QUIC_PARAMETER_NUMBER];
} QUIC_Transport_Params_Tuple_t;

typedef struct {
    UWB_Address_t peer;
    uint16_t connId;
    uint16_t dstConnId;
    QUIC_CLIENT_CONN_STATE_TYPE currentState;
    QUIC_Transport_Params_Tuple_t transportParamsTuple;
    QUIC_Packet_ACK_Window_t packetSendWindow[QUIC_PACKET_ACK_COUNT - 1]; // for client, 1-RTT packet will not use it
    QUIC_Packet_ACK_Window_t packetReceiveWindow[QUIC_PACKET_ACK_COUNT - 1]; // for client, 1-RTT packet will not use it
    QUIC_Stream_t sendStreams;
    QUIC_Packet_Info_Manager_t packetInfoManager; // only client 1-RTT packet will use it, store the packet info with RBTree
} QUIC_Client_Conn_Item_t;

typedef struct {
    UWB_Address_t peer;
    uint16_t connId;
    uint16_t dstConnId;
    QUIC_SERVER_CONN_STATE_TYPE currentState;
    QUIC_Packet_ACK_Window_t packetSendWindow[QUIC_PACKET_ACK_COUNT];
    QUIC_Packet_ACK_Window_t packetReceiveWindow[QUIC_PACKET_ACK_COUNT];
    QUIC_Stream_t readStreams;
} QUIC_Server_Conn_Item_t;

typedef struct {
    uint16_t size;
    uint16_t capacity;
    Map_t connItemsMap; /* Hash table, to store conn parameters, type is QUIC_Client_Conn_Item_t*/
    /* Functions */
    void (*connItemSet)(Map_t *connItemsMap, uint16_t connId, void *connItem, QUIC_NODE_STATUS status);
    void *(*connItemGet)(Map_t *connItemsMap, uint16_t connId);
} QUIC_Conn_t;

typedef struct {
    UWB_Address_t me;
    SemaphoreHandle_t mu;
    QUIC_Conn_t conns;
} QUIC_Node_t;

/* QUIC Packets */
typedef enum {
    QUIC_LONG_HEADER,
    QUIC_SHORT_HEADER,
    QUIC_HEADER_FORM_COUNT,
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
    uint32_t length; /* Total packet length */ // TODO: Can be shorter
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
    uint16_t packetNumber;
    uint16_t length;
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
    QUIC_FRAME_PARAMETER,
    QUIC_FRAME_STREAM,
    QUIC_FRAME_STREAM_FIN,
    QUIC_FRAME_TYPE_COUNT
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
    uint16_t minimumACK;
} __attribute__((packed)) Quic_ACK_Frame_Header_t;

typedef struct {
    Quic_ACK_Frame_Header_t header;
    Quic_ACK_Range_t ACKRange[QUIC_ACK_FRAME_RANGE_SIZE_MAX]; // TODO: need to judge over range in the shorter packet
    //TODO: ECN
} __attribute__((packed)) Quic_ACK_Frame_t;

typedef struct {
    uint8_t parameterID;
    uint8_t parameterLength;
    uint8_t parameterValue[QUIC_PARAMETER_FRAME_VALUE_LENGTH_MAX];
} __attribute__((packed)) Quic_Parameter_Frame_Item_t;

typedef struct {
    uint8_t type;
    uint8_t size;
    uint8_t parameters[QUIC_PARAMETER_FRAME_ITEM_SIZE_MAX];
} __attribute__((packed)) Quic_Parameter_Frame_t;

typedef struct{
    uint8_t type;
    uint16_t streamID;
    uint32_t offset;
    uint32_t length;
} __attribute__((packed)) Quic_Stream_Frame_Header_t;

typedef struct{
    Quic_Stream_Frame_Header_t header;
    uint8_t data[QUIC_STREAM_FRAME_PAYLOAD_SIZE_MAX];
} __attribute__((packed)) Quic_Stream_Frame_t;

void quicInit();

/* Quic Operations */
/* Packet Operations */
int quicGenerateInitialPacket(Quic_Long_Packet_t *packet, const uint16_t srcConnId, const uint16_t dstConnId, void *connItem, QUIC_NODE_STATUS status);
int quicProcessInitialPacket(const Quic_Long_Packet_t *initialPacket, UWB_Address_t peer);
int quicGenerateHandshakePacket(Quic_Long_Packet_t *packet, const uint16_t srcConnId, const uint16_t dstConnId, void *connItem, QUIC_NODE_STATUS status);
int quicProcessHandshakePacket(Quic_Long_Packet_t *handshakePacket, const UWB_Address_t peer);
int quicGenerateOneRTTPacket(Quic_One_RTT_Packet_t *packet, uint16_t dstConnId, QUIC_NODE_STATUS status); // TODO: coding
int quicProcessOneRTTPacket(Quic_One_RTT_Packet_t *packet); // TODO: coding
/* Frame Operations */
int quicGenerateTypeFrame(Quic_Long_Packet_t *packet, uint16_t framePos, QUIC_FRAME_TYPE type); /* Enter the contents of the frame in the packet */
int quicHandleHelloFrame(const Quic_Long_Packet_t *packet, UWB_Address_t peer);
int quicHandleHandshakeDoneFrame(const Quic_Long_Packet_t *packet, int pos, UWB_Address_t peer);
int quicGenerateACKFrame(Quic_Long_Packet_t *packet, const int framePos, const void *connItem_);
int quicHandleACKFrame(const Quic_Long_Packet_t *packet, const int pos, UWB_Address_t peer);
int quicGenerateParameterFrame(Quic_Long_Packet_t *packet, const uint16_t framePos);
int quicHandleParameterFrame(Quic_Long_Packet_t *packet, const int pos, UWB_Address_t peer);
int quicGenerateStreamFrame(Quic_One_RTT_Packet_t *packet, uint16_t framePos, uint16_t connID, uint16_t streamID, uint32_t restLength); // TODO: modify
int quicHandleStreamFrame(Quic_One_RTT_Packet_t *packet, int pos); // TODO: coding
int quicGenerateOneRTTPacketACKFrame(Quic_One_RTT_Packet_t *packet, uint16_t framePos, const QUIC_Server_Conn_Item_t *connItem);
/* Message Operations */
int quicClientSendConnRequest(const UWB_Address_t peer);
int quicServerSendConnReply(const UWB_Address_t peer, const uint16_t connId);
int quicClientSendConnReply(const UWB_Address_t peer, const uint16_t connId);
int quicServerSendConnDone(const UWB_Address_t peer, const uint16_t connId);
int quicClientSendData(uint16_t connID, uint16_t streamID);

#endif