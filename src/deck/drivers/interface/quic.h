#ifndef QUIC_H_
#define QUIC_H_

#include <stdint.h>
#include <timers.h>
#include "quicTools.h"
#include "routing.h"
#define QUIC_DEBUG_ENABLE

/* Tools */
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

/* Queue Constants */
#define QUIC_RX_PACKET_QUEUE_SIZE 5
#define QUIC_RX_PACKET_ITEM_SIZE sizeof(UWB_Packet_t)
#define QUIC_STREAM_NOTIFY_QUEUE_SIZE 10
#define QUIC_STREAM_NOTIFY_QUEUE_ITEM_SIZE sizeof(QUIC_Transport_Info_t)

/* QUIC Constants */
#define QUIC_LONG_PACKET_PAYLOAD_SIZE_MAX (ROUTING_DATA_PACKET_PAYLOAD_SIZE_MAX - 14)
#define QUIC_ONE_RTT_PACKET_PAYLOAD_SIZE_MAX (ROUTING_DATA_PACKET_PAYLOAD_SIZE_MAX - sizeof(Quic_Short_Packet_Header_t))
#define QUIC_ACK_FRAME_RANGE_SIZE_MAX (QUIC_ONE_RTT_PACKET_PAYLOAD_SIZE_MAX - sizeof(Quic_ACK_Frame_Header_t)) / sizeof(Quic_ACK_Range_t) // only 1-RTT packet need it
#define QUIC_PARAMETER_FRAME_VALUE_LENGTH_MAX 4
#define QUIC_PARAMETER_FRAME_ITEM_SIZE_MAX (10 * QUIC_PARAMETER_FRAME_VALUE_LENGTH_MAX)
#define QUIC_CONNECTION_NUMBER_MAX 16
#define QUIC_STREAM_NUMBER_MAX 10
#define QUIC_STREAM_FRAME_PAYLOAD_SIZE_MAX (QUIC_ONE_RTT_PACKET_PAYLOAD_SIZE_MAX - sizeof(Quic_Stream_Frame_Header_t))
#define QUIC_STREAM_DATA_BLOCK_MAX_DATA_SIZE QUIC_STREAM_FRAME_PAYLOAD_SIZE_MAX
#define QUIC_STREAM_EXTEND_FRAME_PAYLOAD_SIZE_MAX (QUIC_ONE_RTT_PACKET_PAYLOAD_SIZE_MAX - sizeof(Quic_Stream_Frame_Extend_Header_t))
#define QUIC_ACK_RANGES_CAPACITY_DEFAULT 10
/* QUIC Timer */
#define QUIC_SERVER_CONN_TIMER_PERIOD_DEFAULT 100 // 100ms
#define QUIC_CLIENT_CONN_TIMER_PERIOD_DEFAULT 100 // 100ms

/* QUIC Parameters */
#define QUIC_PARAMETER_NUMBER 10
#define QUIC_MAX_IDLE_TIMEOUT_DEFAULT 1000 // 1 seconds
#define QUIC_MAX_ROUTE_PAYLOAD_SIZE_DEFAULT ROUTING_DATA_PACKET_PAYLOAD_SIZE_MAX
#define QUIC_INITIAL_MAX_DATA_DEFAULT 1000 // 1KB
#define QUIC_INITIAL_MAX_STREAM_DATA_UNI_DEFAULT 1000 // 1KB
#define QUIC_INITIAL_MAX_STREAMS_UNI_DEFAULT 10 // <= 10 streams
#define QUIC_ACK_DELAY_EXPONENT_DEFAULT 3
#define QUIC_ACK_DELAY_MAX_DEFAULT 200 // 200ms

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
    QUIC_MAX_IDLE_TIMEOUT,
    QUIC_MAX_ROUTE_PAYLOAD_SIZE,
    QUIC_INITIAL_MAX_DATA,
    QUIC_INITIAL_MAX_STREAM_DATA_UNI,
    QUIC_INITIAL_MAX_STREAMS_UNI,
    QUIC_ACK_DELAY_EXPONENT,
    QUIC_MAX_ACK_DELAY,
    QUIC_ACTIVE_CONNECTION_ID_LIMIT,
    QUIC_PARAMETER_TYPE_COUNT,
} QUIC_PARAMETER_TYPE;

typedef struct {
    BlockList_t pendingBlockList;
    BlockList_t unackedBlockList;
    uint32_t sendOffset; // offset that data has been sent
    uint32_t maxSendOffset; // this stream's max send offset, that means the file's length
    DataBlock_t *freeBlocksHead;
    bool sentFin;
} QUIC_Stream_Send_Buffer_t;

typedef struct {
    BlockList_t receiveBlockList;
    uint32_t readOffset; // the offset that has been submitted to the application
    uint32_t consumedOffset;
//    uint32_t maxReceiveOffset;
    DataBlock_t *freeBlocksHead;
    bool receivedFin;
    bool isIntegrity;
    // struct QUIC_Stream_Receive_Buffer_t *nextDataBuffer;
    // struct QUIC_Stream_Receive_Buffer_t *prevDataBuffer;
} QUIC_Stream_Receive_Buffer_t;

typedef struct {
    uint8_t* data;
    uint32_t length;
    uint32_t offset;
    uint32_t packetNumber;
    bool isFin;
    bool isHead;
} QUIC_Stream_Sending_Data_t;

typedef enum {
    QUIC_STREAM_SENDING_READY, // when create a stream, ready to send
    QUIC_STREAM_SENDING_SEND, // when sending data
    QUIC_STREAM_SENDING_DATA_SENT, // when send stream and fin, now retransmit lost packet
    QUIC_STREAM_SENDING_DATA_RECEIVED, // when receive all ack
    // QUIC_STREAM_SENDING_RESET_SENT, // when send RESET_STREAM
    // QUIC_STREAM_SENDING_RESET_RECEIVED, // when send RESET_STREAM before, and receive ack now
} QUIC_STREAM_SENDING_STATUS;

typedef struct {
    uint16_t streamId;
    QUIC_Stream_Send_Buffer_t dataBuffer;
    QUIC_STREAM_SENDING_STATUS sendingStatus;
    bool isHeadStream;
    uint16_t prevStreamID;
    uint16_t nextStreamID;
    struct QUIC_Client_Conn_Item_t *connItemPtr;
    int (*writeSendBuffer)(QUIC_Stream_Send_Buffer_t *streamBuffer, const uint8_t *data, uint32_t dataLength, bool isFin);
    int (*readSendBuffer)(QUIC_Stream_Send_Buffer_t *streamBuffer, QUIC_Stream_Sending_Data_t *sendingData, uint32_t dataLength, uint32_t packetNumber);
} QUIC_Send_Stream_Item_t;

typedef enum {
    QUIC_STREAM_RECEIVING_RECEIVE, // when create a stream, ready to receive
    QUIC_STREAM_RECEIVING_SIZE_KNOWN, // when receive stream and fin, now wait for retransmit lost packet
    QUIC_STREAM_RECEIVING_DATA_RECEIVED, // when receive all stream data
    QUIC_STREAM_RECEIVING_DATA_READY, // app read all data
    // QUIC_STREAM_RECEIVING_RESET_RECEIVED, // when receive RESET_STREAM
    // QUIC_STREAM_RECEIVING_RESET_READ, // app read reset
} QUIC_STREAM_RECEIVING_STATUS;

typedef struct {
    uint16_t streamId;
    uint16_t streamGroupId;
    QUIC_Stream_Receive_Buffer_t dataBuffer;
    QUIC_STREAM_RECEIVING_STATUS receivingStatus;
    bool isHeadStream;
    uint16_t prevStreamID;
    uint16_t nextStreamID;
    struct QUIC_Read_Stream_Item_t *prevStreamItem; // if this stream is in a group, this is the previous stream item
    struct QUIC_Read_Stream_Item_t *nextStreamItem; // if this stream is in a group, this is the next stream item
    struct QUIC_Server_Conn_Item_t *connItemPtr;
    int (*writeReceiveBuffer)(QUIC_Stream_Receive_Buffer_t *streamBuffer, const uint8_t *data, const uint32_t dataLength, const uint32_t offset, const bool isFin);
    int (*readReceiveBuffer)(QUIC_Stream_Receive_Buffer_t *streamBuffer, const uint8_t *data, const uint32_t dataLength);
} QUIC_Read_Stream_Item_t;

typedef struct {
    uint16_t size;
    uint16_t capacity;
    Map_t streamsMap;
    /* Functions */
    void *(*streamItemSet)(Map_t *streamsMap, void *streamItem, uint16_t streamId, QUIC_NODE_STATUS status);
    void *(*streamItemGet)(Map_t *streamsMap, uint16_t streamId);
} QUIC_Stream_t;

/* Packet Info, relative to ack, and the buffer */
typedef struct {
    uint32_t packetNumber;
    uint16_t connId;
    uint16_t streamId;
    uint32_t offset;
    uint32_t length;
    bool isAcked;
    bool isLost;
    TickType_t sendTime; // timestamp
    uint8_t retransmitCount;
    uint16_t retransmitPeriod;
    DataBlock_t *dataBlock;
} QUIC_Packet_Info_Node_t;

typedef struct {
    RBRoot_t *packetInfoRBTree; // the packet number is the key
    uint16_t packetInfoNodeCount; // the packet node number count
    uint32_t largestPacketNumber; // the largest packet number
    uint32_t minimumUnackedPacketNumber; // the minimum unacked packet number
    /* Functions */
    int (*packetInfoNodeInsert)(RBRoot_t *packetInfoRBTree, QUIC_Packet_Info_Node_t *packetInfoNode);
    int (*packetInfoNodeDelete)(RBRoot_t *packetInfoRBTree, uint32_t packetNumber, uint16_t connId);
} QUIC_Packet_Info_Manager_t;

/* ACK ranges */
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
    uint16_t connId; // my local connection id
    uint16_t dstConnId; // peer's connection id
    TaskHandle_t userTaskHandle; // used to notify the user task
    QUIC_CLIENT_CONN_STATE_TYPE currentState;
    QUIC_Transport_Params_Tuple_t transportParamsTuple;
    QUIC_Packet_ACK_Window_t packetSendWindow[QUIC_PACKET_ACK_COUNT - 1]; // for client, 1-RTT packet will not use it
    QUIC_Packet_ACK_Window_t packetReceiveWindow[QUIC_PACKET_ACK_COUNT - 1]; // for client, 1-RTT packet will not use it
    QUIC_Stream_t sendStreams;
    QUIC_Packet_Info_Manager_t packetInfoManager; // only client 1-RTT packet will use it, store the packet info with RBTree
    DataBlock_t *freeBlockPoolHead;
    xTimerHandle timer;
    uint16_t timerPeriod; // ms
    uint16_t connTimeout; // ms
    TickType_t lastReceiveTime; // timestamp, the value must be an integer multiple of the clock period
    uint16_t retransmitPeriod; // ms
} QUIC_Client_Conn_Item_t;

typedef struct {
    UWB_Address_t peer;
    uint16_t connId;
    uint16_t dstConnId;
    QUIC_SERVER_CONN_STATE_TYPE currentState;
    QUIC_Packet_ACK_Window_t packetSendWindow[QUIC_PACKET_ACK_COUNT]; // The 1-RTT packet is managed only as a packet number
    QUIC_Packet_ACK_Window_t packetReceiveWindow[QUIC_PACKET_ACK_COUNT];
    QUIC_Stream_t readStreams;
    // uint16_t minimumFinishedStreamId; // int this connection, the minimum ended stream id, | ok | ok | minimumStreamId | nok | nok |
    DataBlock_t *freeBlockPoolHead;
    xTimerHandle timer;
    uint16_t timerPeriod; // ms
    uint16_t connTimeout; // ms
    TickType_t lastReceiveTime; // timestamp, the value must be an integer multiple of the clock period
    uint16_t connACKPeriod; // ms
    TickType_t lastACKTime; // timestamp, the value must be an integer multiple of the clock period
} QUIC_Server_Conn_Item_t;

typedef struct {
    uint16_t size;
    uint16_t capacity;
    Map_t connItemsMap; /* Hash table, to store conn parameters, type is QUIC_Client_Conn_Item_t*/
    /* Functions */
    void *(*connItemSet)(Map_t *connItemsMap, uint16_t connId, void *connItem, QUIC_NODE_STATUS status);
    void *(*connItemGet)(Map_t *connItemsMap, uint16_t connId);
} QUIC_Conn_t;

/* Transport Information */
typedef struct {
    uint16_t peer;
    uint16_t connectionID;
    uint16_t streamID;
    uint16_t streamGroupID;
} QUIC_Transport_Info_t; // record running stream currently

typedef struct {
    UWB_Address_t me;
    SemaphoreHandle_t mu;
    QUIC_Transport_Info_t transportInfo[QUIC_INITIAL_MAX_STREAMS_UNI_DEFAULT];
    bool isOpen;
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
    QUIC_FRAME_HELLO,
    QUIC_FRAME_HANDSHAKE_DONE,
    QUIC_FRAME_ACK,
    QUIC_FRAME_ACK_ECN,
    QUIC_FRAME_PARAMETER,
    QUIC_FRAME_STREAM,
    QUIC_FRAME_STREAM_FIN,
    // QUIC_FRAME_STREAM_HEAD, // a stream group's first stream
    // QUIC_FRAME_STREAM_HEAD_FIN, // the first stream's fin frame
    QUIC_FRAME_STREAM_EXTEND,
    QUIC_FRAME_STREAM_EXTEND_AND_FIN,
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
    Quic_ACK_Range_t ACKRange[QUIC_ACK_FRAME_RANGE_SIZE_MAX];
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

typedef struct {
    uint8_t type;
    uint16_t streamID;
    // uint16_t preStreamID; // if these streams are in a group, this is the previous stream id
    uint32_t offset;
    uint32_t length;
} __attribute__((packed)) Quic_Stream_Frame_Header_t;

typedef struct {
    Quic_Stream_Frame_Header_t header;
    uint8_t data[QUIC_STREAM_FRAME_PAYLOAD_SIZE_MAX];
} __attribute__((packed)) Quic_Stream_Frame_t;

typedef struct {
    Quic_Stream_Frame_Header_t originHeader;
    uint16_t prevStreamID; // 0 means this is the head stream
    uint16_t nextStreamID; // 0 means this is the tail stream
} __attribute__((packed)) Quic_Stream_Frame_Extend_Header_t;

typedef struct {
    Quic_Stream_Frame_Extend_Header_t extendHeader;
    uint8_t data[QUIC_STREAM_EXTEND_FRAME_PAYLOAD_SIZE_MAX];
} __attribute__((packed)) Quic_Stream_Extend_Frame_t;

void quicInit();

/* Quic Operations */
/* Packet Operations */
int quicGenerateInitialPacket(Quic_Long_Packet_t *packet, uint16_t srcConnId, uint16_t dstConnId, void *connItem, QUIC_NODE_STATUS status);
int quicProcessInitialPacket(const Quic_Long_Packet_t *initialPacket, UWB_Address_t peer, TickType_t receiveTime);
int quicGenerateHandshakePacket(Quic_Long_Packet_t *packet, uint16_t srcConnId, uint16_t dstConnId, void *connItem, QUIC_NODE_STATUS status);
int quicProcessHandshakePacket(Quic_Long_Packet_t *handshakePacket, UWB_Address_t peer, TickType_t receiveTime);
int quicGenerateOneRTTPacket(Quic_One_RTT_Packet_t *packet, uint16_t dstConnId, QUIC_NODE_STATUS status);
int quicProcessOneRTTPacket(const Quic_One_RTT_Packet_t *packet, TickType_t receiveTime);
/* Frame Operations */
int quicGenerateTypeFrame(Quic_Long_Packet_t *packet, uint16_t framePos, QUIC_FRAME_TYPE type); /* Enter the contents of the frame in the packet */
int quicHandleHelloFrame(const Quic_Long_Packet_t *packet, UWB_Address_t peer);
int quicHandleHandshakeDoneFrame(const Quic_Long_Packet_t *packet, int pos, UWB_Address_t peer);
int quicGenerateACKFrame(Quic_Long_Packet_t *packet, int framePos, const void *connItem_);
int quicHandleACKFrame(const Quic_Long_Packet_t *packet, int pos, UWB_Address_t peer);
int quicGenerateParameterFrame(Quic_Long_Packet_t *packet, uint16_t framePos);
int quicHandleParameterFrame(Quic_Long_Packet_t *packet, int pos, UWB_Address_t peer);
int quicGenerateStreamFrame(Quic_One_RTT_Packet_t *packet, uint16_t framePos, uint16_t connID, uint16_t streamID, uint32_t restLength);
int quicHandleStreamFrame(const Quic_One_RTT_Packet_t *packet, int pos);
int quicGenerateOneRTTPacketACKFrame(Quic_One_RTT_Packet_t *packet, uint16_t framePos, const QUIC_Server_Conn_Item_t *connItem);
int quicHandleOneRTTPacketACKFrame(const Quic_One_RTT_Packet_t *packet, int pos);
int quicGenerateResendStreamFrame(Quic_One_RTT_Packet_t *packet, uint16_t framePos, QUIC_Packet_Info_Node_t *packetInfo);
/* Connection Operations */
int quicClientSendConnRequest(UWB_Address_t peer, TaskHandle_t userTaskHandle);
int quicServerSendConnReply(UWB_Address_t peer, uint16_t connId);
int quicClientSendConnReply(UWB_Address_t peer, uint16_t connId);
int quicServerSendConnDone(UWB_Address_t peer, uint16_t connId);
int quicServerSendACK(UWB_Address_t peer, uint16_t connId);
int quicServerConnClose(uint16_t connId);
int quicClientConnClose(uint16_t connId);
/* Stream Operations */
uint16_t getNextStreamId();
int quicSendStreamCreate(uint16_t connID, uint16_t streamID ,uint16_t prevStreamID, uint16_t nextStreamID);
int quicSendStreamWrite(uint16_t connID, uint16_t streamID, const uint8_t *data, uint32_t len, bool isLastSegment);
int quicClientSendData(uint16_t connID, uint16_t streamID);
int quicReceiveStreamRead(uint16_t connectionId, uint16_t streamId, uint8_t *cache, uint32_t len);
int quicReceiveStreamClose(uint16_t connectionId, uint16_t streamId);
int quicClientResendData(QUIC_Packet_Info_Node_t *packetInfo);
int quicReadStreamClear(QUIC_Read_Stream_Item_t *streamItem);
int quicSendStreamClear(QUIC_Send_Stream_Item_t *streamItem);
int quicSendStreamClose(uint16_t connectionId, uint16_t streamId);
/* Transform Information Operations */
int quicTransformInfoAdd(uint16_t connID, uint16_t streamID);
int quicTransformInfoDelete(uint16_t connID, uint16_t streamID);

#endif