#ifndef __BASE_PACKET_H__
#define __BASE_PACKET_H__

#include <stdint.h>

#define AUTOFLY_PACKET_HEAD_LENGTH sizeof(Autofly_packet_Header_t)
#define AUTOFLY_PACKET_MTU (60-AUTOFLY_PACKET_HEAD_LENGTH)

typedef enum{
    // mapping
    MAPPING_DATA = 0x10,
    MAPPING_REQ = 0x11, 

    // explore
    EXPLORE_DATA = 0x20,
    EXPLORE_REQ = 0x21, 
    EXPLORE_RESP = 0x22,

    // path
    PATH_DATA = 0x30,
    PATH_REQ = 0x31,    
    PATH_RESP = 0x32,

    // cluster
    CLUSTER_DATA = 0x40,
    CLUSTER_REQ = 0x41,  
    CLUSTER_RESP = 0x42,
    
    // control
    CONTROL_DATA = 0xF0,
    TERMINATE = 0xFF,   // terminate

    // octoMapData
    OCTOMAP_DATA = 0x50,
    OCTOMAP_FIN = 0x51,
    OCTOMAP_DATA_FRAGEMENT = 0x52,
    OCTOMAP_DATA_FRAGEMENT_SACK = 0x53,
    OCTOMAP_ERROR = 0x54,
    OCTOMAP_ERROR_MISS_BUFFER = 0x55, // 缓冲已被清除
    OCTOMAP_ERROR_HAS_PROCESSED = 0x56, // 数据已被处理
    OCTOMAP_ERROR_TX_WAITING_TIMEOUT = 0x57, // 超时
    OCTOMAP_ERROR_RX_WAITING_TIMEOUT = 0x58, // 超时
    OCTOMAP_RECEIVE_BUSY = 0x59, // 忙碌 
}packetType_t;

typedef struct{
    uint8_t sourceId;
    uint8_t destinationId;
    packetType_t packetType;
    uint8_t length;
} __attribute__((packed)) Autofly_packet_Header_t;  

typedef struct
{   
    Autofly_packet_Header_t header;
    uint8_t data[AUTOFLY_PACKET_MTU];
} __attribute__((packed)) Autofly_packet_t;   // 60

#endif