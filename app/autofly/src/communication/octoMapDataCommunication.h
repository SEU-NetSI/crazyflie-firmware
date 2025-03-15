#ifndef __OCTOMAP_DATA_COMMUNICATION_H__
#define __OCTOMAP_DATA_COMMUNICATION_H__

#include "stdint.h"
#include "autoflyPacket.h"
#include "autoflyPacket.h"

#define OCTOMAP_DATA_TX_TASK_NAME "OCTOMAP_DATA_TX"
#define OCTOMAP_DATA_TX_TASK_STACK_SIZE 512
#define OCTOMAP_DATA_TX_TASK_PRI 3

#define OCTOMAP_DATA_RX_TASK_NAME "OCTOMAP_DATA_RX"
#define OCTOMAP_DATA_RX_TASK_STACK_SIZE 512
#define OCTOMAP_DATA_RX_TASK_PRI 3

#define MAX_FRAGEMENT_DATA_LENGTH (AUTOFLY_PACKET_MTU-sizeof(octoMapFragmentHeader_t))
#define MAX_SACK_DATA_LENGTH (AUTOFLY_PACKET_MTU-sizeof(octoMapPacket_Sack_Header_t))

#define MAX_PACKET_LENGTH MAX_FRAGEMENT_DATA_LENGTH*255

#define STATE_CHECK_INTERVAL 5 // 状态检测间隔

#define FRAGMENT_SEND_INTERVAL 20 // 分片发送间隔

#define TX_ACK_WAIT_TIME_INIT 200 // ACK等待次数初值 
#define MAX_TX_RETRY_TIME 3 // 最大重传次数

#define RX_DATA_UPDATE_WAIT_TIME_INIT 200 // 数据更新等待次数初值
#define MAX_RX_SACK_SEND_TIME 3 // 最大SACK发送次数

#define MAX_OCTOMAP_SEND_SLEEP_TIME 1000 // 发送休眠时间
#define MAX_OCTOMAP_SEND_SLEEP_CHECK_INTERVAL 20 // 发送休眠检测间隔

#define MAX_BUFFER_SIZE 4096

typedef enum{
    // 公共状态
    IDLE = 0, // 空闲
    FAILED, // 失败
    FIN, // 完成

    // 发送状态
    DATA_SENDING, // 数据发送中
    DATA_SENT, // 数据发送完成
    ACK_WAIT, // 等待ACK
    ACK_RECEIVED, // 收到ACK
    SACK_PROCESS, // SACK处理中
    RETRY, // 重传
    ACK_TIMEOUT, // ACK超时
    ACK_BUSY, // 收到ACK表示忙碌
    DATA_SEND_SLEEP, // 数据发送休眠

    // 接收状态
    DATA_RECEIVING, // 数据接收中
    DATA_WAITING, // 等待数据
    DATA_RECEIVED, // 数据接收完成
    DATA_TIMEOUT, // 数据超时
    SACK_SEND, // SACK发送
    FIN_SEND, // FIN发送
    BUSY_SEND, // 忙碌
}OCTOMAP_DATA_COMMUNICATION_STATE;

typedef enum{
    DISCRETE = 0, // 离散
    CONTINUOUS // 连续
}sack_type_t;

typedef struct
{
    uint8_t dataId; // 数据ID
    uint8_t fragmentId; // 分片ID
    uint8_t fragmentCount; // 分片总数,最大255，255*53/1024 = 13.1KB
    uint16_t fragmentLength; // 分片长度 不足一片也将按一片发送
} __attribute__((packed)) octoMapFragmentHeader_t;

typedef struct 
{
    octoMapFragmentHeader_t fragmentHeader;
    uint8_t data[MAX_FRAGEMENT_DATA_LENGTH];
} __attribute__((packed)) octoMapFragement_t;

typedef struct 
{
    uint8_t dataId; // 数据ID
    uint8_t length; // 数据长度
    sack_type_t type; // SACK类型,离散或连续
} __attribute__((packed)) octoMapPacket_Sack_Header_t;

typedef struct
{
    octoMapPacket_Sack_Header_t header;
    uint8_t missDataId[MAX_SACK_DATA_LENGTH]; // 丢失数据片ID
} __attribute__((packed)) octoMapPacket_Sack_t;

typedef struct{
    uint8_t dataId; // 数据ID
}octoMapPacket_Error_t;


bool sendOctoMapData(uint16_t destAddress, uint8_t *data, uint16_t length);
bool processOctoMapData(Autofly_packet_t* packet);
#endif