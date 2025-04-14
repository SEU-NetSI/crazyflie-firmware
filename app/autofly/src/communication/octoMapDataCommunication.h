#ifndef __OCTOMAP_DATA_COMMUNICATION_H__
#define __OCTOMAP_DATA_COMMUNICATION_H__

#include "stdint.h"
#include "autoflyPacket.h"
#include "autoflyPacket.h"

#define OCTOMAP_DATA_TX_TASK_NAME "OCTOMAP_DATA_TX"
#define OCTOMAP_DATA_TX_TASK_STACK_SIZE 512
#define OCTOMAP_DATA_TX_TASK_PRI 5

#define OCTOMAP_DATA_RX_TASK_NAME "OCTOMAP_DATA_RX"
#define OCTOMAP_DATA_RX_TASK_STACK_SIZE 512
#define OCTOMAP_DATA_RX_TASK_PRI 5

#define MAX_FRAGEMENT_DATA_LENGTH (AUTOFLY_PACKET_MTU-sizeof(octoMapFragmentHeader_t))
#define MAX_SACK_DATA_LENGTH (AUTOFLY_PACKET_MTU-sizeof(octoMapPacket_Sack_Header_t))

#define FIN_PACKET_SIZE sizeof(octoMapPacket_Fin_t)

#define MAX_PACKET_LENGTH MAX_FRAGEMENT_DATA_LENGTH*255

#define STATE_CHECK_INTERVAL 5 // 状态检测间隔

#define FRAGMENT_SEND_INTERVAL 20 // 分片发送间隔

#define TX_ACK_WAIT_TIME_INIT 100 // ACK等待次数初值 
#define MAX_TX_RETRY_TIME 6 // 最大重传次数

#define RX_DATA_UPDATE_WAIT_TIME_INIT 40 // 数据更新等待次数初值
#define MAX_RX_SACK_SEND_TIME 7 // 最大SACK发送次数

#define FIN_SEND_INTERVAL_TIMES 10
#define MAX_FIN_SEND_TIME (5*FIN_SEND_INTERVAL_TIMES)

#define MAX_OCTOMAP_SEND_SLEEP_TIME 1000 // 发送休眠时间
#define MAX_OCTOMAP_SEND_SLEEP_CHECK_INTERVAL 20 // 发送休眠检测间隔

#define MAX_BUFFER_SIZE 4096

typedef enum{
    // 公共状态
    IDLE = 0x0, // 空闲
    FAILED = 0x1, // 失败
    FIN = 0x2, // 完成
    ANY = 0x3, // 任意状态,用于状态强转

    // 发送状态
    DATA_SENDING = 0x41, // 数据发送中
    DATA_SENT = 0x42, // 数据发送完成
    ACK_WAIT = 0x43, // 等待ACK
    ACK_RECEIVED = 0x44, // 收到ACK
    SACK_PROCESS = 0x45, // SACK处理中
    RETRY = 0x46, // 重传
    ACK_TIMEOUT = 0x47, // ACK超时
    ACK_BUSY = 0x48, // 收到ACK表示忙碌
    DATA_SEND_SLEEP = 0x49, // 数据发送休眠

    // 接收状态
    DATA_RECEIVING = 0x51, // 数据接收中
    DATA_WAITING = 0x52, // 等待数据
    DATA_RECEIVED = 0x53, // 数据接收完成
    DATA_TIMEOUT = 0x54, // 数据超时
    SACK_SEND = 0x55, // SACK发送
    FIN_SEND = 0x56, // FIN发送
    BUSY_SEND = 0x57, // 忙碌

}OCTOMAP_DATA_COMMUNICATION_STATE;

typedef enum{
    DISCRETE = 0, // 离散
    CONTINUOUS // 连续
}sack_type_t;

typedef struct
{
    uint16_t dataId; // 数据ID
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
    uint16_t dataId; // 数据ID
    uint8_t length; // 数据长度
    sack_type_t type; // SACK类型,离散或连续
} __attribute__((packed)) octoMapPacket_Sack_Header_t;

typedef struct
{
    octoMapPacket_Sack_Header_t header;
    uint8_t missDataId[MAX_SACK_DATA_LENGTH]; // 丢失数据片ID
} __attribute__((packed)) octoMapPacket_Sack_t;

typedef struct
{
    uint16_t dataId; // 数据ID
}__attribute__((packed)) octoMapPacket_Fin_t;

typedef struct{
    uint16_t dataId; // 数据ID
}octoMapPacket_Error_t;

void octoMapDataCommunicationInit();

bool getTestSendEnableNext();
void setTestSendEnableNext(bool enable);

bool sendOctoMapData(uint8_t destAddress, uint8_t *data, uint16_t length);
bool processOctoMapData(Autofly_packet_t* packet);
#endif