#ifndef __COMMUNICATE_H__
#define __COMMUNICATE_H__
#define DEBUG_MODULE "P2P"
#include "auxiliary_tool.h"
#include "autoflyPacket.h"

#define AUTOFLY_TX_TASK_NAME "AUTOFLY_TX"
#define AUTOFLY_TX_TASK_STACK_SIZE 512
#define AUTOFLY_TX_TASK_PRI 3

#define AUTOFLY_RX_TASK_NAME "AUTOFLY_RX"
#define AUTOFLY_RX_TASK_STACK_SIZE 512
#define AUTOFLY_RX_TASK_PRI 3

#define BROADCAST_LIDAR_ID 0xFF
#define AIDECK_ID 0xFF

#define TX_QUEUE_CHECK_INTERVAL 2
#define TX_INTERVAL 2

uint8_t getSourceId();
bool getTestSendEnableNext();
void CommunicateInit();
void CommunicateTerminate();

bool sendAutoFlyPacket(uint8_t destAddress, packetType_t packetType, uint8_t *data, uint8_t length);

bool sendTerminate();
#endif
