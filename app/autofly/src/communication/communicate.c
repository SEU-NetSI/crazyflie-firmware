#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "radiolink.h"
#include "configblock.h"
#include "debug.h"
#include "cpx_internal_router.h"
#include "cpx_external_router.h"

#include "communicate.h"
#include "control_tool.h"
#include "auxiliary_tool.h"
#include "autoflyPacketQueue.h"

#include "mappingCommunication.h"
#include "exploreCommunication.h"
#include "octoMapDataCommunication.h"

static TaskHandle_t autoflyTxTaskHandle = 0;
static Autofly_packet_Queue_t TxQueue;

void ListeningInit();
void P2PCallbackHandler(P2PPacket *p);
void processAutoflyPacket(Autofly_packet_t* autoflyPacket);
void processPathResp();

void TxTask(void * parameter);

uint8_t getSourceId()
{
    uint64_t address = configblockGetRadioAddress();
    uint8_t sourceId = (uint8_t)((address) & 0x00000000ff);
    return sourceId;
}

void ListeningInit()
{
    // Register the callback function so that the CF can receive packets as well.
    p2pRegisterCB(P2PCallbackHandler);
}

void CommunicateInit(){
    initAutoflyPacketQueue(&TxQueue);
    mappingCommunicationInit();
    exploreCommunicationInit();
    octoMapDataCommunicationInit();
    ListeningInit();
    // 启动发送任务
    xTaskCreate(TxTask, AUTOFLY_TX_TASK_NAME, AUTOFLY_TX_TASK_STACK_SIZE, NULL, AUTOFLY_TX_TASK_PRI, &autoflyTxTaskHandle);
}

void CommunicateTerminate(){
    p2pRegisterCB(NULL);
}

bool sendAutoFlyPacket(uint16_t destAddress, packetType_t packetType, uint8_t *data, uint8_t length){
    Autofly_packet_t Autofly_packet;
    Autofly_packet.header.sourceId = getSourceId();
    Autofly_packet.header.destinationId = destAddress;
    Autofly_packet.header.packetType = packetType;
    Autofly_packet.header.length = AUTOFLY_PACKET_HEAD_LENGTH + length;
    if(length > 0){
        memcpy(Autofly_packet.data, data, length);
    }
    // 将数据包放入发送队列
    pushAutoflyPacketQueue(&TxQueue, &Autofly_packet);
}

bool sendTerminate(){
    sendAutoFlyPacket(AIDECK_ID, TERMINATE, NULL, 0);
}

void P2PCallbackHandler(P2PPacket *p)
{
    // Parse the P2P packet
    uint8_t rssi = p->rssi;
    Autofly_packet_t* autoflyPacket = (Autofly_packet_t*)&p->data;
    // DEBUG_PRINT("P2P: Received packet from(%d) to (%d), packetType: %x\n", autoflyPacket->header.sourceId, autoflyPacket->header.destinationId, autoflyPacket->header.packetType);
    if (autoflyPacket->header.destinationId != getSourceId() && autoflyPacket->header.destinationId != BROADCAST_LIDAR_ID)
    {
        return;
    }

    processAutoflyPacket(autoflyPacket);
}

void processAutoflyPacket(Autofly_packet_t* autoflyPacket){
    switch (autoflyPacket->header.packetType & 0xF0)
    {
        case MAPPING_DATA:{
            processMappingRequest(autoflyPacket);
            break;
        }
        case EXPLORE_DATA:{
            processExploreResp(autoflyPacket);
            break;
        }
        case PATH_DATA:{
            
            break;
        }
        case CLUSTER_DATA:{
            
            break;
        }
        case CONTROL_DATA:{
            
            break;
        }
        case OCTOMAP_DATA:{
            processOctoMapData(autoflyPacket);
            break;
        }
        default:{
            break;
        }
    }
}

void TxTask(void * parameter){
    DEBUG_PRINT("P2P: TxTask start\n");
    Autofly_packet_t autoflyPacket;
    while(1){
        if(popAutoflyPacketQueue(&TxQueue, &autoflyPacket)){
            P2PPacket packet;
            packet.port = 0x00;
            packet.size = autoflyPacket.header.length;
            memcpy(&packet.data, &autoflyPacket, sizeof(autoflyPacket));
            // Send the P2P packet
            if(!radiolinkSendP2PPacketBroadcast(&packet)){
                DEBUG_PRINT("[LiDAR-STM32]P2P: Send packet failed\n");
            }
            else{
                // Autofly_packet_t* testAutoFlyPacket = (Autofly_packet_t*)&packet.data;
                // DEBUG_PRINT("[LiDAR-STM32]P2P: Send packet to: %d, packetType: %x\n", testAutoFlyPacket->header.destinationId, testAutoFlyPacket->header.packetType);
            }
            vTaskDelay(M2T(TX_INTERVAL));
        }
        vTaskDelay(M2T(TX_QUEUE_CHECK_INTERVAL));
    }
}