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
static TaskHandle_t autoflyRxTaskHandle = 0;
// static Autofly_packet_Queue_t TxQueue;

static QueueHandle_t txQueue;
static QueueHandle_t rxQueue;

void ListeningInit();
void P2PCallbackHandler(P2PPacket *p);
void processAutoflyPacket(Autofly_packet_t* autoflyPacket);
void processPathResp();

void TxTask(void * parameter);
void RxTask(void * parameter);

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
    // initAutoflyPacketQueue(&TxQueue);
    txQueue = xQueueCreate(AUTOFLY_PACKET_QUEUE_SIZE, AUTOFLY_PACKET_QUEUE_ITEM_SIZE);
    rxQueue = xQueueCreate(AUTOFLY_PACKET_QUEUE_SIZE, AUTOFLY_PACKET_QUEUE_ITEM_SIZE);
    if (txQueue == NULL || rxQueue == NULL)
    {
        DEBUG_PRINT("P2P: Create queue failed\n");
        return;
    }
    // 初始化通信
    mappingCommunicationInit();
    exploreCommunicationInit();
    octoMapDataCommunicationInit();
    ListeningInit();
    // 启动任务
    xTaskCreate(RxTask, AUTOFLY_RX_TASK_NAME, AUTOFLY_RX_TASK_STACK_SIZE, NULL, AUTOFLY_RX_TASK_PRI, &autoflyRxTaskHandle);
    xTaskCreate(TxTask, AUTOFLY_TX_TASK_NAME, AUTOFLY_TX_TASK_STACK_SIZE, NULL, AUTOFLY_TX_TASK_PRI, &autoflyTxTaskHandle);
}

void CommunicateTerminate(){
    p2pRegisterCB(NULL);
}

bool sendAutoFlyPacket(uint8_t destAddress, packetType_t packetType, uint8_t *data, uint8_t length){

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    Autofly_packet_t Autofly_packet;

    Autofly_packet.header.sourceId = getSourceId();
    Autofly_packet.header.destinationId = destAddress;
    Autofly_packet.header.packetType = packetType;
    Autofly_packet.header.length = AUTOFLY_PACKET_HEAD_LENGTH + length;
    if(length > 0){
        memcpy(Autofly_packet.data, data, length);
    }
    // DEBUG_PRINT("[sendAutoFlyPacket]: destAdr: = (%d,%d), packetType: (%x,%x)\n", destAddress,Autofly_packet.header.destinationId, packetType,Autofly_packet.header.packetType);
    // 将数据包放入发送队列
    // pushAutoflyPacketQueue(&TxQueue, &Autofly_packet);
    if (xQueueSendFromISR(txQueue, &Autofly_packet, &xHigherPriorityTaskWoken) != pdPASS) {
        DEBUG_PRINT("[sendAutoFlyPacket]: Send packet failed\n");
        return false;
    }
    return true;
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

    xQueueSendFromISR(rxQueue, autoflyPacket, NULL);
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
        if (xQueueReceive(txQueue, &autoflyPacket, portMAX_DELAY)){
            // if(autoflyPacket.header.packetType == OCTOMAP_DATA_FRAGEMENT){
            //     octoMapFragement_t* fragment = (octoMapFragement_t*)autoflyPacket.data;
            //     DEBUG_PRINT("[TxTask OCTOMAP_DATA_FRAGEMENT]destAdr = %x, fragmentId = %d, dataId = %d, fragmentCount = %d\n", autoflyPacket.header.destinationId, fragment->fragmentHeader.fragmentId, fragment->fragmentHeader.dataId, fragment->fragmentHeader.fragmentCount);
            // }
            // if(autoflyPacket.header.packetType == OCTOMAP_FIN){
            //     octoMapPacket_Fin_t* fin = (octoMapPacket_Fin_t*)autoflyPacket.data;
            //     DEBUG_PRINT("[TxTask OCTOMAP_FIN]destAdr = %x,dataId = %d\n", autoflyPacket.header.destinationId,fin->dataId);
            // }
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
        }
        vTaskDelay(M2T(TX_INTERVAL));
    }
}

void RxTask(void * parameter){
    DEBUG_PRINT("P2P: RxTask start\n");
    Autofly_packet_t autoflyPacket;
    while(1){
        if (xQueueReceive(rxQueue, &autoflyPacket, portMAX_DELAY)){
            // Process the received packet
            processAutoflyPacket(&autoflyPacket);
        }
        vTaskDelay(M2T(TX_INTERVAL));
    }
}