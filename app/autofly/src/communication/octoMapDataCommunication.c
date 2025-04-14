#include <string.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "semphr.h"

#include "octoMapDataCommunication.h"
#include "autoflyPacket.h"
#include "communicate.h"
#include "octoMapSerializer.h"

#include "radiolink.h"

static bool testSendEnableNext = false;

static TaskHandle_t octoMapDataCommunicationTxTaskHandle;
static TaskHandle_t octoMapDataCommunicationRxTaskHandle;

static OCTOMAP_DATA_COMMUNICATION_STATE TX_STATE; // 发送状态
static SemaphoreHandle_t muStateTx;
static SemaphoreHandle_t muDataTX;
static uint8_t dataTXBuffer[MAX_BUFFER_SIZE];
static uint16_t dataTXLength;
static uint16_t dataTXId; // 数据ID,检测数据是否被更新
static uint8_t destinationTxId; // 目标地址
static bool isSackReceived; // 是否收到SACK
static Autofly_packet_t lastReceiveSackPacket; // 上一个SACK包
static uint16_t lastTxErrorId = -1; // 上一个发送的错误数据ID
static uint16_t lastFinId = -1; // 上一个FIN数据ID

static OCTOMAP_DATA_COMMUNICATION_STATE RX_STATE; // 接收状态
static SemaphoreHandle_t muStateRx;
static SemaphoreHandle_t muDataRX;
static uint8_t dataRXBuffer[MAX_BUFFER_SIZE];
static uint16_t dataRXLength;
static uint8_t sourceRxId; // 源地址
static uint16_t dataRXId; // 数据ID,检测数据是否被更新
static uint8_t checkFLag[32]; // 检测某位片数据是否被接收
static uint8_t totalCountNum; // 总数据数量
static uint8_t curCountNum; // 当前接收的数据数量
static uint16_t lastRxErrorId = -1; // 上一个接收的错误数据ID
static bool curDataComplete = false;
static bool curDataFailed = false;

static uint16_t TxAckTimeoutTimes; // 发送响应超时时间
static uint16_t maxTxAckWaitTimes; // ACK最大超时时间
static uint8_t retryTimes; // 重传次数

static uint16_t RxDataUpdateTimeoutTimes; // 接收数据更新超时次数
static uint16_t maxRxDataUpdateWaitTimes; // 数据更新最大等待次数
static uint8_t RxSackSendTimes; // SACK发送次数
static octoMapPacket_Sack_t lastSendSackPacket; // 上一个SACK包
static uint8_t finSendTimes; // FIN发送次数

static uint16_t sendFragmentCount = 0;

// extern TaskHandle_t testTaskHandle;

void octoMapDataCommunicationRxTask(void * parameter);
void octoMapDataCommunicationTxTask(void * parameter);
bool processOctoMapFragMent(Autofly_packet_t *packet);
bool processOctoMapPacket_Sack(Autofly_packet_t *packet);

void octoMapDataCommunicationInit(){
    muDataTX = xSemaphoreCreateMutex();
    muStateTx = xSemaphoreCreateMutex();
    muDataRX = xSemaphoreCreateMutex();
    muStateRx = xSemaphoreCreateMutex();
    octoMapDataCommunicationRxInit();
    octoMapDataCommunicationTxInit();
    
    xTaskCreate(octoMapDataCommunicationRxTask, OCTOMAP_DATA_RX_TASK_NAME, OCTOMAP_DATA_RX_TASK_STACK_SIZE, NULL, OCTOMAP_DATA_RX_TASK_PRI, &octoMapDataCommunicationRxTaskHandle);
    xTaskCreate(octoMapDataCommunicationTxTask, OCTOMAP_DATA_TX_TASK_NAME, OCTOMAP_DATA_TX_TASK_STACK_SIZE, NULL, OCTOMAP_DATA_TX_TASK_PRI, &octoMapDataCommunicationTxTaskHandle);

    testSendEnableNext = true;
}

bool setTxState(OCTOMAP_DATA_COMMUNICATION_STATE state,OCTOMAP_DATA_COMMUNICATION_STATE pre){
    OCTOMAP_DATA_COMMUNICATION_STATE cur = TX_STATE;
    if(TX_STATE == pre || pre == ANY){
        xSemaphoreTake(muStateTx, portMAX_DELAY);
        TX_STATE = state;
        // DEBUG_PRINT("[setTxState]pre = %x, TX_STATE = %x\n", cur, TX_STATE);
        xSemaphoreGive(muStateTx);
        return true;
    }else{
        DEBUG_PRINT("[setTxState failed]state = %x, pre = %x, TX_STATE = %x\n", state, pre, cur);
        return false;
    }
}

bool setRxState(OCTOMAP_DATA_COMMUNICATION_STATE state,OCTOMAP_DATA_COMMUNICATION_STATE pre){
    OCTOMAP_DATA_COMMUNICATION_STATE cur = TX_STATE;
    if(RX_STATE == pre || pre == ANY){
        xSemaphoreTake(muStateRx, portMAX_DELAY);
        RX_STATE = state;
        // DEBUG_PRINT("[setRxState]pre = %x, RX_STATE = %x\n", cur, RX_STATE);
        xSemaphoreGive(muStateRx);
        return true;
    }else{
        DEBUG_PRINT("[setRxState failed]state = %x, pre = %x, RX_STATE = %x\n", state, pre, cur);
        return false;
    }
}

bool getTestSendEnableNext(){
    return testSendEnableNext;
}

void setTestSendEnableNext(bool enable){
    testSendEnableNext = enable;
}

void octoMapDataCommunicationRxInit(){
    setRxState(IDLE,ANY);
    dataRXLength = 0;
    dataRXId = 0;
    memset(checkFLag, 0, 32);
    curCountNum = 0;
    finSendTimes = 0;

    RxDataUpdateTimeoutTimes = 0;
    maxRxDataUpdateWaitTimes = RX_DATA_UPDATE_WAIT_TIME_INIT;
    RxSackSendTimes = 0;
}

void octoMapDataCommunicationTxInit(){
    setTxState(IDLE,ANY);
    dataTXLength = 0;
    dataTXId = 0;

    TxAckTimeoutTimes = 0;
    maxTxAckWaitTimes = TX_ACK_WAIT_TIME_INIT;
    retryTimes = 0;
}


bool sendOctoMapData(uint8_t destAddress, uint8_t *data, uint16_t length){
    // 进入发送状态
    xSemaphoreTake(muDataTX, portMAX_DELAY);
    setTxState(DATA_SENDING,ANY);
    // DEBUG_PRINT("[sendOctoMapData]data length = %d\n", length);
    // 初始化响应超时时间、最大超时时间和重传次数
    TxAckTimeoutTimes = 0;
    maxTxAckWaitTimes = TX_ACK_WAIT_TIME_INIT;
    retryTimes = 0;
    // 初始化SACK接收状态
    isSackReceived = false;
    // 更新数据ID和目标地址
    dataTXId++;
    destinationTxId = destAddress;
    if(length > MAX_BUFFER_SIZE){
        DEBUG_PRINT("[sendOctoMapData]data is too long, dataTXBuffer maxLength = %d, data length = %d\n", MAX_BUFFER_SIZE, length);
        setTxState(FAILED,ANY);
        // 释放信号量
        xSemaphoreGive(muDataTX);
        return false;
    }
    if(length >= MAX_PACKET_LENGTH){
        // 数据过长
        DEBUG_PRINT("[sendOctoMapData]data is too long, maxPacketLength = %d, data length = %d\n", MAX_PACKET_LENGTH, length);
        setTxState(FAILED,ANY);
        xSemaphoreGive(muDataTX);
        return false;
    }
    else{
        memcpy(dataTXBuffer, data, length);
        dataTXLength = length;
        // 分片发送，不足一片也按一片发送
        // 计算分片数量,向上取整
        uint16_t fragmentCount = ((dataTXLength + MAX_FRAGEMENT_DATA_LENGTH - 1) / MAX_FRAGEMENT_DATA_LENGTH);
        DEBUG_PRINT("[sendOctoMapData]dataTXId = %d, dataTXLength = %d, fragmentCount = %d, sigFragmentlength = %d\n", dataTXId, dataTXLength, fragmentCount, MAX_FRAGEMENT_DATA_LENGTH);
        octoMapFragement_t fragment;
        for (int i = 0; i < fragmentCount; i++)
        {
            fragment.fragmentHeader.dataId = dataTXId;
            fragment.fragmentHeader.fragmentId = i;
            fragment.fragmentHeader.fragmentCount = fragmentCount;
            if(i == fragmentCount - 1){
                fragment.fragmentHeader.fragmentLength = dataTXLength - i*MAX_FRAGEMENT_DATA_LENGTH;
            }
            else{
                fragment.fragmentHeader.fragmentLength = MAX_FRAGEMENT_DATA_LENGTH;
            }
            memcpy(fragment.data, dataTXBuffer+i*MAX_FRAGEMENT_DATA_LENGTH, fragment.fragmentHeader.fragmentLength);
            sendFragmentCount ++;
            sendAutoFlyPacket(destAddress, OCTOMAP_DATA_FRAGEMENT, (uint8_t*)&fragment, fragment.fragmentHeader.fragmentLength + sizeof(octoMapFragmentHeader_t));
            vTaskDelay(M2T(FRAGMENT_SEND_INTERVAL));
        }
    }
    // 数据发送完成
    setTxState(DATA_SENT,DATA_SENDING);
    xSemaphoreGive(muDataTX);
}

bool sendOctomapSack(){
    xSemaphoreTake(muDataRX, portMAX_DELAY);
    RxSackSendTimes++;
    if(RxSackSendTimes > 1){
        // RxSackSendTimes > 1 说明在此期间数据未被更新，sack无需重新生成
        DEBUG_PRINT("[sendOctomapSack]RxSackSendTimes = %d, sack is not need to be generated\n", RxSackSendTimes);
        sendAutoFlyPacket(sourceRxId, OCTOMAP_DATA_FRAGEMENT_SACK, (uint8_t*)&lastSendSackPacket, lastSendSackPacket.header.length + sizeof(octoMapPacket_Sack_Header_t));
        setRxState(DATA_WAITING,SACK_SEND);
        xSemaphoreGive(muDataRX);
        return true;
    }
    octoMapPacket_Sack_t sack;
    sack.header.dataId = dataRXId;
    // 统计缺失数据
    uint8_t missCountDiscrete = 0;
    uint8_t missCountContinuous = 0;
    bool iscontinuous = false;
    for (int i = 0; i < totalCountNum; i++)
    {
        uint8_t index = i / 8;
        uint8_t offset = i % 8;
        if(!(checkFLag[index] & (1 << offset))){
            missCountDiscrete++;
            if(!iscontinuous){
                iscontinuous = true;
                missCountContinuous++;
            }
        }else{
            if(iscontinuous){
                missCountContinuous++;
                iscontinuous = false;
            }
        }
    }
    if(iscontinuous){
        missCountContinuous++;
    }
    if(missCountDiscrete <= missCountContinuous){
        // 缺失数据较离散
        if(missCountDiscrete > MAX_SACK_DATA_LENGTH){
            // 缺失数据过多
            DEBUG_PRINT("[sendOctomapSack]miss data is too much, missCountDiscrete = %d\n", missCountDiscrete);
            setRxState(FAILED,ANY);
            xSemaphoreGive(muDataRX);
            return false;
        }
        sack.header.type = DISCRETE;
        sack.header.length = missCountDiscrete;
        int top = 0;
        for (int i = 0; i < totalCountNum; i++)
        {
            uint8_t index = i / 8;
            uint8_t offset = i % 8;
            if(!(checkFLag[index] & (1 << offset))){
                sack.missDataId[top++] = i;
            }
        }
        if(top != sack.header.length){
            DEBUG_PRINT("[sendOctomapSack]miss data length is not equal to sack length, missCountDiscrete = %d, sack length = %d\n", missCountDiscrete, sack.header.length);
        }
        for(int i = 0; i< top; i++){
            DEBUG_PRINT("[sendOctomapSack]missDataId[%d] = %d\n", i, sack.missDataId[i]);
        }
    }else{
        // 缺失数据较连续
        if(missCountContinuous > MAX_SACK_DATA_LENGTH){
            // 缺失数据过多
            DEBUG_PRINT("[sendOctomapSack]miss data is too much, missCountContinuous = %d\n", missCountContinuous);
            setRxState(FAILED,ANY);
            xSemaphoreGive(muDataRX);
            return false;
        }
        sack.header.type = CONTINUOUS;
        sack.header.length = missCountContinuous;
        iscontinuous = false;
        int i = 0;
        int top = 0;
        for (i = 0; i < totalCountNum; i++)
        {
            uint8_t index = i / 8;
            uint8_t offset = i % 8;
            if(!(checkFLag[index] & (1 << offset))){
                if(!iscontinuous){
                    iscontinuous = true;
                    sack.missDataId[top++] = i;
                }
            }else{
                if(iscontinuous){
                    iscontinuous = false;
                    sack.missDataId[top++] = i-1;
                }
            }
        }
        if(iscontinuous){
            iscontinuous = false;
            sack.missDataId[top++] = i-1;
        }
        if(top != sack.header.length){
            DEBUG_PRINT("[sendOctomapSack]miss data length is not equal to sack length, missCountContinuous = %d, sack length = %d\n", missCountContinuous, sack.header.length);
        }
        for(int i = 0; i< top; i+=2){
            DEBUG_PRINT("[sendOctomapSack]missData startID = %d, endID = %d\n", sack.missDataId[i], sack.missDataId[i+1]);
        }
    }
    DEBUG_PRINT("[sendOctomapSack]sack type = %d, dataID = %d, sack length = %d\n", sack.header.type, sack.header.dataId, sack.header.length);
    sendAutoFlyPacket(sourceRxId, OCTOMAP_DATA_FRAGEMENT_SACK, (uint8_t*)&sack, sack.header.length + sizeof(octoMapPacket_Sack_Header_t));
    memcpy(&lastSendSackPacket, &sack, sizeof(octoMapPacket_Sack_t));
    setRxState(DATA_WAITING,SACK_SEND);
    xSemaphoreGive(muDataRX);
    return true;
}

bool processOctoMapData(Autofly_packet_t* packet){
    // DEBUG_PRINT("[processOctoMapData]packetType = %d, sourceId = %d, destId = %d\n", packet->header.packetType, packet->header.sourceId, packet->header.destinationId);
    octoMapPacket_Error_t *error = NULL;
    switch (packet->header.packetType)
    {
    case OCTOMAP_DATA_FRAGEMENT:
        return processOctoMapFragMent(packet);
    case OCTOMAP_DATA_FRAGEMENT_SACK:
        return processOctoMapPacket_Sack(packet);
    case OCTOMAP_FIN:
        // 数据接收完成
        DEBUG_PRINT("[processOctoMapData] receive FIN\n");
        octoMapPacket_Fin_t *fin = (octoMapPacket_Fin_t*)packet->data;
        xSemaphoreTake(muDataTX, portMAX_DELAY);
        if(fin->dataId == dataTXId && packet->header.sourceId == destinationTxId && fin->dataId != lastFinId){
            setTxState(FIN,ANY);
            lastFinId = fin->dataId;    
            DEBUG_PRINT("[processOctoMapData OCTOMAP_FIN] dataTxId = %d, finId = %d\n", dataTXId,fin->dataId);
            // 发送FIN_ACK
            octoMapPacket_Fin_t finAck;
            finAck.dataId = dataTXId;
            sendAutoFlyPacket(destinationTxId, OCTOMAP_FIN_ACK, (uint8_t*)&finAck, sizeof(octoMapPacket_Fin_t));
        }else{
            if(packet->header.sourceId != destinationTxId){
                // 不是当前源设备发送的数据
                DEBUG_PRINT("[processOctoMapData]receive fin, but sourceId is not equal to destinationTxId, sourceId = %d, destinationTxId = %d\n", packet->header.sourceId, destinationTxId);
            }else if(fin->dataId != dataTXId){
                // 不是当前数据的FIN
                DEBUG_PRINT("[processOctoMapData]receive fin, but dataId is not equal to dataTXId, dataTXId = %d, finId = %d\n", dataTXId, fin->dataId);
            }else{
                // 重复的FIN
                octoMapPacket_Fin_t finAck;
                finAck.dataId = dataTXId;
                sendAutoFlyPacket(destinationTxId, OCTOMAP_FIN_ACK, (uint8_t*)&finAck, sizeof(octoMapPacket_Fin_t));
            }
        }
        xSemaphoreGive(muDataTX);
        break;
    case OCTOMAP_ERROR_MISS_BUFFER:
        // 发送方已清除error->dataId数据
        DEBUG_PRINT("[processOctoMapData]receive error OCTOMAP_ERROR_MISS_BUFFER\n");
        error = (octoMapPacket_Error_t*)packet->data;
        if(error->dataId == dataRXId && error->dataId != lastRxErrorId){
            setRxState(FAILED,ANY);    
            // 避免短期多次错误报文重复处理
            lastRxErrorId = error->dataId;
        }
        break;
    case OCTOMAP_ERROR_HAS_PROCESSED:
        // error->dataId数据对方已处理过
        DEBUG_PRINT("[processOctoMapData]receive error OCTOMAP_ERROR_HAS_PROCESSED\n");
        error = (octoMapPacket_Error_t*)packet->data;
        if(error->dataId == dataTXId && error->dataId != lastTxErrorId){
            setTxState(FAILED,ANY);
            // 避免短期多次错误报文重复处理
            lastTxErrorId = error->dataId;
        }
        break;
    case OCTOMAP_ERROR_TX_WAITING_TIMEOUT:
        // 发送方对error->dataId数据等待超时
        DEBUG_PRINT("[processOctoMapData]receive error OCTOMAP_ERROR_TX_WAITING_TIMEOUT\n");
        error = (octoMapPacket_Error_t*)packet->data;
        if(error->dataId == dataRXId && error->dataId != lastRxErrorId){
            setRxState(FAILED,ANY);    
            // 避免短期多次错误报文重复处理
            lastRxErrorId = error->dataId;
        }
        break;
    case OCTOMAP_ERROR_RX_WAITING_TIMEOUT:
        // 接收方对error->dataId数据等待超时
        DEBUG_PRINT("[processOctoMapData]receive error OCTOMAP_ERROR_RX_WAITING_TIMEOUT\n");
        error = (octoMapPacket_Error_t*)packet->data;
        if(error->dataId == dataTXId && error->dataId != lastTxErrorId){
            setTxState(FAILED,ANY);
            // 避免短期多次错误报文重复处理
            lastTxErrorId = error->dataId;
        }
        break;
    case OCTOMAP_RECEIVE_BUSY:
        // 忙碌
        setTxState(ACK_BUSY,ANY);
        break;
    case OCTOMAP_FIN_ACK:
        // FIN_ACK
        DEBUG_PRINT("[processOctoMapData]receive FIN_ACK\n");
        octoMapPacket_Fin_t *finAck = (octoMapPacket_Fin_t*)packet->data;
        xSemaphoreTake(muDataRX, portMAX_DELAY);
        if(finAck->dataId == dataRXId && packet->header.sourceId == sourceRxId){
            setTxState(IDLE,ANY);
            DEBUG_PRINT("[processOctoMapData]receive FIN_ACK, dataId = %d\n", finAck->dataId);
        }
        xSemaphoreGive(muDataRX);
        break;
    default:
        break;
    }
}


bool processOctoMapFragMent(Autofly_packet_t *packet){
    if(RX_STATE != IDLE && packet->header.sourceId != sourceRxId){
        // 不是当前源设备发送的数据
        sendAutoFlyPacket(packet->header.sourceId, OCTOMAP_RECEIVE_BUSY, NULL, 0);
        DEBUG_PRINT("[processOctoMapData]receive data, but sourceId is not equal to sourceRxId, sourceId = %d, sourceRxId = %d\n", packet->header.sourceId, sourceRxId);
        return false;
    }
    xSemaphoreTake(muDataRX, portMAX_DELAY);
    octoMapFragement_t *fragment = (octoMapFragement_t*)packet->data;
    bool res = false;
    if(packet->header.sourceId == sourceRxId && fragment->fragmentHeader.dataId < dataRXId){
        // 同一个源设备发送的数据，数据id小于当前数据id，代表已经是历史数据
        octoMapPacket_Error_t error;
        error.dataId = fragment->fragmentHeader.dataId;
        sendAutoFlyPacket(packet->header.sourceId, OCTOMAP_ERROR_HAS_PROCESSED, (uint8_t*)&error, sizeof(octoMapPacket_Error_t));
        DEBUG_PRINT("[processOctoMapData]receive data, but dataId is less than dataRXId, dataId = %d, dataRXId = %d\n", fragment->fragmentHeader.dataId, dataRXId);
        xSemaphoreGive(muDataRX);
        return false;
    }
    // 接收数据
    setRxState(DATA_RECEIVING,ANY);
    DEBUG_PRINT("[processOctoMapData]dataId = %d, fragmentId = %d, maxFragementId = %d\n", fragment->fragmentHeader.dataId, fragment->fragmentHeader.fragmentId, fragment->fragmentHeader.fragmentCount); 
    if(RX_STATE == IDLE | (fragment->fragmentHeader.dataId > dataRXId)){
        // 新数据,接收到新数据代表旧数据已被抛弃
        dataRXId = fragment->fragmentHeader.dataId;
        dataRXLength = 0;
        curCountNum = 0;
        memset(checkFLag, 0, 32);
        sourceRxId = packet->header.sourceId;
        totalCountNum = fragment->fragmentHeader.fragmentCount;
        finSendTimes = 0;
        curDataComplete = false;
        curDataFailed = false;

        RxDataUpdateTimeoutTimes = 0;
        if(maxRxDataUpdateWaitTimes > RX_DATA_UPDATE_WAIT_TIME_INIT){
            maxRxDataUpdateWaitTimes = maxRxDataUpdateWaitTimes/2;
        }
        RxSackSendTimes = 0;
        DEBUG_PRINT("[processOctoMapData]new data, dataId = %d, fragmentId = %d, fragmentCount = %d\n", fragment->fragmentHeader.dataId, fragment->fragmentHeader.fragmentId, fragment->fragmentHeader.fragmentCount);
    }
    // 数据已接收完成或失败
    if(curDataComplete || curDataFailed){
        // 直接丢弃
        DEBUG_PRINT("[processOctoMapData]data has been received, dataId = %d\n", fragment->fragmentHeader.dataId);
        // 发送FIN报文
        if(curDataComplete){
            octoMapPacket_Fin_t fin;
            fin.dataId = fragment->fragmentHeader.dataId;
            DEBUG_PRINT("[processOctoMapData]send FIN, dataId = %d\n", fin.dataId);
            sendAutoFlyPacket(packet->header.sourceId, OCTOMAP_FIN, (uint8_t*)&fin, sizeof(octoMapPacket_Fin_t));
        }else{
            octoMapPacket_Error_t error;
            error.dataId = fragment->fragmentHeader.dataId;
            DEBUG_PRINT("[processOctoMapData]send error, dataId = %d\n", error.dataId);
            sendAutoFlyPacket(packet->header.sourceId, OCTOMAP_ERROR_RX_WAITING_TIMEOUT, (uint8_t*)&error, sizeof(octoMapPacket_Error_t));
        }
        setTxState(IDLE,ANY);
        xSemaphoreGive(muDataRX);
        return false;
    }
    // 判断是否重复
    uint8_t index = fragment->fragmentHeader.fragmentId / 8;
    uint8_t offset = fragment->fragmentHeader.fragmentId % 8;
    if(checkFLag[index] & (1 << offset)){
        // 重复数据
        DEBUG_PRINT("[processOctoMapData]repeat data, dataId = %d, fragmentId = %d\n", fragment->fragmentHeader.dataId, fragment->fragmentHeader.fragmentId);
        setRxState(DATA_WAITING,DATA_RECEIVING);
    }
    else{
        // 未重复
        memcpy(dataRXBuffer + fragment->fragmentHeader.fragmentId * MAX_FRAGEMENT_DATA_LENGTH, fragment->data, fragment->fragmentHeader.fragmentLength);
        checkFLag[index] |= (1 << offset);
        curCountNum++;
        dataRXLength += fragment->fragmentHeader.fragmentLength;
        // 接收到新数据清空接收数据更新超时时间、最大超时时间和SACK发送次数
        RxDataUpdateTimeoutTimes = 0;
        if(maxRxDataUpdateWaitTimes > RX_DATA_UPDATE_WAIT_TIME_INIT){
            maxRxDataUpdateWaitTimes = maxRxDataUpdateWaitTimes/2;
        }
        RxSackSendTimes = 0;
        if(curCountNum == totalCountNum){
            // 数据接收完
            // todo: 处理数据
            // octoMapSerializerResult_t res;
            // initOctoMapSerializerResult(&res);
            // memcpy(&res, dataRXBuffer, dataRXLength);
            // uint8_t checkCode = 0;
            // for (int i = 0; i < res.dataLength; i++)
            // {
            //     checkCode = checkCode ^ res.data[i];
            // }
            // if(checkCode != res.checkCode){
            //     // 数据校验失败
            //     DEBUG_PRINT("[processOctoMapData]data check failed, dataId = %d, checkCode = %d\n", dataRXId, checkCode);
            // }else{
            DEBUG_PRINT("[processOctoMapData]data received, dataId = %d, dataLength = %d\n", dataRXId, dataRXLength);
            // }
            // 数据接收完
            curDataComplete = true;
            setRxState(DATA_RECEIVED,DATA_RECEIVING);
        }
        else{
            setRxState(DATA_WAITING,DATA_RECEIVING);
        }
    }
    xSemaphoreGive(muDataRX);
    return true;
}

bool processOctoMapPacket_Sack(Autofly_packet_t *packet){
    // 处理SACK
    octoMapPacket_Sack_t *sack = (octoMapPacket_Sack_t*)packet->data;
    DEBUG_PRINT("[processOctoMapPacket_Sack]destAdr = %x, sourceId = %x, dataId = %d\n", packet->header.destinationId, packet->header.sourceId, sack->header.dataId);
    xSemaphoreTake(muDataTX, portMAX_DELAY);
    // 判断数据是否过期
    if(sack->header.dataId < dataTXId){
        // 数据已过期
        octoMapPacket_Error_t error;
        error.dataId = sack->header.dataId;
        sendAutoFlyPacket(packet->header.sourceId, OCTOMAP_ERROR_MISS_BUFFER, (uint8_t*)&error, sizeof(octoMapPacket_Error_t));
        DEBUG_PRINT("[processOctoMapPacket_Sack]dataId is out of range, dataId = %d, dataTXId = %d\n", sack->header.dataId, dataTXId);
        xSemaphoreGive(muDataTX);
        return false;
    }else if(sack->header.dataId > dataTXId){
        // 数据不存在
        octoMapPacket_Error_t error;
        error.dataId = sack->header.dataId;
        sendAutoFlyPacket(packet->header.sourceId, OCTOMAP_ERROR, (uint8_t*)&error, sizeof(octoMapPacket_Error_t));
        DEBUG_PRINT("[processOctoMapPacket_Sack]dataId is out of range, dataId = %d, dataTXId = %d\n", sack->header.dataId, dataTXId);
        xSemaphoreGive(muDataTX);
        return false;
    }
    else{
        // 收到有效SACK，转入SACK处理状态
        setTxState(SACK_PROCESS,ANY);
        // 清空ACK超时次数,最大超时时间降档，重传次数清零
        TxAckTimeoutTimes = 0;
        if(maxTxAckWaitTimes > TX_ACK_WAIT_TIME_INIT){
            maxTxAckWaitTimes = maxTxAckWaitTimes/2;
        }
        retryTimes = 0;
        // 更新SACK记录
        isSackReceived = true;
        memcpy(&lastReceiveSackPacket, packet, sizeof(Autofly_packet_t));
        // 缺失数据重传，转入数据发送状态
        if(!setTxState(DATA_SENDING,SACK_PROCESS)){
            // 设置状态失败，说明当前状态不是SACK_PROCESS
            DEBUG_PRINT("[processOctoMapPacket_Sack]setTxState failed, TX_STATE = %x\n", TX_STATE);
            xSemaphoreGive(muDataTX);
            return false;
        }
        uint8_t fragmentCount = ((dataTXLength + MAX_FRAGEMENT_DATA_LENGTH - 1) / MAX_FRAGEMENT_DATA_LENGTH);
        DEBUG_PRINT("[processOctoMapPacket_Sack]dataTXId = %d, dataTXLength = %d, fragmentCount = %d, sigFragmentlength = %d\n", dataTXId, dataTXLength, fragmentCount, MAX_FRAGEMENT_DATA_LENGTH);
        octoMapFragement_t fragment;
        if(sack->header.type == DISCRETE){
            for (int i = 0; i < sack->header.length; i++)
            {
                // 重发丢失数据
                uint8_t fragmentId = sack->missDataId[i];
                DEBUG_PRINT("[processOctoMapPacket_Sack]fragmentId = %d\n", fragmentId);
                if(fragmentId >= fragmentCount){
                    // 分片ID越界
                    DEBUG_PRINT("[processOctoMapPacket_Sack]fragmentId is out of range, fragmentId = %d, fragmentCount = %d\n", fragmentId, fragmentCount);
                    continue;
                }
                fragment.fragmentHeader.dataId = dataTXId;
                fragment.fragmentHeader.fragmentId = fragmentId;
                fragment.fragmentHeader.fragmentCount = fragmentCount;
                if(fragmentId == fragmentCount - 1){
                    fragment.fragmentHeader.fragmentLength = dataTXLength - fragmentId*MAX_FRAGEMENT_DATA_LENGTH;
                }
                else{
                    fragment.fragmentHeader.fragmentLength = MAX_FRAGEMENT_DATA_LENGTH;
                }
                memcpy(fragment.data, dataTXBuffer+fragmentId*MAX_FRAGEMENT_DATA_LENGTH, fragment.fragmentHeader.fragmentLength);
                sendAutoFlyPacket(packet->header.sourceId, OCTOMAP_DATA_FRAGEMENT, (uint8_t*)&fragment, fragment.fragmentHeader.fragmentLength + sizeof(octoMapFragmentHeader_t));
                sendFragmentCount ++;
                vTaskDelay(M2T(FRAGMENT_SEND_INTERVAL));
            }
        }else{
            // 连续数据丢失
            // 读取两个值
            for (int i = 0; i < sack->header.length; i+=2)
            {
                uint8_t start = sack->missDataId[i];
                uint8_t end = sack->missDataId[i+1];
                if(start >= fragmentCount || end >= fragmentCount){
                    // 分片ID越界
                    DEBUG_PRINT("[processOctoMapPacket_Sack]fragmentId is out of range, start = %d, end = %d, fragmentCount = %d\n", start, end, fragmentCount);
                    continue;
                }
                if(start > end){
                    // 起始ID大于结束ID
                    DEBUG_PRINT("[processOctoMapPacket_Sack]start is greater than end, start = %d, end = %d\n", start, end);
                    continue;
                }
                DEBUG_PRINT("[processOctoMapPacket_Sack]start = %d, end = %d\n", start, end);
                for (int j = start; j <= end; j++)
                {
                    // 重发丢失数据
                    fragment.fragmentHeader.dataId = dataTXId;
                    fragment.fragmentHeader.fragmentId = j;
                    fragment.fragmentHeader.fragmentCount = fragmentCount;
                    if(j == fragmentCount - 1){
                        fragment.fragmentHeader.fragmentLength = dataTXLength - j*MAX_FRAGEMENT_DATA_LENGTH;
                    }
                    else{
                        fragment.fragmentHeader.fragmentLength = MAX_FRAGEMENT_DATA_LENGTH;
                    }
                    memcpy(fragment.data, dataTXBuffer+j*MAX_FRAGEMENT_DATA_LENGTH, fragment.fragmentHeader.fragmentLength);
                    sendAutoFlyPacket(packet->header.sourceId, OCTOMAP_DATA_FRAGEMENT, (uint8_t*)&fragment, fragment.fragmentHeader.fragmentLength + sizeof(octoMapFragmentHeader_t));
                    vTaskDelay(M2T(FRAGMENT_SEND_INTERVAL));
                }
            }
        }
        // 缺失数据重传完成
        setTxState(DATA_SENT,DATA_SENDING);
        xSemaphoreGive(muDataTX);
    }
}

bool retrySendData(){
    
    // 重传
    xSemaphoreTake(muDataTX, portMAX_DELAY);
    if(!setTxState(DATA_SENDING,RETRY)){
        // 设置状态失败，说明当前状态不是RETRY
        DEBUG_PRINT("[retrySendData]setTxState failed, TX_STATE = %x\n", TX_STATE);
        xSemaphoreGive(muDataTX);
        return false;
    }
    retryTimes++;
    DEBUG_PRINT("[retrySendData]retry send data, retryTimes = %d,maxwaitTimes = %d\n", retryTimes, maxTxAckWaitTimes * STATE_CHECK_INTERVAL);
    // Ack等待超时清空
    TxAckTimeoutTimes = 0;
    // 重传间隔翻倍
    maxTxAckWaitTimes *= 2;
    // 重传一个数据激活对面的SACK接收
    if(isSackReceived){
        //  收到过SACK，重传缺少数据即可
        octoMapPacket_Sack_t *sack = (octoMapPacket_Sack_t*)lastReceiveSackPacket.data;
        uint8_t fragmentCount = ((dataTXLength + MAX_FRAGEMENT_DATA_LENGTH - 1) / MAX_FRAGEMENT_DATA_LENGTH);
        DEBUG_PRINT("[retrySendData]dataTXId = %d, dataTXLength = %d, fragmentCount = %d, sigFragmentlength = %d\n", dataTXId, dataTXLength, fragmentCount, MAX_FRAGEMENT_DATA_LENGTH);
        octoMapFragement_t fragment;
        if(sack->header.type == DISCRETE){
            for (int i = 0; i < sack->header.length; i++)
            {
                // 重发丢失数据
                uint8_t fragmentId = sack->missDataId[i];
                DEBUG_PRINT("[processOctoMapPacket_Sack]fragmentId = %d\n", fragmentId);
                if(fragmentId >= fragmentCount){
                    // 分片ID越界
                    DEBUG_PRINT("[processOctoMapPacket_Sack]fragmentId is out of range, fragmentId = %d, fragmentCount = %d\n", fragmentId, fragmentCount);
                    continue;
                }
                fragment.fragmentHeader.dataId = dataTXId;
                fragment.fragmentHeader.fragmentId = fragmentId;
                fragment.fragmentHeader.fragmentCount = fragmentCount;
                if(fragmentId == fragmentCount - 1){
                    fragment.fragmentHeader.fragmentLength = dataTXLength - fragmentId*MAX_FRAGEMENT_DATA_LENGTH;
                }
                else{
                    fragment.fragmentHeader.fragmentLength = MAX_FRAGEMENT_DATA_LENGTH;
                }
                memcpy(fragment.data, dataTXBuffer+fragmentId*MAX_FRAGEMENT_DATA_LENGTH, fragment.fragmentHeader.fragmentLength);
                sendAutoFlyPacket(lastReceiveSackPacket.header.sourceId, OCTOMAP_DATA_FRAGEMENT, (uint8_t*)&fragment, fragment.fragmentHeader.fragmentLength + sizeof(octoMapFragmentHeader_t));
                sendFragmentCount ++;
                vTaskDelay(M2T(FRAGMENT_SEND_INTERVAL));

                setTxState(DATA_SENT,DATA_SENDING);
                xSemaphoreGive(muDataTX);
                return true;
            }
        }else{
            // 连续数据丢失
            // 读取两个值
            for (int i = 0; i < sack->header.length; i+=2)
            {
                uint8_t start = sack->missDataId[i];
                uint8_t end = sack->missDataId[i+1];
                if(start >= fragmentCount || end >= fragmentCount){
                    // 分片ID越界
                    DEBUG_PRINT("[processOctoMapPacket_Sack]fragmentId is out of range, start = %d, end = %d, fragmentCount = %d\n", start, end, fragmentCount);
                    continue;
                }
                if(start > end){
                    // 起始ID大于结束ID
                    DEBUG_PRINT("[processOctoMapPacket_Sack]start is greater than end, start = %d, end = %d\n", start, end);
                    continue;
                }
                DEBUG_PRINT("[processOctoMapPacket_Sack]start = %d, end = %d\n", start, end);
                for (int j = start; j <= end; j++)
                {
                    // 重发丢失数据
                    fragment.fragmentHeader.dataId = dataTXId;
                    fragment.fragmentHeader.fragmentId = j;
                    fragment.fragmentHeader.fragmentCount = fragmentCount;
                    if(j == fragmentCount - 1){
                        fragment.fragmentHeader.fragmentLength = dataTXLength - j*MAX_FRAGEMENT_DATA_LENGTH;
                    }
                    else{
                        fragment.fragmentHeader.fragmentLength = MAX_FRAGEMENT_DATA_LENGTH;
                    }
                    memcpy(fragment.data, dataTXBuffer+j*MAX_FRAGEMENT_DATA_LENGTH, fragment.fragmentHeader.fragmentLength);
                    sendAutoFlyPacket(lastReceiveSackPacket.header.sourceId, OCTOMAP_DATA_FRAGEMENT, (uint8_t*)&fragment, fragment.fragmentHeader.fragmentLength + sizeof(octoMapFragmentHeader_t));
                    vTaskDelay(M2T(FRAGMENT_SEND_INTERVAL));

                    setTxState(DATA_SENT,DATA_SENDING);
                    xSemaphoreGive(muDataTX);
                    return true;
                }
            }
        }
        // 缺失数据重传完成
    }else{
        // 重传所有数据
        // 分片发送，不足一片也按一片发送
        // 计算分片数量,向上取整
        uint8_t fragmentCount = ((dataTXLength + MAX_FRAGEMENT_DATA_LENGTH - 1) / MAX_FRAGEMENT_DATA_LENGTH);
        DEBUG_PRINT("[retrySendData]dataTXId = %d, dataTXLength = %d, fragmentCount = %d, sigFragmentlength=%d\n", dataTXId, dataTXLength, fragmentCount, MAX_FRAGEMENT_DATA_LENGTH);
        octoMapFragement_t fragment;
        for (int i = 0; i < fragmentCount; i++)
        {
            fragment.fragmentHeader.dataId = dataTXId;
            fragment.fragmentHeader.fragmentId = i;
            fragment.fragmentHeader.fragmentCount = fragmentCount;
            if(i == fragmentCount - 1){
                fragment.fragmentHeader.fragmentLength = dataTXLength - i*MAX_FRAGEMENT_DATA_LENGTH;
            }
            else{
                fragment.fragmentHeader.fragmentLength = MAX_FRAGEMENT_DATA_LENGTH;
            }
            memcpy(fragment.data, dataTXBuffer+i*MAX_FRAGEMENT_DATA_LENGTH, fragment.fragmentHeader.fragmentLength);
            sendAutoFlyPacket(destinationTxId, OCTOMAP_DATA_FRAGEMENT, (uint8_t*)&fragment, fragment.fragmentHeader.fragmentLength + sizeof(octoMapFragmentHeader_t));
            sendFragmentCount ++;
            vTaskDelay(M2T(FRAGMENT_SEND_INTERVAL));

            setTxState(DATA_SENT,DATA_SENDING);
            xSemaphoreGive(muDataTX);
            return true;
        }
        setTxState(DATA_SENT,DATA_SENDING);
        xSemaphoreGive(muDataTX);
    }
    return true;
}

void octoMapDataCommunicationTxTask(void * parameter){
    while (1)
    {
        switch (TX_STATE)
        {
            case IDLE:
                break;
            case FAILED:
                DEBUG_PRINT("[octoMapDataCommunicationTxTask]data send failed, dataId = %d, sendFragmentCount=%d\n", dataTXId, sendFragmentCount);
                xSemaphoreTake(muDataTX, portMAX_DELAY);
                dataTXLength = 0;
                TxAckTimeoutTimes = 0;
                maxTxAckWaitTimes = TX_ACK_WAIT_TIME_INIT;
                retryTimes = 0;

                dataTXId++;
                sendFragmentCount = 0;
                setTxState(IDLE,ANY);
                setTestSendEnableNext(true);
                xSemaphoreGive(muDataTX);
                break;
            case FIN:
                DEBUG_PRINT("[octoMapDataCommunicationTxTask]data send finished, dataId = %d, sendFragmentCount=%d\n", dataTXId, sendFragmentCount);
                sendFragmentCount = 0;
                setTxState(IDLE,ANY);
                setTestSendEnableNext(true);
                break;
            case DATA_SENDING:
                break;
            case DATA_SENT:
                // 发送完成转入ACK等待状态
                setTxState(ACK_WAIT,DATA_SENT);
                break;
            case ACK_WAIT:
                // 等待ACK，累积ACK超时次数
                TxAckTimeoutTimes++;
                // 超时重传
                if(TxAckTimeoutTimes > maxTxAckWaitTimes){
                    setTxState(ACK_TIMEOUT,ACK_WAIT);
                }
                break;
            case ACK_RECEIVED:
                break;
            case ACK_TIMEOUT:
                setTxState(RETRY,ACK_TIMEOUT);
                break;
            case RETRY:
                if(retryTimes >= MAX_TX_RETRY_TIME){
                    octoMapPacket_Error_t error;
                    error.dataId = dataTXId;
                    DEBUG_PRINT("[retrySendData]retry send times is too much, retryTimes = %d\n", retryTimes);
                    sendAutoFlyPacket(destinationTxId, OCTOMAP_ERROR_TX_WAITING_TIMEOUT, (uint8_t*)&error, sizeof(octoMapPacket_Error_t));
                    setTxState(FAILED,ANY);
                    retryTimes = 0;
                }else{
                    retrySendData();
                }
                break;
            case ACK_BUSY:
                setTxState(DATA_SEND_SLEEP,ACK_BUSY);
                break;
            case DATA_SEND_SLEEP:
                // 在C89标准下，标签后必须紧跟一个语句，而不能直接是变量声明。
                setTxState(DATA_SEND_SLEEP,DATA_SEND_SLEEP);
                int j = 0;
                for (; j < MAX_OCTOMAP_SEND_SLEEP_TIME / MAX_OCTOMAP_SEND_SLEEP_CHECK_INTERVAL; j++)
                {
                    if(TX_STATE != DATA_SEND_SLEEP){
                        break;
                    }
                    vTaskDelay(M2T(MAX_OCTOMAP_SEND_SLEEP_CHECK_INTERVAL));
                }
                // 自然结束休眠状态，重传数据
                if(j == MAX_OCTOMAP_SEND_SLEEP_TIME / MAX_OCTOMAP_SEND_SLEEP_CHECK_INTERVAL){
                    retrySendData();
                }
                break;
            default:
                break;
        }
        vTaskDelay(M2T(STATE_CHECK_INTERVAL));
    }
    
}

void octoMapDataCommunicationRxTask(void * parameter){
    while (1)
    {
        switch (RX_STATE)
        {
            case IDLE:
                break;
            case FAILED:
                xSemaphoreTake(muDataRX, portMAX_DELAY);
                curDataFailed = true;
                curDataComplete = false;
                octoMapDataCommunicationRxInit();
                setRxState(IDLE,ANY);
                xSemaphoreGive(muDataRX);
                break;
            case FIN:
                setRxState(IDLE,ANY);
                break;
            case DATA_RECEIVING:
                break;
            case DATA_RECEIVED:
                setRxState(FIN_SEND,DATA_RECEIVED);
                break;
            case DATA_WAITING:
                RxDataUpdateTimeoutTimes++;
                if(RxDataUpdateTimeoutTimes > maxRxDataUpdateWaitTimes){
                    setRxState(DATA_TIMEOUT,DATA_WAITING);
                    RxDataUpdateTimeoutTimes = 0;
                    maxRxDataUpdateWaitTimes *= 2;
                }
                break;
            case FIN_SEND:
                if(finSendTimes < MAX_FIN_SEND_TIME){
                    if(finSendTimes % FIN_SEND_INTERVAL_TIMES == 0){
                        xSemaphoreTake(muDataRX, portMAX_DELAY);
                        // 再次检索防止来新数据致使数据ID变化
                        if(RX_STATE == FIN_SEND){
                            octoMapPacket_Fin_t fin;
                            fin.dataId = dataRXId;
                            DEBUG_PRINT("[octoMapDataCommunicationRxTask]send FIN, dataId = %d\n", fin.dataId);
                            sendAutoFlyPacket(sourceRxId, OCTOMAP_FIN, (uint8_t*)&fin, FIN_PACKET_SIZE);
                            finSendTimes++;
                        }
                        xSemaphoreGive(muDataRX);
                    }
                }else{
                    setRxState(IDLE,ANY);
                    finSendTimes = 0;
                }
                break;
            case DATA_TIMEOUT:
                setRxState(SACK_SEND,DATA_TIMEOUT);
                break;
            case SACK_SEND:
                if(RxSackSendTimes > MAX_RX_SACK_SEND_TIME){
                    // SACK发送次数过多
                    xSemaphoreTake(muDataRX, portMAX_DELAY);
                    if(RX_STATE == SACK_SEND){
                        DEBUG_PRINT("[sendOctomapSack]SACK send times is too much, RxSackSendTimes = %d\n", RxSackSendTimes);
                        octoMapPacket_Error_t error;
                        error.dataId = dataRXId;
                        sendAutoFlyPacket(sourceRxId, OCTOMAP_ERROR_RX_WAITING_TIMEOUT, (uint8_t*)&error, sizeof(octoMapPacket_Error_t));
                        setRxState(FAILED,ANY);
                    }
                    xSemaphoreGive(muDataRX);
                }else{
                    sendOctomapSack();
                }
                break;
            default:
                break;
        }
        vTaskDelay(M2T(STATE_CHECK_INTERVAL));
    }
}