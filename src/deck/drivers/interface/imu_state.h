#ifndef IMU_STATE_H
#define IMU_STATE_H

#include "FreeRTOS.h"
#include "timers.h"
#include "FreeRTOSConfig.h"
#include "estimator_kalman.h"
#include "task.h"
#include "semphr.h"

#define IMU_STATE_LIST_LENGTH 10
#define COLLECT_FREQUENCY_TICK 10


typedef struct ImuState_t
{
    float velocityXInWorld; // 2 byte cm/s 在世界坐标系下的速度（不是基于机体坐标系的速度）
    float velocityYInWorld; // 2 byte cm/s 在世界坐标系下的速度（不是基于机体坐标系的速度）
    float gyroZ;
    float posiZ;
    uint32_t allTickCount;
    uint32_t lastUpdateTick;
}ImuState_t;

typedef struct ImuStateList_t {
    ImuState_t imuStateList[IMU_STATE_LIST_LENGTH];
    int head;  // Points to the next slot for insertion
    int curr;  // 当前正在处理的位置
    int size;  // Tracks the number of elements in the buffer
    SemaphoreHandle_t mu;
} ImuStateList_t;


// Initialize the circular buffer
void initImuStateList(ImuStateList_t *list);

// Check if the buffer is full
int isBufferFull(ImuStateList_t *list);

// Check if the buffer is empty
int isBufferEmpty(ImuStateList_t *list);

// Add a new IMU state to the buffer
void addImuState(ImuStateList_t *list, ImuState_t state);

void updateImuState(ImuStateList_t *list, ImuState_t state, bool isFirstAdd);

ImuStateList_t* getGlobalImuState();

// 下面和timer有关系
void initImuStateTimer();


#endif