#include "imu_state.h"
#include "debug.h"

ImuStateList_t imuStateList;

// Initialize the circular buffer
void initImuStateList(ImuStateList_t *list)
{
    list->head = 0;
    list->curr = 0;
    list->size = 0;
    list->mu = xSemaphoreCreateMutex();
}

// Check if the buffer is full
int isBufferFull(ImuStateList_t *list)
{
    return list->size == IMU_STATE_LIST_LENGTH;
}

// Check if the buffer is empty
int isBufferEmpty(ImuStateList_t *list)
{
    return list->size == 0;
}

// Add a new IMU state to the buffer
void addImuState(ImuStateList_t *list, ImuState_t state)
{
    if (isBufferFull(list))
    {
    }
    else
    {
        list->size++;
    }
    list->imuStateList[list->head] = state;
    list->curr = list->head;
    list->head = (list->head + 1) % IMU_STATE_LIST_LENGTH; // Move head forward
}

void updateImuState(ImuStateList_t *list, ImuState_t newState, bool isFirstAdd)
{
    // 如果现在是空的，或者指定是新插入的，则插入
    if (isBufferEmpty(list) || isFirstAdd == true)
    {
        addImuState(list, newState);
    }
    else
    {
        ImuState_t *currState = &list->imuStateList[list->curr];
        if (currState->allTickCount != 0)
        {
            // 和当前的进行更新
            uint32_t diffTickCount = newState.lastUpdateTick - currState->lastUpdateTick;
            uint32_t allTickCount = diffTickCount + currState->allTickCount;
            // DEBUG_PRINT("new-count:%d\n",newState.lastUpdateTick);
            // DEBUG_PRINT("curr-count:%d\n",currState->lastUpdateTick);
            // DEBUG_PRINT("newVx:%f\n",newState.velocityXInWorld);
            // DEBUG_PRINT("currVx:%f\n",currState->velocityXInWorld);
            // DEBUG_PRINT("allcount:%d\n",currState->allTickCount);
            currState->velocityXInWorld = ((currState->allTickCount * currState->velocityXInWorld) + (diffTickCount * newState.velocityXInWorld)) / (allTickCount);
            currState->velocityYInWorld = ((currState->allTickCount * currState->velocityYInWorld) + (diffTickCount * newState.velocityYInWorld)) / (allTickCount);
            currState->gyroZ = ((currState->allTickCount * currState->gyroZ) + (diffTickCount * newState.gyroZ)) / (allTickCount);
            currState->posiZ = newState.posiZ;

            // 更新最新的均值的时间
            currState->lastUpdateTick = newState.lastUpdateTick;
            // 更新当前均值持续的时间
            currState->allTickCount = allTickCount;
            
        }
        else
        {
            // DEBUG_PRINT("newadd\n");
            currState->allTickCount = newState.allTickCount;
            currState->gyroZ = newState.gyroZ;
            currState->lastUpdateTick = newState.lastUpdateTick;
            currState->posiZ = newState.posiZ;
            currState->velocityXInWorld = newState.velocityXInWorld;
            currState->velocityYInWorld = newState.velocityYInWorld;
        }
    }
    
}

ImuStateList_t *getGlobalImuState()
{
    return &imuStateList;
}

static void collectHistoryImuStateTimerCallback(TimerHandle_t timer)
{
    ImuState_t newImuState;
    newImuState.lastUpdateTick = xTaskGetTickCount();
    newImuState.allTickCount = COLLECT_FREQUENCY_TICK;
    estimatorKalmanGetSwarmInfo(&newImuState.velocityXInWorld, &newImuState.velocityYInWorld, &newImuState.gyroZ, &newImuState.posiZ);
    xSemaphoreTake(imuStateList.mu, portMAX_DELAY);
    updateImuState(&imuStateList, newImuState, false);
    xSemaphoreGive(imuStateList.mu);
}

void initImuStateTimer()
{
    static TimerHandle_t collectHistoryImuStateTimer;
    DEBUG_PRINT("imu state timerStart\n");
    initImuStateList(&imuStateList);
    collectHistoryImuStateTimer = xTimerCreate("imu_state_timer",
                                               M2T(COLLECT_FREQUENCY_TICK),
                                               pdTRUE,
                                               (void *)0,
                                               collectHistoryImuStateTimerCallback);
    if (collectHistoryImuStateTimer != NULL)
    {
        xTimerStart(collectHistoryImuStateTimer, M2T(0));
        DEBUG_PRINT("succ timer");
    }else{
        DEBUG_PRINT("fail timer");
    }
}