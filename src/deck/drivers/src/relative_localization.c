#include "relative_localization.h"
#include "debug.h"

#include <string.h>
#include <stdint.h>
#include <math.h>
#include "arm_math.h"
#include "FreeRTOS.h"
#include "task.h"

#include "param.h"

#include "log.h"
#include "system.h"
#include "semphr.h"
#include "swarm_ranging.h"
#include <radiolink.h>
#include "estimator_kalman.h"

static uint16_t MY_UWB_ADDRESS;
static bool isInit;
#define UPDATE_LOCATION_TICK 50

static float Qv = 0.2f;         // velocity deviation,初始值为1.0    -0.25
static float Qr = 0.1f;         // yaw rate deviation
static float Ruwb = 0.2f;       // ranging deviation                -0.5
static float InitCovPos = 0.2f; // 初始位置误差
static float InitCovYaw = 0.2f; // 初始偏航角误差

static relaVariable_t relaVar[RANGING_TABLE_SIZE];

static Realtime_Relative_Location_t realtimeRelativeLocation[RANGING_TABLE_SIZE];

static float A[STATE_DIM_rl][STATE_DIM_rl];
static float h[STATE_DIM_rl] = {0};
static arm_matrix_instance_f32 H = {1, STATE_DIM_rl, h};
static arm_matrix_instance_f32 Am = {STATE_DIM_rl, STATE_DIM_rl, (float *)A};

// Temporary matrices for the covariance updates
static float tmpNN1d[STATE_DIM_rl * STATE_DIM_rl];
static arm_matrix_instance_f32 tmpNN1m = {STATE_DIM_rl, STATE_DIM_rl, tmpNN1d};
static float tmpNN2d[STATE_DIM_rl * STATE_DIM_rl];
static arm_matrix_instance_f32 tmpNN2m = {STATE_DIM_rl, STATE_DIM_rl, tmpNN2d};
static float K[STATE_DIM_rl];
static arm_matrix_instance_f32 Km = {STATE_DIM_rl, 1, (float *)K};
static float tmpNN3d[STATE_DIM_rl * STATE_DIM_rl];
static arm_matrix_instance_f32 tmpNN3m = {STATE_DIM_rl, STATE_DIM_rl, tmpNN3d};
static float HTd[STATE_DIM_rl * 1];
static arm_matrix_instance_f32 HTm = {STATE_DIM_rl, 1, HTd};
static float PHTd[STATE_DIM_rl * 1];
static arm_matrix_instance_f32 PHTm = {STATE_DIM_rl, 1, PHTd};

static float vxj_t, vyj_t;
static float vxi_t, vyi_t;
static uint16_t hi_t, hj_t; // height of robot i and j

static float vxj, vyj, rj; // receive vx, vy, gz and distance
static float vxi, vyi, ri; // self vx, vy, gz
static uint16_t dij;       // distance between self i and other j
static float hi, hj;       // height of robot i and j

// static int16_t initRelativePosition[RANGING_TABLE_SIZE][RANGING_TABLE_SIZE][STATE_DIM_rl - 1];/*用于在指定无人机的初始位置时使用,由于在同一水平面，为了节省内存就不用Z轴了*/

// 初始时，所有无人机基于0号无人机的相对位置
// 正方向编队方案
// const float_t initDist = 1.3;
static const float_t initDist = 1;
static const float_t doubInitDist = 2;
static const float_t initPositionRela0[25][STATE_DIM_rl] = {
    {0.0f, 0.0f, 0.0f},               // 0
    {0.0f, -initDist, 0.0f},          // 1
    {-initDist, -initDist, 0.0f},     // 2
    {-initDist, 0.0f, 0.0f},          // 3
    {-initDist, initDist, 0.0f},      // 4
    {0.0f, initDist, 0.0f},           // 5
    {initDist, initDist, 0.0f},       // 6
    {initDist, 0.0f, 0.0f},           // 7
    {initDist, -initDist, 0.0f},      // 8
    {doubInitDist, initDist, 0.0f},   // 9
    {doubInitDist, 0.0f, 0.0f},       // 10
    {doubInitDist, -initDist, 0.0f},  // 11
    {initDist, -doubInitDist, 0.0f},  // 12
    {0.0f, -doubInitDist, 0.0f},      // 13
    {-initDist, -doubInitDist, 0.0f}, // 14
    {-doubInitDist, -initDist, 0.0f}, // 15
    {-doubInitDist, 0.0f, 0.0f},      // 16
    {-doubInitDist, initDist, 0.0f},  // 17
    {-initDist, doubInitDist, 0.0f},  // 18
    {0.0f, doubInitDist, 0.0f},       // 19
    {initDist, doubInitDist, 0.0f},   // 20
    {0.0f, 0.0f, 0.0f}};
// 八边形编队方案
// static const float_t initPositionRela0[15][STATE_DIM_rl] = {
//     {0.0f, 0.0f, 0.0f},   // 0
//     {-1.8f, -0.9f, 0.0f}, // 1
//     {-1.8f, 0.9f, 0.0f},  // 2
//     {-0.9f, 1.8f, 0.0f},  // 3
//     {0.9f, 1.8f, 0.0f},   // 4
//     {1.8f, 0.9f, 0.0f},   // 5
//     {1.8f, -0.9f, 0.0f},  // 6
//     {0.9, -1.8f, 0.0f},   // 7
//     {-0.9f, -1.8f, 0.0f}, // 8
//     {0.0f, 0.0f, 0.0f},   // 9
//     {0.0f, 0.0f, 0.0f}};  // 10

// 矩阵转置
static inline void mat_trans(const arm_matrix_instance_f32 *pSrc, arm_matrix_instance_f32 *pDst)
{
    configASSERT(ARM_MATH_SUCCESS == arm_mat_trans_f32(pSrc, pDst));
}
// 矩阵求逆
static inline void mat_inv(const arm_matrix_instance_f32 *pSrc, arm_matrix_instance_f32 *pDst)
{
    configASSERT(ARM_MATH_SUCCESS == arm_mat_inverse_f32(pSrc, pDst));
}
// 矩阵相乘
static inline void mat_mult(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst)
{
    configASSERT(ARM_MATH_SUCCESS == arm_mat_mult_f32(pSrcA, pSrcB, pDst));
}
// 求根号
static inline float arm_sqrt(float32_t in)
{
    float pOut = 0;
    arm_status result = arm_sqrt_f32(in, &pOut);
    configASSERT(ARM_MATH_SUCCESS == result);
    return pOut;
}

static void updateLocationTimerCallback(TimerHandle_t timer)
{
    // 遍历rangingTable中所有的成员进行位置更新
    CurrentNeighborAddressInfo_t *currentNeighborAddressInfo = getGlobalCurrentNeighborAddressInfo();
    xSemaphoreTake(currentNeighborAddressInfo->mu, portMAX_DELAY);
    for (int i = 0; i < currentNeighborAddressInfo->size; i++)
    {

        UWB_Address_t neighborAddress = currentNeighborAddressInfo->address[i];
        ASSERT(neighborAddress < RANGING_TABLE_SIZE_MAX);
        uint32_t dt = (float)(xTaskGetTickCount() - realtimeRelativeLocation[neighborAddress].oldTimetick) / configTICK_RATE_HZ;
        // 获取自己的
        estimatorKalmanGetSwarmInfo(&vxi, &vyi, &ri, &hi);
        updateRealtimeLocationImuInfo(neighborAddress, vxi, vyi, ri, hi, xTaskGetTickCount());
        vxi = vxi / 100;
        vyi = vyi / 100;
        // DEBUG_PRINT("locVx:%f\n", vxi);
        // 获取邻居的
        getLatestNeighborStateInfo(neighborAddress, &vxj, &vyj, &rj);
        vxj /= 100;
        vyj /= 100;
        // 更新
        relativeLocationPredict(neighborAddress, vxi, vyi, ri, vxj, vyj, rj, dt);
    }
    xSemaphoreGive(currentNeighborAddressInfo->mu);
}

static void initUpdateLocationTimer()
{
    static TimerHandle_t updateLocationTimer;
    DEBUG_PRINT("init updateLocationTimer\n");
    updateLocationTimer = xTimerCreate("imu_state_timer",
                                       M2T(UPDATE_LOCATION_TICK),
                                       pdTRUE,
                                       (void *)0,
                                       updateLocationTimerCallback);
    if (updateLocationTimer != NULL)
    {
        xTimerStart(updateLocationTimer, M2T(0));
        DEBUG_PRINT("succ timer");
    }
    else
    {
        DEBUG_PRINT("fail timer");
    }
}

void relativeLocoInit(void)
{
    if (isInit)
    {
        return;
    }
    while (isRangingInitComplete() == false)
    {
        vTaskDelay(100);
    }
    MY_UWB_ADDRESS = uwbGetAddress();
    initUpdateLocationTimer();
    xTaskCreate(relativeLocoTask, "relative_Localization", ZRANGER_TASK_STACKSIZE + 150, NULL, ZRANGER_TASK_PRI, NULL);
    isInit = true;
}

void initRelaVar(relaVariable_t *relaVar, uint16_t neighborAddress)
{
    ASSERT(neighborAddress <= RANGING_TABLE_SIZE);
    for (int i = 0; i < STATE_DIM_rl; i++)
    {
        for (int j = 0; j < STATE_DIM_rl; j++)
        {
            relaVar[neighborAddress].P[i][j] = 0;
        }
    }
    relaVar[neighborAddress].P[STATE_rlX][STATE_rlX] = InitCovPos;
    relaVar[neighborAddress].P[STATE_rlY][STATE_rlY] = InitCovPos;
    relaVar[neighborAddress].P[STATE_rlYaw][STATE_rlYaw] = InitCovYaw;
    relaVar[neighborAddress].S[STATE_rlX] = 0;
    relaVar[neighborAddress].S[STATE_rlY] = 0;
    relaVar[neighborAddress].S[STATE_rlYaw] = 0;
    // relaVar[neighborAddress].S[STATE_rlX] = initRelativePosition[neighborAddress][MY_UWB_ADDRESS][STATE_rlX]; // 设定初始位置
    // relaVar[neighborAddress].S[STATE_rlY] = initRelativePosition[neighborAddress][MY_UWB_ADDRESS][STATE_rlY];
    // relaVar[neighborAddress].S[STATE_rlYaw] = initRelativePosition[neighborAddress][MY_UWB_ADDRESS][STATE_rlYaw];
    /*设定初始位置*/
    relaVar[neighborAddress].S[STATE_rlX] = initPositionRela0[neighborAddress][STATE_rlX] - initPositionRela0[MY_UWB_ADDRESS][STATE_rlX]; // 设定初始位置
    relaVar[neighborAddress].S[STATE_rlY] = initPositionRela0[neighborAddress][STATE_rlY] - initPositionRela0[MY_UWB_ADDRESS][STATE_rlY];
    relaVar[neighborAddress].S[STATE_rlYaw] = 0;
    /*----------*/
    relaVar[neighborAddress].oldTimetick = xTaskGetTickCount();
    relaVar[neighborAddress].oldMsgSequence = 0;

    // DEBUG_PRINT("%f\n", relaVar[neighborAddress].S[STATE_rlX]);
}

void relativeLocoTask(void *arg)
{
    systemWaitStart();
    while (1)
    {
        vTaskDelay(1);
        UWB_Address_t neighborAddress;
        if (xQueueReceive(queueDistUpdatedAddress, &neighborAddress, portMAX_DELAY))
        {
            // DEBUG_PRINT("location address:%d\n", neighborAddress);
            bool isNewAdd; // 邻居是否是新加入的

            if (getNeighborStateInfo(neighborAddress, &relaVar[neighborAddress].oldMsgSequence, &dij, &vxj_t, &vyj_t, &rj, &hj_t, &isNewAdd))
            {
                // DEBUG_PRINT("location seq:%d\n", relaVar[neighborAddress].oldMsgSequence);
                vxj = (vxj_t + 0.0) / 100;
                vyj = (vyj_t + 0.0) / 100;
                hj = (hj_t + 0.0) / 100;
                if (isNewAdd)
                {
                    initRelaVar(relaVar, neighborAddress);
                    // 相对定位初始化完成，更新最新位置
                    updateRealtimeLocationFromRelaVar(neighborAddress);
                    initRealtimeLocation(realtimeRelativeLocation, neighborAddress);
                }
                else
                {
                    getCurrImuInfo(neighborAddress, &vxi_t, &vyi_t, &ri, &hi_t); // 当前无人机的信息
                    // DEBUG_PRINT("vxi:%f\n",vxi_t);
                    // DEBUG_PRINT("vyi:%f\n",vyi_t);
                    vxi = (vxi_t + 0.0) / 100;
                    vyi = (vyi_t + 0.0) / 100;
                    hi = (hi_t + 0.0) / 100;
                    uint32_t osTick = xTaskGetTickCount();
                    float dtEKF = (float)(osTick - relaVar[neighborAddress].oldTimetick) / configTICK_RATE_HZ;
                    relaVar[neighborAddress].oldTimetick = osTick;
                    relaVar[neighborAddress].height = hj;
                    // 又要开始新的一轮统计了
                    initRealtimeLocation(realtimeRelativeLocation, neighborAddress);
                    relativeEKF(neighborAddress, vxi, vyi, ri, hi, vxj, vyj, rj, hj, dij, dtEKF);
                    // 相对定位完成，更新校正后的位置
                    updateRealtimeLocationFromRelaVar(neighborAddress);
                }
                // DEBUG_PRINT("addr:%d,X:%f,Y:%f\n",neighborAddress,relaVar[neighborAddress].S[STATE_rlX],relaVar[neighborAddress].S[STATE_rlY]);
            }
        }
    }
}

void relativeLocationPredict(int n, float vxi, float vyi, float ri, float vxj, float vyj, float rj, float dt)
{
    // some preprocessing
    arm_matrix_instance_f32 Pm = {STATE_DIM_rl, STATE_DIM_rl, (float *)relaVar[n].P};
    float cyaw = arm_cos_f32(realtimeRelativeLocation[n].S[STATE_rlYaw]);
    float syaw = arm_sin_f32(realtimeRelativeLocation[n].S[STATE_rlYaw]);
    float xij = realtimeRelativeLocation[n].S[STATE_rlX];
    float yij = realtimeRelativeLocation[n].S[STATE_rlY];

    // prediction
    realtimeRelativeLocation[n].S[STATE_rlX] = xij + (cyaw * vxj - syaw * vyj - vxi + ri * yij) * dt;
    realtimeRelativeLocation[n].S[STATE_rlY] = yij + (syaw * vxj + cyaw * vyj - vyi - ri * xij) * dt;
    realtimeRelativeLocation[n].S[STATE_rlYaw] = realtimeRelativeLocation[n].S[STATE_rlYaw] + (rj - ri) * dt;

    DEBUG_PRINT("X:%d,Y:%d\n",realtimeRelativeLocation[n].S[STATE_rlX],realtimeRelativeLocation[n].S[STATE_rlY]);
}

void relativeEKF(int n, float vxi, float vyi, float ri, float hi, float vxj, float vyj, float rj, float hj, uint16_t dij, float dt)
{
    // some preprocessing
    arm_matrix_instance_f32 Pm = {STATE_DIM_rl, STATE_DIM_rl, (float *)relaVar[n].P};
    float cyaw = arm_cos_f32(relaVar[n].S[STATE_rlYaw]);
    float syaw = arm_sin_f32(relaVar[n].S[STATE_rlYaw]);
    float xij = relaVar[n].S[STATE_rlX];
    float yij = relaVar[n].S[STATE_rlY];

    // prediction
    relaVar[n].S[STATE_rlX] = xij + (cyaw * vxj - syaw * vyj - vxi + ri * yij) * dt;
    relaVar[n].S[STATE_rlY] = yij + (syaw * vxj + cyaw * vyj - vyi - ri * xij) * dt;
    relaVar[n].S[STATE_rlYaw] = relaVar[n].S[STATE_rlYaw] + (rj - ri) * dt;
    // A状态转移矩阵
    A[0][0] = 1;
    A[0][1] = ri * dt;
    A[0][2] = (-syaw * vxj - cyaw * vyj) * dt;
    A[1][0] = -ri * dt;
    A[1][1] = 1;
    A[1][2] = (cyaw * vxj - syaw * vyj) * dt;
    A[2][0] = 0;
    A[2][1] = 0;
    A[2][2] = 1;

    mat_mult(&Am, &Pm, &tmpNN1m);      // A P
    mat_trans(&Am, &tmpNN2m);          // A'
    mat_mult(&tmpNN1m, &tmpNN2m, &Pm); // A P A'

    // BQB' = [ Qv*c^2 + Qv*s^2 + Qr*y^2 + Qv,                       -Qr*x*y, -Qr*y]
    //        [                       -Qr*x*y, Qv*c^2 + Qv*s^2 + Qr*x^2 + Qv,  Qr*x]
    //        [                         -Qr*y,                          Qr*x,  2*Qr]*dt^2
    float dt2 = dt * dt;
    relaVar[n].P[0][0] += dt2 * (Qv + Qv + Qr * yij * yij);
    relaVar[n].P[0][1] += dt2 * (-Qr * xij * yij);
    relaVar[n].P[0][2] += dt2 * (-Qr * yij);
    relaVar[n].P[1][0] += dt2 * (-Qr * xij * yij);
    relaVar[n].P[1][1] += dt2 * (Qv + Qv + Qr * xij * xij);
    relaVar[n].P[1][2] += dt2 * (Qr * xij);
    relaVar[n].P[2][0] += dt2 * (-Qr * yij);
    relaVar[n].P[2][1] += dt2 * (Qr * xij);
    relaVar[n].P[2][2] += dt2 * (2 * Qr);

    xij = relaVar[n].S[STATE_rlX];
    yij = relaVar[n].S[STATE_rlY];
    float distPred = arm_sqrt(xij * xij + yij * yij + (hi - hj) * (hi - hj)) + 0.0001f;
    float distMeas = (float)(dij / 100.0f);
    // h矩阵
    h[0] = xij / distPred;
    h[1] = yij / distPred;
    h[2] = 0;

    mat_trans(&H, &HTm);        // H'
    mat_mult(&Pm, &HTm, &PHTm); // PH'
    float HPHR = powf(Ruwb, 2); // HPH' + R
    for (int i = 0; i < STATE_DIM_rl; i++)
    {                                 // Add the element of HPH' to the above
        HPHR += H.pData[i] * PHTd[i]; // this obviously only works if the update is scalar (as in this function)
    }
    for (int i = 0; i < STATE_DIM_rl; i++)
    {
        K[i] = PHTd[i] / HPHR; // kalman gain = (PH' (HPH' + R )^-1)
        // DEBUG_PRINT("K[%d]:%f\n", i, K[i]);
        // DEBUG_PRINT("relaVarStart:%.3lf", relaVar[n].S[i]);
        relaVar[n].S[i] = relaVar[n].S[i] + K[i] * (distMeas - distPred); // state update
        // DEBUG_PRINT(",relaVar:%.3f,K:%.3f,distMeas:%.3f,distPred%.3f\n", relaVar[n].S[i], K[i], distMeas, distPred);
    }
    mat_mult(&Km, &H, &tmpNN1m); // KH
    for (int i = 0; i < STATE_DIM_rl; i++)
    {
        tmpNN1d[STATE_DIM_rl * i + i] -= 1;
    } // KH - I
    mat_trans(&tmpNN1m, &tmpNN2m);     // (KH - I)'
    mat_mult(&tmpNN1m, &Pm, &tmpNN3m); // (KH - I)*P
    mat_mult(&tmpNN3m, &tmpNN2m, &Pm); // (KH - I)*P*(KH - I)'
    // DEBUG_PRINT("dis:%d\n", dij);
}

void updateRealtimeLocationFromRelaVar(UWB_Address_t neighborAddress)
{
    realtimeRelativeLocation[neighborAddress].S[STATE_rlX] = relaVar[neighborAddress].S[STATE_rlX];
    realtimeRelativeLocation[neighborAddress].S[STATE_rlY] = relaVar[neighborAddress].S[STATE_rlY];
    realtimeRelativeLocation[neighborAddress].S[STATE_rlYaw] = relaVar[neighborAddress].S[STATE_rlYaw];
    realtimeRelativeLocation[neighborAddress].oldTimetick = relaVar[neighborAddress].oldTimetick;
    DEBUG_PRINT("update-X:%d,Y:%d\n",realtimeRelativeLocation[neighborAddress].S[STATE_rlX],realtimeRelativeLocation[neighborAddress].S[STATE_rlY]);
}

void updateRealtimeLocationImuInfo(UWB_Address_t neighborAddress, float velocityXInWorld, float velocityYInWorld, float gyroZ, float posiZ, uint32_t updatedTick)
{
    // 和当前的进行更新
    uint32_t holdTick = realtimeRelativeLocation[neighborAddress].imuState.allTickCount;
    uint32_t diffTickCount = updatedTick - realtimeRelativeLocation[neighborAddress].imuState.lastUpdateTick;
    uint32_t allTickCount = diffTickCount + holdTick;
    ASSERT(allTickCount > 0);
    realtimeRelativeLocation[neighborAddress].imuState.velocityXInWorld = ((holdTick * realtimeRelativeLocation[neighborAddress].imuState.velocityXInWorld) + (diffTickCount * velocityXInWorld)) / (allTickCount);
    realtimeRelativeLocation[neighborAddress].imuState.velocityYInWorld = ((holdTick * realtimeRelativeLocation[neighborAddress].imuState.velocityYInWorld) + (diffTickCount * velocityYInWorld)) / (allTickCount);
    realtimeRelativeLocation[neighborAddress].imuState.gyroZ = ((holdTick * realtimeRelativeLocation[neighborAddress].imuState.gyroZ) + (diffTickCount * gyroZ)) / (allTickCount);
    realtimeRelativeLocation[neighborAddress].imuState.posiZ = posiZ;

    // 更新最新的均值的时间
    realtimeRelativeLocation[neighborAddress].imuState.lastUpdateTick = updatedTick;
    // 更新当前均值持续的时间
    realtimeRelativeLocation[neighborAddress].imuState.allTickCount = allTickCount;
}

void initRealtimeLocation(Realtime_Relative_Location_t *realtimeRelativeLocation, UWB_Address_t neighborAddress)
{
    realtimeRelativeLocation[neighborAddress].imuState.allTickCount = 0;
    realtimeRelativeLocation[neighborAddress].imuState.lastUpdateTick = 0;
}

void initRealtimeLocationList(Realtime_Relative_Location_t *realtimeRelativeLocation)
{
    for (int i = 0; i < RANGING_TABLE_SIZE; i++)
    {
        initRealtimeLocation(initRealtimeLocation, i);
    }
}

void getCurrImuInfo(UWB_Address_t neighborAddress, float *vxi, float *vyi, float *ri, float *hi)
{
    *vxi = realtimeRelativeLocation[neighborAddress].imuState.velocityXInWorld;
    *vyi = realtimeRelativeLocation[neighborAddress].imuState.velocityYInWorld;
    *ri = realtimeRelativeLocation[neighborAddress].imuState.gyroZ;
    *hi = realtimeRelativeLocation[neighborAddress].imuState.posiZ;
}

void copyTargetList(float_t *dest, float_t *src)
{
    for (int i = 0; i < ARRAY_LENGTH; i++)
    {
        *(dest + i * STATE_DIM_rl + 0) = *(src + i * STATE_DIM_rl + 0);
        *(dest + i * STATE_DIM_rl + 1) = *(src + i * STATE_DIM_rl + 1);
        *(dest + i * STATE_DIM_rl + 2) = *(src + i * STATE_DIM_rl + 2);
    }
}

Realtime_Relative_Location_t *getGlobalRealtimeLocation()
{
    return realtimeRelativeLocation;
}

// LOG_GROUP_START(relative_pos)
// LOG_ADD(LOG_FLOAT, rlX0, &relaVar[0].S[STATE_rlX])
// LOG_ADD(LOG_FLOAT, rlY0, &relaVar[0].S[STATE_rlY])
// LOG_ADD(LOG_FLOAT, rlYaw0, &relaVar[0].S[STATE_rlYaw])

// LOG_ADD(LOG_FLOAT, rlX1, &relaVar[1].S[STATE_rlX])
// LOG_ADD(LOG_FLOAT, rlY1, &relaVar[1].S[STATE_rlY])
// LOG_ADD(LOG_FLOAT, rlYaw1, &relaVar[1].S[STATE_rlYaw])

// LOG_ADD(LOG_FLOAT, rlX2, &relaVar[2].S[STATE_rlX])
// LOG_ADD(LOG_FLOAT, rlY2, &relaVar[2].S[STATE_rlY])
// LOG_ADD(LOG_FLOAT, rlYaw2, &relaVar[2].S[STATE_rlYaw])

// LOG_ADD(LOG_FLOAT, rlX3, &relaVar[3].S[STATE_rlX])
// LOG_ADD(LOG_FLOAT, rlY3, &relaVar[3].S[STATE_rlY])
// LOG_ADD(LOG_FLOAT, rlYaw3, &relaVar[3].S[STATE_rlYaw])

// LOG_ADD(LOG_FLOAT, rlX4, &relaVar[4].S[STATE_rlX])
// LOG_ADD(LOG_FLOAT, rlY4, &relaVar[4].S[STATE_rlY])
// LOG_ADD(LOG_FLOAT, rlYaw4, &relaVar[4].S[STATE_rlYaw])

// LOG_ADD(LOG_FLOAT, rlX5, &relaVar[5].S[STATE_rlX])
// LOG_ADD(LOG_FLOAT, rlY5, &relaVar[5].S[STATE_rlY])
// LOG_ADD(LOG_FLOAT, rlYaw5, &relaVar[5].S[STATE_rlYaw])
// LOG_GROUP_STOP(relative_pos)

// PARAM_GROUP_START(arelative_pos)
// PARAM_ADD(PARAM_FLOAT, noiFlow, &Qv) // make sure the name is not too long
// PARAM_ADD(PARAM_FLOAT, noiGyroZ, &Qr)
// PARAM_ADD(PARAM_FLOAT, noiUWB, &Ruwb)
// PARAM_ADD(PARAM_FLOAT, Ppos, &InitCovPos)
// PARAM_ADD(PARAM_FLOAT, Pyaw, &InitCovYaw)
// PARAM_GROUP_STOP(arelative_pos)