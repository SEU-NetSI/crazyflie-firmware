#include "system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "commander.h"
#include "relative_localization.h"
#include "ranging_struct.h"
#include "num.h"
#include "param.h"
#include "debug.h"
#include <stdlib.h> // random
#include "configblock.h"
#include "uart2.h"
#include "log.h"
#include "math.h"
#include "adhocdeck.h"
#define RUNNING_STAGE 1 // 0代码debug阶段，1代码运行阶段

static uint16_t MY_UWB_ADDRESS;
static bool isInit;
static bool onGround = true;               // 无人机当前是否在地面上?
static bool isCompleteTaskAndLand = false; // 无人机是否已经执行了飞行任务并落地?
static bool keepFlying = false;
static setpoint_t setpoint;
static float_t relaVarInCtrl[RANGING_TABLE_SIZE + 1][STATE_DIM_rl];
static currentNeighborAddressInfo_t currentNeighborAddressInfo;
static float_t height = 0.5;

static float relaCtrl_p = 2.0f;
static float relaCtrl_i = 0.0001f;
static float relaCtrl_d = 0.01f;
// static float NDI_k = 2.0f;

static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;
  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;
  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;
  setpoint->velocity_body = true;
  commanderSetSetpoint(setpoint, 3);
}

static void flyRandomIn1meter(float_t randomVel)
{
  float_t randomYaw = (rand() / (float)RAND_MAX) * 6.28f; // 0-2pi rad
  // float_t randomVel = (rand() / (float)RAND_MAX) * 1;     // 0-1 m/s
  float_t vxBody = randomVel * cosf(randomYaw); // 速度分解
  float_t vyBody = randomVel * sinf(randomYaw);
  for (int i = 1; i < 100; i++)
  {
    setHoverSetpoint(&setpoint, vxBody, vyBody, height, 0);
    vTaskDelay(M2T(10));
  }
  setHoverSetpoint(&setpoint, 0, 0, height, 0);
  vTaskDelay(M2T(10));
  for (int i = 1; i < 100; i++)
  {
    setHoverSetpoint(&setpoint, -vxBody, -vyBody, height, 0);
    vTaskDelay(M2T(10));
  }
}

#define SIGN(a) ((a >= 0) ? 1 : -1)
static float_t targetX;
static float_t targetY;
static float PreErr_x = 0;
static float PreErr_y = 0;
static float IntErr_x = 0;
static float IntErr_y = 0;
static uint32_t PreTime;
static void formation0asCenter(float_t tarX, float_t tarY)
{
  float dt = (float)(xTaskGetTickCount() - PreTime) / configTICK_RATE_HZ;
  PreTime = xTaskGetTickCount();
  if (dt > 1) // skip the first run of the EKF
    return;
  // pid control for formation flight 当前是1号无人机
  float err_x = -(tarX - relaVarInCtrl[0][STATE_rlX]);
  float err_y = -(tarY - relaVarInCtrl[0][STATE_rlY]);
  float pid_vx = relaCtrl_p * err_x;  // 2.0*err_x 基于距离差进行一个速度控制
  float pid_vy = relaCtrl_p * err_y;  // 2.0*err_y
  float dx = (err_x - PreErr_x) / dt; // 先前的速度
  float dy = (err_y - PreErr_y) / dt;
  PreErr_x = err_x;
  PreErr_y = err_y;
  pid_vx += relaCtrl_d * dx; // 0.01*dx 先前速度*比例系数
  pid_vy += relaCtrl_d * dy; // 0.01*dy
  IntErr_x += err_x * dt;
  IntErr_y += err_y * dt;
  pid_vx += relaCtrl_i * constrain(IntErr_x, -0.5, 0.5); // += (+-)0.00005
  pid_vy += relaCtrl_i * constrain(IntErr_y, -0.5, 0.5);
  pid_vx = constrain(pid_vx, -1.5f, 1.5f);
  pid_vy = constrain(pid_vy, -1.5f, 1.5f);

  // float rep_x = 0.0f;
  // float rep_y = 0.0f;
  // for(uint8_t i=0; i<NumUWB; i++){
  //   if(i!=selfID){
  //     float dist = relaVarInCtrl[i][STATE_rlX]*relaVarInCtrl[i][STATE_rlX] + relaVarInCtrl[i][STATE_rlY]*relaVarInCtrl[i][STATE_rlY];
  //     dist = sqrtf(dist);
  //     rep_x += -0.5f * (SIGN(0.5f - dist) + 1) / (abs(relaVarInCtrl[i][STATE_rlX]) + 0.001f) * SIGN(relaVarInCtrl[i][STATE_rlX]);
  //     rep_y += -0.5f * (SIGN(0.5f - dist) + 1) / (abs(relaVarInCtrl[i][STATE_rlY]) + 0.001f) * SIGN(relaVarInCtrl[i][STATE_rlY]);
  //   }
  // }
  // rep_x = constrain(rep_x, -1.5f, 1.5f);
  // rep_y = constrain(rep_y, -1.5f, 1.5f);

  // pid_vx = constrain(pid_vx + rep_x, -1.5f, 1.5f);
  // pid_vy = constrain(pid_vy + rep_y, -1.5f, 1.5f);

  setHoverSetpoint(&setpoint, pid_vx, pid_vy, height, 0);
}

void take_off()
{
  for (int i = 0; i < 5; i++)
  {
    setHoverSetpoint(&setpoint, 0, 0, height, 0);
    vTaskDelay(M2T(100));
  }
  // for (int i = 0; i < 10 * MY_UWB_ADDRESS; i++)
  // {
  //   setHoverSetpoint(&setpoint, 0, 0, height, 0);
  //   vTaskDelay(M2T(100));
  // }
  onGround = false;
}

void land()
{
  // landing procedure
  if (!onGround)
  {
    int i = 0;
    float land_height_per_100ms = 0.02;            // 每秒下降的高度为该变量的值*10
    while (height - i * land_height_per_100ms > 0) // 1s下降0.2s
    {
      i++;
      setHoverSetpoint(&setpoint, 0, 0, height - (float)i * land_height_per_100ms, 0);
      vTaskDelay(M2T(100));
    }
    isCompleteTaskAndLand = true;
  }
  onGround = true;
}

float get_min(float *var_history, int len_history)
{
  float res = var_history[0];
  for (size_t i = 1; i < len_history; i++)
  {
    res = var_history[i] < res ? var_history[i] : res;
  }
  return res;
}

float get_max(float *var_history, int len_history)
{
  float res = var_history[0];
  for (size_t i = 1; i < len_history; i++)
  {
    res = var_history[i] > res ? var_history[i] : res;
  }
  return res;
}

void reset_estimators()
{

  int len_history = 10;
  float var_x_history[len_history];
  float var_y_history[len_history];
  float var_z_history[len_history];
  for (size_t i = 0; i < len_history; i++)
  {
    var_x_history[i] = 1000.0;
    var_y_history[i] = 1000.0;
    var_z_history[i] = 1000.0;
  }
  float threshold = 0.001;
  int i = 0;
  while (true)
  {
    /* PX,PY,PZ log variable id */
    float idVelocityX = logGetVarId("kalman", "varPX");
    float idVelocityY = logGetVarId("kalman", "varPY");
    float idVelocityZ = logGetVarId("kalman", "varPZ");
    float velocityX = logGetFloat(idVelocityX);
    float velocityY = logGetFloat(idVelocityY);
    float velocityZ = logGetFloat(idVelocityZ);
    var_x_history[i] = velocityX;
    var_y_history[i] = velocityY;
    var_z_history[i] = velocityZ;

    float min_x = get_min(var_x_history, len_history);
    float max_x = get_max(var_x_history, len_history);
    float min_y = get_min(var_y_history, len_history);
    float max_y = get_max(var_y_history, len_history);
    float min_z = get_min(var_z_history, len_history);
    float max_z = get_max(var_z_history, len_history);
    if (((max_x - min_x) < threshold) &&
        ((max_y - min_y) < threshold) &&
        ((max_z - min_z) < threshold))
    {
      break;
    }
    i = (i + 1) % len_history;
  }
}

void relativeControlTask(void *arg)
{
  int8_t targetShift = 0;
  uint8_t UAV_NUM = 9;
  static const float_t targetList[10][STATE_DIM_rl] = {
      {0.0f, 0.0f, 0.0f},   // 0
      {-1.5f, -1.5f, 0.0f}, // 1
      {-1.5f, 0.0f, 0.0f},  // 2
      {-1.5f, 1.5f, 0.0f},  // 3
      {0.0f, 1.5f, 0.0f},   // 4
      {1.5f, 1.5f, 0.0f},   // 5
      {1.5f, 0.0f, 0.0f},   // 6
      {1.5, -1.5f, 0.0f},   // 7
      {0.0f, -1.5f, 0.0f},  // 8
      {0.0f, 0.0f, 0.0f},   // 9
      {0.0f, 0.0f, 0.0f}};  // 10
  systemWaitStart();
  reset_estimators(); // 判断无人机数值是否收敛

  while (1)
  {
    vTaskDelay(10);
    keepFlying = getOrSetKeepflying(MY_UWB_ADDRESS, keepFlying);
    bool is_connect = relativeInfoRead((float_t *)relaVarInCtrl, &currentNeighborAddressInfo);
    int8_t leaderStage = getLeaderStage();
    // int8_t index = (MY_UWB_ADDRESS + targetShift) % (UAV_NUM-1) + 1; 
    //DEBUG_PRINT("%d,%d\n",keepFlying,leaderStage);
    if (RUNNING_STAGE == 1) // debug阶段就不能让无人机飞
    {
      if (is_connect && keepFlying && !isCompleteTaskAndLand)
      {

        // take off
        if (onGround)
        {
          vTaskDelay(2000); // 设定位置使得其收敛时间
          take_off();
          // onGround = false;
          setMyTakeoff(true);
        }
        // DEBUG_PRINT("tick:%d,rlx:%f,rly:%f,rlraw:%f\n", tickInterval, relaVarInCtrl[1][STATE_rlX], relaVarInCtrl[1][STATE_rlY], relaVarInCtrl[1][STATE_rlYaw]);
        // DEBUG_PRINT("tick:%f\n", relaVarInCtrl[0][STATE_rlYaw]);
        if (leaderStage == ZERO_STAGE) // 第0个阶段随机飞行
        {
          float_t randomVel = 0.7;
          flyRandomIn1meter(randomVel);
          targetX = relaVarInCtrl[0][STATE_rlX];
          targetY = relaVarInCtrl[0][STATE_rlY];
        }
        else if (leaderStage == FIRST_STAGE) // 第1个阶段跟随飞行
        {
          if (MY_UWB_ADDRESS == 0)
          {
            float_t randomVel = 0.7;      // 0-randomVel m/s
            flyRandomIn1meter(randomVel); //
          }
          else
          {
            formation0asCenter(targetX, targetY);
          }
        }
        else if (leaderStage != LAND_STAGE) // 第三个阶段
        {
          if (MY_UWB_ADDRESS == 0)
          {
            setHoverSetpoint(&setpoint, 0, 0, height, 0);
          }
          else
          {
            targetShift = leaderStage;
            // 使得targetList在1~UAV_NUM之间偏移
            int8_t index = (MY_UWB_ADDRESS + targetShift) % (UAV_NUM-1) + 1; // 目标地址索引
            targetX = -cosf(relaVarInCtrl[0][STATE_rlYaw]) * targetList[index][STATE_rlX] + sinf(relaVarInCtrl[0][STATE_rlYaw]) * targetList[index][STATE_rlY];
            targetY = -sinf(relaVarInCtrl[0][STATE_rlYaw]) * targetList[index][STATE_rlX] - cosf(relaVarInCtrl[0][STATE_rlYaw]) * targetList[index][STATE_rlY];
            formation0asCenter(targetX, targetY);
          }
        }
        else
        {
          // 运行90s之后，落地
          land();
        }
      }
      else
      {
        land();
      }
    }
  }
}

void relativeControlInit(void)
{
  if (isInit)
    return;
  MY_UWB_ADDRESS = getUWBAddress();
  xTaskCreate(relativeControlTask, "relative_Control", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
  isInit = true;
}

PARAM_GROUP_START(relative_ctrl)
PARAM_ADD(PARAM_UINT8, keepFlying, &keepFlying)
PARAM_ADD(PARAM_FLOAT, relaCtrl_p, &relaCtrl_p)
PARAM_ADD(PARAM_FLOAT, relaCtrl_i, &relaCtrl_i)
PARAM_ADD(PARAM_FLOAT, relaCtrl_d, &relaCtrl_d)
PARAM_GROUP_STOP(relative_ctrl)