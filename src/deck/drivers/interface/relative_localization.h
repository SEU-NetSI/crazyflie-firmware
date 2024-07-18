#ifndef RELATIVELOCA_H_
#define RELATIVELOCA_H_
#include "swarm_ranging.h"
#include "math.h"
typedef enum
{
  STATE_rlX,
  STATE_rlY,
  STATE_rlYaw,
  STATE_DIM_rl
} relative_stateIdx_t;

typedef enum
{
  INPUT_vxi,
  INPUT_vyi,
  INPUT_ri,
  INPUT_vxj,
  INPUT_vyj,
  INPUT_rj,
  INPUT_DIM
} relative_inputIdx_t;

typedef struct
{
  float S[STATE_DIM_rl];
  float P[STATE_DIM_rl][STATE_DIM_rl];
  float height;
  uint32_t oldTimetick;
  uint8_t receiveFlag;
} relaVariable_t;

/*--用于初始位置设定--*/
static uint8_t CONTROL_MODE = 1;
const static uint8_t ARRAY_LENGTH = 15;
// static  float_t TARGETLIST0[ARRAY_LENGTH][STATE_DIM_rl] = {
//     {0.0f, 0.0f, 0.0f},   // 0
//     {-1.5f, -1.5f, 0.0f}, // 1
//     {-1.5f, 0.0f, 0.0f},  // 2
//     {-1.5f, 1.5f, 0.0f},  // 3
//     {0.0f, 1.5f, 0.0f},   // 4
//     {1.5f, 1.5f, 0.0f},   // 5
//     {1.5f, 0.0f, 0.0f},   // 6
//     {1.5, -1.5f, 0.0f},   // 7
//     {0.0f, -1.5f, 0.0f},  // 8
//     {0.0f, 0.0f, 0.0f},   // 9
//     {0.0f, 0.0f, 0.0f}};  // 10
// static const float_t TARGETLIST1[ARRAY_LENGTH][STATE_DIM_rl] = {
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
void copyTargetList(float_t *dest, float_t *src);
/*--用于初始位置设定--*/

void relativeLocoInit(void);
void relativeLocoTask(void *arg);
void relativeEKF(int n, float vxi, float vyi, float ri, float hi, float vxj, float vyj, float rj, float hj, uint16_t dij, float dt);
bool relativeInfoRead(float *relaVarParam, float *neighbor_height, currentNeighborAddressInfo_t *dest);
void relaVarInit(relaVariable_t *relaVar, uint16_t neighborAddress); // // Initialize EKF for relative localization
#endif