#ifndef RELATIVELOCA_H_
#define RELATIVELOCA_H_
#include "ranging_struct.h"
#include "swarm_ranging.h"
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
  uint32_t oldTimetick;
  bool receiveFlag;
} relaVariable_t;

void relativeLocoInit(void);
void relativeLocoTask(void *arg);
void relativeEKF(int n, float vxi, float vyi, float ri, float hi, float vxj, float vyj, float rj, float hj, uint16_t dij, float dt);
bool relativeInfoRead(float *relaVarParam, currentNeighborAddressInfo_t *dest);
void relaVarInit(relaVariable_t *relaVar, uint16_t neighborAddress); // // Initialize EKF for relative localization
#endif