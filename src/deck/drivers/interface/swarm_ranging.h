#ifndef _SWARM_RANGING_H_
#define _SWARM_RANGING_H_
#include "adhocdeck.h"
#include "ranging_struct.h"

/* Function Switch */
#define ENABLE_BUS_BOARDING_SCHEME

/* Queue Constants */
#define RANGING_RX_QUEUE_SIZE 10
#define RANGING_RX_QUEUE_ITEM_SIZE sizeof(Ranging_Message_With_Timestamp_t)

/* Ranging Constants */
#define RANGING_INTERVAL_MIN 20  // default 20
#define RANGING_INTERVAL_MAX 500 // default 500
#define Tf_BUFFER_POOL_SIZE (4 * RANGING_INTERVAL_MAX / RANGING_INTERVAL_MIN)
#define TX_PERIOD_IN_MS 50
#define TX_RV_INTERVAL_HISTORY_SIZE 5
#define ZERO_STAGE 124  // 飞行阶段，0阶段随机飞
#define FIRST_STAGE 125 //
#define SECOND_STAGE 126
#define LAND_STAGE 127

/*--2添加--*/
typedef struct
{
    int16_t distance_history[3];
    uint8_t index_inserting;
} median_data_t;

typedef struct
{
    uint16_t interval[TX_RV_INTERVAL_HISTORY_SIZE]; // 近似两次数据包的发送间隔.存5次历史值
    uint8_t latest_data_index;                      // 存储最新数据的index;
} tx_rv_interval_history_t;

typedef struct
{
    uint16_t address;
    int8_t stage;
    bool keepFlying;
    uint32_t keepFlyingTrueTick;
    bool alreadyTakeoff;

} leaderStateInfo_t;

typedef struct
{
    uint16_t distanceTowards[RANGING_TABLE_SIZE + 1]; // cm
    short velocityXInWorld[RANGING_TABLE_SIZE + 1];   // 2byte m/s 在世界坐标系下的速度（不是机体坐标系）
    short velocityYInWorld[RANGING_TABLE_SIZE + 1];   // 2byte cm/s 在世界坐标系下的速度（不是机体坐标系）
    float gyroZ[RANGING_TABLE_SIZE + 1];              // 4 byte rad/s
    uint16_t positionZ[RANGING_TABLE_SIZE + 1];       // 2 byte cm/s
    bool refresh[RANGING_TABLE_SIZE + 1];             // 当前信息从上次EKF获取，到现在是否更新
    bool isNewAdd[RANGING_TABLE_SIZE + 1];            // 这个邻居是否是新加入的
    bool isNewAddUsed[RANGING_TABLE_SIZE + 1];
    /* 用于辅助判断这个邻居是否是新加入的（注意：这里的'新加入'指的是，是相对于EKF来说的，主要用于在EKF中判断是否需要执行初始化工作）*/
} neighborStateInfo_t; /*存储正在和本无人机进行通信的邻居的所有信息（用于EKF）*/

typedef struct
{
    address_t address[RANGING_TABLE_SIZE + 1];
    int size;
} currentNeighborAddressInfo_t; /*当前正在和本无人机进行通信的邻居地址信息*/

/*--2添加--*/

/* Ranging Operations */
void rangingInit();
int16_t computeDistance(uint16_t neighborAddress, Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp,
                        Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr,
                        Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf);
void processRangingMessage(Ranging_Message_With_Timestamp_t *rangingMessageWithTimestamp);
int generateRangingMessage(Ranging_Message_t *rangingMessage);
int16_t getDistance(uint16_t neighborAddress);
void setDistance(uint16_t neighborAddress, int16_t distance);

/*--3添加--*/

/*设置当前无人机已经起飞*/
void setMyTakeoff(bool isAlreadyTakeoff);

/*获取leader的阶段信息*/
int8_t getLeaderStage();

/*初始化leader状态信息*/
void initLeaderStateInfo();

/*set邻居的状态信息*/
void setNeighborStateInfo(uint16_t neighborAddress, int16_t distance, Ranging_Message_Header_t *rangingMessageHeader);

/*set邻居是否是新加入的*/
void setNeighborStateInfo_isNewAdd(uint16_t neighborAddress, bool isNewAddNeighbor);

/*get邻居的状态信息*/
bool getNeighborStateInfo(uint16_t neighborAddress, uint16_t *distance, short *vx, short *vy, float *gyroZ, uint16_t *height, bool *isNewAddNeighbor);

/*getOrSetKeepflying*/
bool getOrSetKeepflying(uint16_t RobIDfromControl, bool keep_flying);

/*get正在和本无人机进行通信的邻居地址信息，供外部调用*/
void getCurrentNeighborAddressInfo_t(currentNeighborAddressInfo_t *currentNeighborAddressInfo);

/*--3添加--*/
#endif