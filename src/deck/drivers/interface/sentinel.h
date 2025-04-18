#ifndef SENTINEL_H
#define SENTINEL_H
#include "adhocdeck.h"
#include "semphr.h"

/* Queue Constants */
#define SNIFFER_SWARMRANGING_RX_QUEUE_SIZE 10   // 这个是接收rangingMessage的消息队列
#define SNIFFER_SWARMRANGING_RX_QUEUE_ITEM_SIZE sizeof(UWB_Packet_With_Timestamp_t)

#define SENTINEl_RX_QUEUE_SIZE 5 // 这个是用于sentinel之间进去通信的队列

#define DOWN_AFTER_MILLISECOND 500
#define SNIFFER_SWARMRANGING_SIZE_MAX 25

typedef enum {
  IS_MASTER_DOWN_BY_ADDRESS,               // 看看当前master是否已经宕机了
  IS_MASTER_DOWN_BY_ADDRESS_REPLY,         // 看看当前master是否已经宕机了的回复
} SENTINEL_COMMAND_TYPE;

typedef struct 
{
    SENTINEL_COMMAND_TYPE type;
    UWB_Address_t address;  // 被sentinel判断为主观下线的master的地址
    uint16_t currentEpoch; // 当前的配置epoch,用于选举leader
    UWB_Address_t runAddress;     // 发消息者的地址，或者空
}Is_Master_Down_By_Address_Message_t;

typedef struct 
{
    SENTINEL_COMMAND_TYPE type;
    uint8_t downState;      // 返回哨兵对master服务器的检查结果, 1代表master下线，0代表master未下线
    UWB_Address_t leaderRunAddress;
    uint16_t leaderEpoch;
}Is_Master_Down_By_Address_Reply_Message_t;

typedef struct Sentinel_Message_t
{

}Sentinel_Message_t;

typedef struct {
  int size;
  SemaphoreHandle_t mu;
  uint32_t lastRecvNeighborTick[SNIFFER_SWARMRANGING_SIZE_MAX];
  uint8_t isValid[SNIFFER_SWARMRANGING_SIZE_MAX];   // 表示是否有效，从而可以判断是否新旧节点
  int16_t prev[SNIFFER_SWARMRANGING_SIZE_MAX];      // 表示向前指针
  int16_t next[SNIFFER_SWARMRANGING_SIZE_MAX];      // 表示向后指针
  int16_t head;
} Neighbor_State_Table_Set_t;

// 既要实现高效的插入，还要有高效的删除

// 维护一个数据结构，表示当前的集群的状态
typedef struct Sentinel_Node_t
{
    UWB_Address_t currLeader;   // 当前leader
    uint16_t currentTerm;   // 当前任期
    UWB_Address_t voteFor;  // 给谁投票了
    Neighbor_State_Table_Set_t neighborStateSet;    // 当前邻居的状态集合
}Sentinel_Node_t;



// 哨兵节点的地址都是事先配置好的

// 需要一个数据结构存储当前leader信息
// 维护每一个无人机的最新收到报文的时间，相当于心跳
// 定期检查是否过期
// 如果是follower过期直接从邻居表中清除掉
// 如果是leader主观下线，则向其他哨兵节点发送消息，请求判断，如果大于1/2投票，则主观下线



/* 主观下线后

1. 哨兵之间进行选举，选举出一个leader
    - 如何选举（是否上面成功判断主观下线的人就可以）
2. leader选举一个节点作为master（随机挑一个）
    - 如何选举
3. 通知定位集群当前master节点，（任期，三个节点要一致？）
    - 发布消息
4. 需要其他节点下次发报文均带上当前master节点，如果均带上，则故障切换完成，否则重新广播

*/ 

void initNeighborStateTableSet(Neighbor_State_Table_Set_t *set);
void neighborStateTableSetUpdate(Neighbor_State_Table_Set_t *set, UWB_Address_t neighborAddress, uint32_t tick);
void neighborStateTableSetRemove(Neighbor_State_Table_Set_t *set, UWB_Address_t neighborAddress);
#endif