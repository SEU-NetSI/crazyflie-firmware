#ifndef __ROUTING_H__
#define __ROUTING_H__

#include "stdint.h"
#include "semphr.h"
#include "adhocdeck.h"

//#define ROUTING_DEBUG_ENABLE
//#define ROUTING_AODV_ENABLE
// #define ROUTING_OLSR_ENABLE

/* Queue Constants */
#define ROUTING_RX_QUEUE_SIZE 5
#define ROUTING_RX_QUEUE_ITEM_SIZE sizeof (UWB_Packet_t)
#define ROUTING_TX_QUEUE_SIZE 5
#define ROUTING_TX_QUEUE_ITEM_SIZE sizeof (UWB_Data_Packet_t)
#define ROUTING_TX_QUEUE_WAIT_TIME 100 // default 1 seconds
#define ROUTING_TX_BUFFER_QUEUE_SIZE 10
#define ROUTING_TX_BUFFER_QUEUE_ITEM_SIZE sizeof(UWB_Data_Packet_With_Timestamp_t)
#define ROUTING_TX_BUFFER_QUEUE_ITEM_HOLD_TIME 2000 // default 2 seconds

/* Data Packet */
#define ROUTING_DATA_PACKET_SIZE_MAX UWB_PAYLOAD_SIZE_MAX
#define ROUTING_DATA_PACKET_PAYLOAD_SIZE_MAX (ROUTING_DATA_PACKET_SIZE_MAX - sizeof(UWB_Data_Packet_Header_t))

/* Routing Table */
#define ROUTING_TABLE_SIZE_MAX 32
#define ROUTING_TABLE_HOLD_TIME 5000 // default 5 seconds
#define ROUTING_TABLE_EVICT_POLICY_STALEST

/* UWB Data packet definition */
typedef enum {
  UWB_DATA_MESSAGE_RESERVED = 0,
  UWB_DATA_MESSAGE_RAFT = 1,
  UWB_DATA_MESSAGE_COMMAND = 2,
  UWB_DATA_MESSAGE_TYPE_COUNT,
} UWB_DATA_MESSAGE_TYPE;

typedef struct {
  UWB_Address_t srcAddress;
  UWB_Address_t destAddress;
  uint32_t seqNumber;
  UWB_DATA_MESSAGE_TYPE type: 6;
  uint16_t length: 10;
  uint8_t ttl;
} __attribute__((packed)) UWB_Data_Packet_Header_t;

typedef struct {
  UWB_Data_Packet_Header_t header;
  uint8_t payload[ROUTING_DATA_PACKET_PAYLOAD_SIZE_MAX];
} __attribute__((packed)) UWB_Data_Packet_t;

typedef struct {
  UWB_Data_Packet_t packet;
  Time_t evictTime;
} __attribute__((packed)) UWB_Data_Packet_With_Timestamp_t;

typedef struct {
  UWB_DATA_MESSAGE_TYPE type;
  QueueHandle_t rxQueue;
  UWBCallback rxCb;
  UWBCallback txCb;
} UWB_Data_Packet_Listener_t;

typedef enum {
  ROUTE_RESERVED,
  ROUTE_AODV,
  ROUTE_OLSR
} ROUTE_TYPE;

typedef struct {
  ROUTE_TYPE type;
  bool valid;
  UWB_Address_t destAddress;
  UWB_Address_t nextHop;
  uint8_t hopCount;
  Time_t expirationTime;
  /* AODV properties */
  uint32_t destSeqNumber;
  bool validDestSeqFlag;
  uint64_t precursors; // bit set
} Route_Entry_t;

typedef void (*routeExpirationHook)(UWB_Address_t *, int);

typedef struct Route_Expiration_Hook {
  routeExpirationHook hook;
  struct Route_Expiration_Hook_Node* next;
} Route_Expiration_Hooks_t;

typedef struct {
  uint8_t size;
  SemaphoreHandle_t mu; // mutex
  Route_Entry_t entries[ROUTING_TABLE_SIZE_MAX];
  Route_Expiration_Hooks_t expirationHooks;
} Routing_Table_t;

void routingInit();

/* Messaging Operations */
int uwbSendDataPacket(UWB_Data_Packet_t *packet);
int uwbSendDataPacketBlock(UWB_Data_Packet_t *packet);
int uwbSendDataPacketWait(UWB_Data_Packet_t *packet, int wait);
int uwbReceiveDataPacket(UWB_DATA_MESSAGE_TYPE type, UWB_Data_Packet_t *packet);
int uwbReceiveDataPacketBlock(UWB_DATA_MESSAGE_TYPE type, UWB_Data_Packet_t *packet);
int uwbReceiveDataPacketWait(UWB_DATA_MESSAGE_TYPE type, UWB_Data_Packet_t *packet, int wait);
void uwbRegisterDataPacketListener(UWB_Data_Packet_Listener_t *listener);

/* Routing Table Operations */
Route_Entry_t emptyRouteEntry();
Routing_Table_t* getGlobalRoutingTable();
void routingTableInit(Routing_Table_t *table);
void routingTableAddEntry(Routing_Table_t *table, Route_Entry_t entry);
void routingTableUpdateEntry(Routing_Table_t *table, Route_Entry_t entry);
void routingTableRemoveEntry(Routing_Table_t *table, UWB_Address_t destAddress);
Route_Entry_t routingTableFindEntry(Routing_Table_t *table, UWB_Address_t destAddress);
int routingTableSearchEntry(Routing_Table_t *table, UWB_Address_t targetAddress);
void routingTableSort(Routing_Table_t *table);
int routingTableClearExpire(Routing_Table_t *table);
void routingTableRegisterExpirationHook(Routing_Table_t *table, routeExpirationHook hook);
void routeExpirationHooksInvoke(Route_Expiration_Hooks_t *hooks, UWB_Address_t *addresses, int count);

/* Debug Operations */
void printRouteEntry(Route_Entry_t *entry);
void printRoutingTable(Routing_Table_t *table);
#endif