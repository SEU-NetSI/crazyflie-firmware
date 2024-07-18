#ifndef _ADHOCDECK_SNIFFER_H_
#define _ADHOCDECK_SNIFFER_H_

#include "adhocdeck.h"
#include "swarm_ranging.h"

/* Queue Constants */
#define SNIFFER_RX_QUEUE_SIZE 10
#define SNIFFER_RX_QUEUE_ITEM_SIZE sizeof(UWB_Packet_With_Timestamp_t)

typedef union {
  uint8_t raw[18];
  struct {
    uint32_t magic;
    uint16_t senderAddress;
    uint16_t seqNumber;
    uint16_t msgLength;
    uint64_t rxTime;
  } __attribute__((packed));
} __attribute__((packed)) Sniffer_Meta_t;

void snifferInit();

#endif
