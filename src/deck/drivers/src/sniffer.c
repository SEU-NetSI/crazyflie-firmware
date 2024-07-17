#define DEBUG_MODULE "SNIFFER"

#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "system.h"
#include "adhocdeck.h"
#include "swarm_ranging.h"
#include "usb.h"
#include "debug.h"
#include "sniffer.h"

static TaskHandle_t snifferTaskHandle = 0;
static QueueHandle_t rxQueue;

void snifferRxCallback(void *parameters) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  
  UWB_Packet_t *packet = (UWB_Packet_t *) parameters;

  dwTime_t rxTime = {0};
  dwt_readrxtimestamp((uint8_t *) &rxTime.raw);
  UWB_Packet_With_Timestamp_t uwbPacketWithTimestamp;
  uwbPacketWithTimestamp.rxTime = rxTime;
  uwbPacketWithTimestamp.uwbPacket = *packet;
  
  xQueueSendFromISR(rxQueue, &uwbPacketWithTimestamp, &xHigherPriorityTaskWoken);
}

static void snifferTask(void *parameters) {
  systemWaitStart();
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  UWB_Packet_With_Timestamp_t rxPacketCache;
  memset(&rxPacketCache, 0, sizeof(rxPacketCache));
  rxPacketCache.uwbPacket.header.length = UWB_FRAME_LEN_MAX;

  Sniffer_Meta_t snifferMetaCache = {
      .magic = 0xBB88,
      .senderAddress = 0,
      .seqNumber = 0,
      .msgLength = 0,
      .rxTime = 0
  };

  while (1) {
    if (xQueueReceive(rxQueue, &rxPacketCache, 1000)) {
      Ranging_Message_t *rangingMessage = (Ranging_Message_t *)rxPacketCache.uwbPacket.payload;
      snifferMetaCache.senderAddress = rangingMessage->header.srcAddress;
      snifferMetaCache.seqNumber = rangingMessage->header.msgSequence;
      snifferMetaCache.msgLength = rxPacketCache.uwbPacket.header.length -sizeof(UWB_Packet_Header_t);
      snifferMetaCache.rxTime = rxPacketCache.rxTime.full;

      usbSendData(sizeof(Sniffer_Meta_t), snifferMetaCache.raw);

      uint16_t msgSize = rxPacketCache.uwbPacket.header.length - sizeof(UWB_Packet_Header_t);
      uint8_t *pointer = rxPacketCache.uwbPacket.payload;
      int remain = msgSize;
      while (remain > 0) {
        int sizeToSend = remain > USB_RX_TX_PACKET_SIZE ? USB_RX_TX_PACKET_SIZE : remain;
        usbSendData(sizeToSend, pointer);
        pointer += sizeToSend;
        remain -= sizeToSend;
      }
    }
    vTaskDelay(1); // TODO pick proper timespan
  }
}

void snifferInit() {
  rxQueue = xQueueCreate(SNIFFER_RX_QUEUE_SIZE, SNIFFER_RX_QUEUE_ITEM_SIZE);

  UWB_Message_Listener_t listener;
  listener.type = UWB_SNIFFER_MESSAGE;
  listener.rxQueue = NULL;
  listener.rxCb = snifferRxCallback;
  listener.txCb = NULL;
  uwbRegisterListener(&listener);

  xTaskCreate(snifferTask, "ADHOC_DECK_SNIFFER_TASK_NAME", 4 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &snifferTaskHandle);
}

