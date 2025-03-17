#define DEBUG_MODULE "SNIFFER"

#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "system.h"
#include "adhocdeck.h"
#include "usb.h"
#include "debug.h"
#include "uwb_print.h"

static TaskHandle_t uwbPrintTaskHandle = 0;
static QueueHandle_t rxQueue;

void uwbPrintRxCallback(void *parameters) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  UWB_Packet_t *packet = (UWB_Packet_t *) parameters;

  dwTime_t rxTime = {0};
  dwt_readrxtimestamp((uint8_t *) &rxTime.raw);
  UWB_Packet_With_Timestamp_t uwbPacketWithTimestamp;
  uwbPacketWithTimestamp.rxTime = rxTime;
  uwbPacketWithTimestamp.uwbPacket = *packet;

  xQueueSendFromISR(rxQueue, &uwbPacketWithTimestamp, &xHigherPriorityTaskWoken);
}

static void uwbPrintTask(void *parameters) {
  systemWaitStart();
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  UWB_Packet_With_Timestamp_t rxPacketCache;
  memset(&rxPacketCache, 0, sizeof(rxPacketCache));
  rxPacketCache.uwbPacket.header.length = FRAME_LEN_MAX;

  Sniffer_Meta_t snifferMetaCache = {
      .magic = 0xBB88,
      .senderAddress = 0,
      .seqNumber = 0,
      .msgLength = 0,
      .rxTime = 0
  };

  while (1) {
    if (xQueueReceive(rxQueue, &rxPacketCache, 1000)) {
      snifferMetaCache.senderAddress = rxPacketCache.uwbPacket.header.srcAddress;
      snifferMetaCache.seqNumber = rxPacketCache.uwbPacket.header.seqNumber;
      snifferMetaCache.msgLength = rxPacketCache.uwbPacket.header.length - sizeof(Packet_Header_t);
      snifferMetaCache.rxTime = rxPacketCache.rxTime.full;

      usbSendData(sizeof(Sniffer_Meta_t), snifferMetaCache.raw);

      uint16_t msgSize = rxPacketCache.uwbPacket.header.length - sizeof(Packet_Header_t);
      uint8_t *pointer = rxPacketCache.uwbPacket.payload;
      int remain = msgSize;
      while (remain > 0) {
        int sizeToSend = remain > USB_RX_TX_PACKET_SIZE ? USB_RX_TX_PACKET_SIZE : remain;
        usbSendData(sizeToSend, pointer);
        pointer += sizeToSend;
        remain -= sizeToSend;
      }
    }
    DEBUG_PRINT("rxenable");
    dwt_forcetrxoff();
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    vTaskDelay(1); // TODO pick proper timespan
  }
}

void uwbPrintInit() {
  rxQueue = xQueueCreate(SNIFFER_RX_QUEUE_SIZE, SNIFFER_RX_QUEUE_ITEM_SIZE);

  UWB_Message_Listener_t listener;
  listener.type = PRINT;
  listener.rxQueue = NULL;
  listener.rxCb = uwbPrintRxCallback;
  listener.txCb = NULL;
  uwbRegisterListener(&listener);

  xTaskCreate(uwbPrintTask, "ADHOC_DECK_SNIFFER_TASK_NAME", 5 * configMINIMAL_STACK_SIZE+100, NULL,
              ADHOC_DECK_TASK_PRI, &uwbPrintTaskHandle);
}

