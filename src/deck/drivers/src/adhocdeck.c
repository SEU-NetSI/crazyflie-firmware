#define DEBUG_MODULE "DWM3k"

#include <stdint.h>
#include <string.h>

#include "stm32fxxx.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "autoconf.h"
#include "debug.h"
#include "deck.h"
#include "estimator.h"
#include "log.h"
#include "param.h"
#include "system.h"

#include "adhocdeck.h"
#include "dwTypes.h"
#include "libdw3000.h"
#include "dw3000.h"
#include "swarm_ranging.h"
#include "flooding.h"
#include "routing.h"
#include "raft.h"

#ifndef UWB_DEBUG_ENABLE
  #undef DEBUG_PRINT
  #define DEBUG_PRINT
#endif

#define CS_PIN DECK_GPIO_IO1

// LOCO deck alternative IRQ and RESET pins(IO_2, IO_4) instead of default (RX1, TX1), leaving UART1 free for use
#ifdef CONFIG_DECK_ADHOCDECK_USE_ALT_PINS
  #define GPIO_PIN_IRQ      DECK_GPIO_IO2
  #ifndef ADHOCDECK_ALT_PIN_RESET
    #define GPIO_PIN_RESET    DECK_GPIO_IO4
  #else
    #define GPIO_PIN_RESET 	ADHOCDECK_ALT_PIN_RESET
  #endif
  #define EXTI_PortSource EXTI_PortSourceGPIOB
  #define EXTI_PinSource    EXTI_PinSource5
  #define EXTI_LineN          EXTI_Line5
#elif defined(CONFIG_DECK_ADHOCDECK_USE_UART2_PINS)
  #define GPIO_PIN_IRQ 	  DECK_GPIO_TX2
  #define GPIO_PIN_RESET 	DECK_GPIO_RX2
  #define EXTI_PortSource EXTI_PortSourceGPIOA
  #define EXTI_PinSource 	EXTI_PinSource2
  #define EXTI_LineN 		  EXTI_Line2
#else
  #define GPIO_PIN_IRQ      DECK_GPIO_RX1
  #define GPIO_PIN_RESET    DECK_GPIO_TX1
  #define EXTI_PortSource EXTI_PortSourceGPIOC
  #define EXTI_PinSource    EXTI_PinSource11
  #define EXTI_LineN          EXTI_Line11
#endif

#define DEFAULT_RX_TIMEOUT 0xFFFFF

static uint16_t MY_UWB_ADDRESS;
static bool isInit = false;
static TaskHandle_t uwbTaskHandle = 0;
static TaskHandle_t uwbTxTaskHandle = 0;
static SemaphoreHandle_t irqSemaphore;

static QueueHandle_t txQueue;
static xQueueHandle queues[UWB_MESSAGE_TYPE_COUNT];
static UWB_Message_Listener_t listeners[UWB_MESSAGE_TYPE_COUNT];

/* rx buffer used in rx_callback */
static uint8_t rxBuffer[UWB_FRAME_LEN_MAX];

static void txCallback() {
//  DEBUG_PRINT("txCallback \n");
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(uwbTxTaskHandle, &xHigherPriorityTaskWoken);
}

/* Packet dispatcher */
static void rxCallback(dwt_cb_data_t *cbData) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  uint32_t dataLength = cbData->datalength;

  ASSERT(dataLength != 0 && dataLength <= UWB_FRAME_LEN_MAX);

  dwt_readrxdata(rxBuffer, dataLength - FCS_LEN, 0); /* No need to read the FCS/CRC. */

//  DEBUG_PRINT("rxCallback: data length = %lu \n", dataLength);

  UWB_Packet_t *packet = (UWB_Packet_t *) &rxBuffer;
  UWB_MESSAGE_TYPE msgType = packet->header.type;

  ASSERT(msgType < UWB_MESSAGE_TYPE_COUNT);

  if (!(packet->header.destAddress == MY_UWB_ADDRESS || packet->header.destAddress == UWB_DEST_ANY)) {
    dwt_forcetrxoff();
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    return;
  }

  if (listeners[msgType].rxCb) {
    listeners[msgType].rxCb(packet);
  }

  if (listeners[msgType].rxQueue) {
    xQueueSendFromISR(listeners[msgType].rxQueue, packet, &xHigherPriorityTaskWoken);
  }

  // dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void rxTimeoutCallback() {
  // dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void rxErrorCallback() {
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  DEBUG_PRINT("rxErrorCallback: some error occurs when rx\n");
}

uint16_t uwbGetAddress() {
  return MY_UWB_ADDRESS;
}

int uwbSendPacket(UWB_Packet_t *packet) {
  ASSERT(packet);
  return xQueueSend(txQueue, packet, 0);
}

int uwbSendPacketBlock(UWB_Packet_t *packet) {
  ASSERT(packet);
  return xQueueSend(txQueue, packet, portMAX_DELAY);
}

int uwbSendPacketWait(UWB_Packet_t *packet, int wait) {
  ASSERT(packet);
  return xQueueSend(txQueue, packet, M2T(wait));
}

int uwbReceivePacket(UWB_MESSAGE_TYPE type, UWB_Packet_t *packet) {
  ASSERT(packet);
  ASSERT(type < UWB_MESSAGE_TYPE_COUNT);
  return xQueueReceive(queues[type], packet, 0);
}

int uwbReceivePacketBlock(UWB_MESSAGE_TYPE type, UWB_Packet_t *packet) {
  ASSERT(packet);
  ASSERT(type < UWB_MESSAGE_TYPE_COUNT);
  return xQueueReceive(queues[type], packet, portMAX_DELAY);
}

int uwbReceivePacketWait(UWB_MESSAGE_TYPE type, UWB_Packet_t *packet, int wait) {
  ASSERT(packet);
  ASSERT(type < UWB_MESSAGE_TYPE_COUNT);
  return xQueueReceive(queues[type], packet, M2T(wait));
}

void uwbRegisterListener(UWB_Message_Listener_t *listener) {
  ASSERT(listener->type < UWB_MESSAGE_TYPE_COUNT);
  queues[listener->type] = listener->rxQueue;
  listeners[listener->type] = *listener;
}

static int uwbInit() {
  /* Need to make sure DW IC is in IDLE_RC before proceeding */
  for (int i = 0; !dwt_checkidlerc() && i < 3; i++) {
  }

  if (!dwt_checkidlerc()) {
    DEBUG_PRINT("Error: DW IC is not in IDLE_RC state \n");
    return DWT_ERROR;
  }

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    return DWT_ERROR;
  }

  if (dwt_configure(&uwbPhrConfig) == DWT_ERROR) {
    return DWT_ERROR;
  }

  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  /* Configure the TX spectrum parameters (power, PG delay and PG count) */
  dwt_configuretxrf(&uwbTxConfigOptions);

  /* Configure Antenna Delay */
  dwt_setrxantennadelay(UWB_RX_ANT_DLY);
  dwt_settxantennadelay(UWB_TX_ANT_DLY);

  dwt_setrxtimeout(DEFAULT_RX_TIMEOUT);

  /* Set callback functions */
  dwt_setcallbacks(&txCallback, &rxCallback, &rxTimeoutCallback, &rxErrorCallback, NULL, NULL);

  /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors). */
  dwt_setinterrupt(SYS_ENABLE_LO_TXFRS_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXFTO_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXPTO_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXPHE_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXFCE_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXFSL_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXSTO_ENABLE_BIT_MASK,
                   0, DWT_ENABLE_INT);

  /* Clearing the SPI ready interrupt */
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RCINIT_BIT_MASK | SYS_STATUS_SPIRDY_BIT_MASK);

  irqSemaphore = xSemaphoreCreateMutex();

  return DWT_SUCCESS;
}

static void uwbTxTask(void *parameters) {
  systemWaitStart();

  UWB_Packet_t packetCache;
  while (true) {
    if (xQueueReceive(txQueue, &packetCache, portMAX_DELAY)) {
      ASSERT(packetCache.header.type < UWB_MESSAGE_TYPE_COUNT);
      ASSERT(packetCache.header.length <= UWB_FRAME_LEN_MAX);

      /* Reset DW3000 to idle state */
      dwt_forcetrxoff();
      dwt_writetxdata(packetCache.header.length, (uint8_t *) &packetCache, 0);
      dwt_writetxfctrl(packetCache.header.length + FCS_LEN, 0, 1);
      /* Start transmission. */
      if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) ==
          DWT_ERROR) {
        DEBUG_PRINT("uwbTxTask:  TX ERROR\n");
      } else {
        if (ulTaskNotifyTake(pdTRUE, M2T(20)) == pdFALSE) {
          DEBUG_PRINT("uwbTxTask: Timeout when waiting for tx success signal from txCallback, os may extremely busy now.\n");
        } else {
          /* Invoke txCallback */
          if (listeners[packetCache.header.type].txCb) {
            listeners[packetCache.header.type].txCb(&packetCache);
          }
        }
      }
      vTaskDelay(M2T(1));
    }
  }
}

static void uwbTask(void *parameters) {
  systemWaitStart();

  while (1) {
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
      do {
        xSemaphoreTake(irqSemaphore, portMAX_DELAY);
        dwt_isr();
        xSemaphoreGive(irqSemaphore);
      } while (digitalRead(GPIO_PIN_IRQ) != 0);
    }
  }
}

/************ Low level ops for libdw **********/

static uint8_t spiTxBuffer[UWB_FRAME_LEN_MAX];
static uint8_t spiRxBuffer[UWB_FRAME_LEN_MAX];
static uint16_t spiSpeed = SPI_BAUDRATE_21MHZ;

static void spiWrite(const void *header, size_t headerLength, const void *data,
                     size_t dataLength) {
  spiBeginTransaction(spiSpeed);
  digitalWrite(CS_PIN, LOW);
  memcpy(spiTxBuffer, header, headerLength);
  memcpy(spiTxBuffer + headerLength, data, dataLength);
  spiExchange(headerLength + dataLength, spiTxBuffer, spiRxBuffer);
  digitalWrite(CS_PIN, HIGH);
  spiEndTransaction();
}

static void spiRead(const void *header, size_t headerLength, void *data,
                    size_t dataLength) {
  spiBeginTransaction(spiSpeed);
  digitalWrite(CS_PIN, LOW);
  memcpy(spiTxBuffer, header, headerLength);
  memset(spiTxBuffer + headerLength, 0, dataLength);
  spiExchange(headerLength + dataLength, spiTxBuffer, spiRxBuffer);
  memcpy(data, spiRxBuffer + headerLength, dataLength);
  digitalWrite(CS_PIN, HIGH);
  spiEndTransaction();
}

#ifdef CONFIG_DECK_ADHOCDECK_USE_ALT_PINS
void __attribute__((used)) EXTI5_Callback(void)
#elif defined(CONFIG_DECK_ADHOCDECK_USE_UART2_PINS)
void __attribute__((used)) EXTI2_Callback(void)
#else
void __attribute__((used)) EXTI11_Callback(void)
#endif
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Unlock interrupt handling task
  vTaskNotifyGiveFromISR(uwbTaskHandle, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken) {
    portYIELD();
  }
}

static void spiSetSpeed(dwSpiSpeed_t speed) {
  if (speed == dwSpiSpeedLow) {
    spiSpeed = SPI_BAUDRATE_2MHZ;
  } else if (speed == dwSpiSpeedHigh) {
    spiSpeed = SPI_BAUDRATE_21MHZ;
  }
}

static void delayms(unsigned int delay) { vTaskDelay(M2T(delay)); }

static void reset(void) {
  digitalWrite(GPIO_PIN_RESET, 0);
  vTaskDelay(M2T(10));
  digitalWrite(GPIO_PIN_RESET, 1);
  vTaskDelay(M2T(10));
}

extern dwOps_t dwt_ops = {
    .spiRead = spiRead,
    .spiWrite = spiWrite,
    .spiSetSpeed = spiSetSpeed,
    .delayms = delayms,
    .reset = reset
};

static void pinInit() {
  EXTI_InitTypeDef EXTI_InitStructure;

  spiBegin();

  // Set up interrupt
  SYSCFG_EXTILineConfig(EXTI_PortSource, EXTI_PinSource);

  EXTI_InitStructure.EXTI_Line = EXTI_LineN;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
#ifdef CONFIG_DECK_ADHOCDECK_USE_ALT_PINS
  DEBUG_PRINT("USE_ALT_PINS\n");
#elif CONFIG_DECK_ADHOCDECK_USE_UART2_PINS
  DEBUG_PRINT("USE_UART2_PINS\n");
#else
  DEBUG_PRINT("USE_UART1_PINS\n");
#endif
  // Init pins
#ifdef CONFIG_DECK_ADHOCDECK_USE_UART2_PINS
  pinMode(CS_PIN, OUTPUT);
//  pinMode(GPIO_PIN_RESET, OUTPUT); TODO: magic, don't know why, need further debugging
  pinMode(GPIO_PIN_IRQ, INPUT);
#else
  pinMode(CS_PIN, OUTPUT);
  pinMode(GPIO_PIN_RESET, OUTPUT);
  pinMode(GPIO_PIN_IRQ, INPUT);
#endif
  //Reset the DW3000 chip
  dwt_ops.reset();
}

static void queueInit() {
  txQueue = xQueueCreate(UWB_TX_QUEUE_SIZE, UWB_TX_QUEUE_ITEM_SIZE);
}

static void uwbTaskInit() {
  /* Create UWB Task */
  xTaskCreate(uwbTask, ADHOC_DECK_TASK_NAME, UWB_TASK_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbTaskHandle);
  xTaskCreate(uwbTxTask, ADHOC_DECK_TX_TASK_NAME, UWB_TASK_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbTxTaskHandle);
#ifdef UWB_RANGING_ENABLE
  rangingInit();
#endif
#ifdef UWB_ROUTING_ENABLE
  routingInit();
#endif
#ifdef UWB_RAFT_ENABLE
  raftInit();
#endif
#ifdef UWB_FLOODING_ENABLE
  floodingInit();
#endif
}
/*********** Deck driver initialization ***************/
static void dwm3000Init(DeckInfo *info) {
  pinInit();
  queueInit();
  if (uwbInit() == DWT_SUCCESS) {
    uwbTaskInit();
    isInit = true;
  } else {
    isInit = false;
  }
  DEBUG_PRINT("MY_UWB_ADDRESS = %d \n", MY_UWB_ADDRESS);
}

static bool dwm3000Test() {
  if (!isInit) {
    DEBUG_PRINT("Error while initializing DWM3000\n");
  }

  return isInit;
}

static const DeckDriver dwm3000_deck = {
    .vid = 0xBC,
    .pid = 0x06,
    .name = "DWM3000",

#ifdef CONFIG_DECK_ADHOCDECK_USE_ALT_PINS
    .usedGpio = DECK_USING_IO_1 | DECK_USING_IO_2 | DECK_USING_IO_4,
#elif defined(CONFIG_DECK_ADHOCDECK_USE_UART2_PINS)
    .usedGpio = DECK_USING_IO_1 | DECK_USING_UART2,
#else
    .usedGpio = DECK_USING_IO_1 | DECK_USING_UART1,
#endif
    .usedPeriph = DECK_USING_SPI,
#ifdef ADHOCDECK_NO_LOW_INTERFERENCE
    .requiredLowInterferenceRadioMode = false,
#else
    .requiredLowInterferenceRadioMode = true,
#endif

    .init = dwm3000Init,
    .test = dwm3000Test,
};

DECK_DRIVER(dwm3000_deck);

PARAM_GROUP_START(deck)
        PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, DWM3000, &isInit)
PARAM_GROUP_STOP(deck)

PARAM_GROUP_START(ADHOC)
        PARAM_ADD_CORE(PARAM_UINT16 | PARAM_PERSISTENT, MY_UWB_ADDRESS, &MY_UWB_ADDRESS)
PARAM_GROUP_STOP(ADHOC)