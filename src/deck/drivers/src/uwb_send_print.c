#define DEBUG_MODULE "SNIFFER"

#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "system.h"
#include "adhocdeck.h"
#include "usb.h"
#include "debug.h"
#include "timers.h"
#include "uwb_send_print.h"
#include "semphr.h"
#include <stdint.h> 
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>

static TaskHandle_t uwbPrintTaskHandle = 0;
static QueueHandle_t rxQueue;

static bool SendingisPending = 0;
#define UWB_PACKET_NUM 2
int uwb_debug_print_init = 0;
// 静态初始化数组，每个元素赋默认值
static UWB_Packet_t uwbPackets[UWB_PACKET_NUM] = {};
static uint8_t uwbPacketsIsSend[UWB_PACKET_NUM] = {};
static int uwbPacketsWriteIndex = 0;
static int uwbPacketsReadIndex = 0;
static int uwbPacketsSize = 0;
SemaphoreHandle_t uwbPacketsMu;

int len = 0;
static const char digit[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 
                             'A', 'B', 'C', 'D', 'E', 'F'};

#define DEBUG_PRINT_FREQUENCY_TICK 500

static int getIntLen (long int value)
{
  int l = 1;
  while(value > 9)
  {
    l++;
    value /= 10;
  }
  return l;
}

static int power(int a, int b)
{
  int i;
  int x = a;

  for (i = 1; i < b; i++)
  {
    x *= a;
  }

  return x;
}

static int itoa10Unsigned(putc_t putcf, unsigned long long int num)
{
  int len = 0;

  if (num == 0)
  {
    putcf('0');
    return 1;
  }

  unsigned long long int i = 1;

  while ((num / i) > 9)
  {
    i *= 10L;
  }

  do
  {
    putcf(digit[(num / i) % 10L]);
    len++;
  }
  while (i /= 10L);

  return len;
}

static int itoa10(putc_t putcf, long long int num, int precision)
{
  int len = 0;

  if (num == 0)
  {
    putcf('0');
    return 1;
  }

  long long unsigned int n = num;
  if (num < 0)
  {
    n = -num;
    putcf('-');
    len++;
  }

  int numLenght = getIntLen(num);
  if (numLenght < precision)
  {
    int fillWithZero = precision - numLenght;
    while (fillWithZero > 0)
    {
      putcf('0');
      len++;
      fillWithZero--;
    }
  }

  return itoa10Unsigned(putcf, n) + len;
}

static int itoa16(putc_t putcf, uint64_t num, int width, char padChar)
{
  int len = 0;
  bool foundFirst = false;

  for (int i = 15; i >= 0; i--)
  {
    int shift = i * 4;
    uint64_t mask = (uint64_t)0x0F << shift;
    uint64_t val = (num & mask) >> shift;

    if (val > 0)
    {
      foundFirst = true;
    }

    if (foundFirst || i < width)
    {
      if (foundFirst)
      {
        putcf(digit[val]);
      }
      else
      {
        putcf(padChar);
      }

      len++;
    }
  }

  return len;
}

static int handleLongLong(putc_t putcf, const char** fmt, unsigned long long int val, int width, char padChar)
{
  int len = 0;

  switch(*((*fmt)++))
  {
    case 'i':
    case 'd':
      len = itoa10(putcf, (long long int)val, 0);
      break;
    case 'u':
      len = itoa10Unsigned(putcf, val);
      break;
    case 'x':
    case 'X':
      len = itoa16(putcf, val, width, padChar);
      break;
    default:
      // Nothing here
      break;
  }

  return len;
}

static int handleLong(putc_t putcf, const char** fmt, unsigned long int val, int width, char padChar)
{
  int len = 0;

  switch(*((*fmt)++))
  {
    case 'i':
    case 'd':
      len = itoa10(putcf, (long int)val, 0);
      break;
    case 'u':
      len = itoa10Unsigned(putcf, val);
      break;
    case 'x':
    case 'X':
      len = itoa16(putcf, val, width, padChar);
      break;
    default:
      // Nothing here
      break;
  }

  return len;
}

int evUwbprintf(putc_t putcf, const char * fmt, va_list ap)
{
  int len=0;
  float num;
  char* str;
  int precision;
  int width;
  char padChar;

  while (*fmt)
  {
    if (*fmt == '%')
    {
      precision = 6;
      padChar = ' ';
      width = 0;

      fmt++;
      if (*fmt == '%') {
        putcf(*fmt++);
        len++;
        continue;
      }

      while ('0' == *fmt)
      {
        padChar = '0';
        fmt++;
      }

			while(isdigit((unsigned)*fmt))
			{
				width *= 10;
				width += *fmt - '0';
				fmt++;
			}

      while (!isalpha((unsigned) *fmt))
      {
        if (*fmt == '.')
        {
          fmt++;
          if (isdigit((unsigned)*fmt))
          {
            precision = *fmt - '0';
            fmt++;
          }
        }
      }
      switch (*fmt++)
      {
        case 'i':
        case 'd':
          len += itoa10(putcf, va_arg(ap, int), 0);
          break;
        case 'u':
          len += itoa10Unsigned(putcf, va_arg(ap, unsigned int));
          break;
        case 'x':
        case 'X':
          len += itoa16(putcf, va_arg(ap, unsigned int), width, padChar);
          break;
        case 'l':
          // Look ahead for ll
          if (*fmt == 'l') {
            fmt++;
            len += handleLongLong(putcf, &fmt, va_arg(ap, unsigned long long int), width, padChar);
          } else {
            len += handleLong(putcf, &fmt, va_arg(ap, unsigned long int), width, padChar);
          }

          break;
        case 'f':
          num = va_arg(ap, double);
          if(num<0)
          {
            putcf('-');
            num = -num;
            len++;
          }
          len += itoa10(putcf, (int)num, 0);
          putcf('.'); len++;
          len += itoa10(putcf, (num - (int)num) * power(10,precision), precision);
          break;

        case 's':
          str = va_arg(ap, char* );
          while(*str)
          {
            putcf(*str++);
            len++;
          }
          break;
        case 'c':
          putcf((char)va_arg(ap, int));
          len++;
          break;
        default:
          break;
      }
    }
    else
    {
      putcf(*fmt++);
      len++;
    }
  }
  
  return len;
}

int uwbprintf(putc_t putcf, const char * fmt, ...)
{
  va_list ap;
  int len;

  va_start(ap, fmt);
  len = evUwbprintf(putcf, fmt, ap);
  va_end(ap);

  return len;
}

int uwbPutchar(int ch)
{
  // return ch;
  xSemaphoreTake(uwbPacketsMu, portMAX_DELAY);
  if(len < PAYLOAD_SIZE)
  {
    uwbPackets[uwbPacketsWriteIndex].payload[len] = (uint8_t)ch;
    uwbPackets[uwbPacketsWriteIndex].header.length += 1;
    uwbPacketsIsSend[uwbPacketsWriteIndex] = 0;
    len++;
  }
  // DEBUG_PRINT("len:%d\n",len);
  if(/*ch == '\n' ||*/ len >= PAYLOAD_SIZE-100)
  {
    // 发送
    uwbPackets[uwbPacketsWriteIndex].header.type = PRINT;
    uwbSendPacketBlock(&uwbPackets[uwbPacketsWriteIndex]);
    uwbPacketsIsSend[uwbPacketsWriteIndex] = 1;
    // 初始化下一个
    uwbPacketsWriteIndex = (uwbPacketsWriteIndex + 1) % UWB_PACKET_NUM;
    uwbPackets[uwbPacketsWriteIndex].header.length = sizeof(Packet_Header_t);
    len = 0;
  }
  xSemaphoreGive(uwbPacketsMu);
  return ch;
}
static void debugPrintTimerCallback(TimerHandle_t timer){
  // uwbSendPacketBlock(&uwbPackets[0]);
  // return 0;
  
  if(len != 0){
    // 这里size是已经封装好的包，如果没有封装好，则应该为+1
    xSemaphoreTake(uwbPacketsMu, portMAX_DELAY);
    for(int i = 0; i < UWB_PACKET_NUM; i++){
        if(uwbPacketsIsSend[uwbPacketsReadIndex] == 0){
            uwbSendPacketBlock(&uwbPackets[uwbPacketsReadIndex]);
            uwbPacketsIsSend[uwbPacketsReadIndex]=1;
            if(uwbPacketsReadIndex == uwbPacketsWriteIndex){
                // 重置下一个块
                uwbPacketsWriteIndex = (uwbPacketsWriteIndex + 1) % UWB_PACKET_NUM;
                uwbPackets[uwbPacketsWriteIndex].header.length = sizeof(Packet_Header_t);
                len = 0;
            }
            uwbPacketsReadIndex = (uwbPacketsReadIndex + 1) % UWB_PACKET_NUM;
            uwbPacketsSize-=1;
        }else{
            break;
        }
    }
    xSemaphoreGive(uwbPacketsMu);
  }
}

static void initDebugPrintTimer()
{
    static TimerHandle_t debugPrintTimer;
    debugPrintTimer = xTimerCreate("debug_print_timer",
                                               M2T(DEBUG_PRINT_FREQUENCY_TICK),
                                               pdTRUE,
                                               (void *)0,
                                               debugPrintTimerCallback);
    if (debugPrintTimer != NULL)
    {
        xTimerStart(debugPrintTimer, M2T(0));
    }else{

    }
}

void initUWBDebugPrint(void) {
    for (int i = 0; i < UWB_PACKET_NUM; i++) {
        uwbPackets[i].header.type = PRINT;  // 设置 header.type 字段为 PRINT
        uwbPackets[i].header.length = sizeof(Packet_Header_t);
        uwbPacketsIsSend[i] = 1;
    }
    uwbPacketsWriteIndex = 0;
    uwbPacketsWriteIndex = 0;
    uwbPacketsReadIndex = 0;
    uwbPacketsSize = 0;
    uwbPacketsMu = xSemaphoreCreateMutex();
    initDebugPrintTimer();
}