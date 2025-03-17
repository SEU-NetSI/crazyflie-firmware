#ifndef _ADHOCDECK_UWB_PRINTF_H_
#define _ADHOCDECK_UWB_PRINTF_H_

#include "adhocdeck.h"
#include <stdint.h> 
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
typedef int (*putc_t)(int c);

int evUwbprintf(putc_t putcf, const char * fmt, va_list ap);
int uwbprintf(putc_t putcf, const char * fmt, ...);
int uwbPutchar(int ch);

#define UWB_DEBUG_PRINTF(fmt, ...) consolePrintf(fmt, ##__VA_ARGS__)
#define consolePrintf(FMT, ...) uwbprintf(uwbPutchar, FMT, ## __VA_ARGS__)
void initUWBDebugPrint(void);
#endif
