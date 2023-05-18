#pragma once

#ifdef DEBUG
#include "stdio.h"
#include "systick.h"
#define PRINT(x, ...) printf(x, ##__VA_ARGS__)
#define DEBUG_MSG(x, ...) PRINT("[%ld.%03ld] "x"\r\n", SysTick_Ms / 1000, SysTick_Ms % 1000, ##__VA_ARGS__)
#else
#define PRINT(x, ...)
#define DEBUG_MSG(x, ...)
#endif

void debug_UARTInit();