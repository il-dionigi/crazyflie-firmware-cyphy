#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <types.h>

/* STM32F4xx includes */
//#include "stm32fxxx.h"

/* FreeRtos includes */
#include "FreeRTOS.h"

void rngInit();
int getRng(byte* output, word32 sz);
bool RngIsInit();
