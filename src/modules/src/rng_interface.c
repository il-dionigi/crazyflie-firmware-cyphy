#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <types.h>

/* STM32F4xx includes */
#include "stm32fxxx.h"

/* FreeRtos includes */
#include "FreeRTOS.h"

/*
 * no need for OS_SEED to be used, as this is not a windows aplication
 *
 * init()
 * RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_TNG, ENABLE)
 * RNG_Cmd(ENABLE)
 *
 * get rng funct
 * {
 * while RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET
 * 		busy wait (see if threading is available)
 * random32bitstorage = RNG_GetRandomNumber();
 *
 * }
 *
 *
 * */

void rngInit();
int getRngByte(byte* output, word32 sz);
bool RngIsInit();
//static uint32_t randomNumber = 0;

static bool isInit = false;

void rngInit(){

	if(isInit)
		return;


	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
	RNG_Cmd(ENABLE);

	isInit = true;
}
//I DONT KNOW HOW TO DO THIS ONE WHELELELELEP
int getRngByte(byte *output, word32 sz){
	/*
	if(sz <= 0 || sz > 4){
	  return -1;
	}

	while(RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET)
	{
	}
	randomNumber = RNG_GetRandomNumber();

	for(int i = 0; i < sz; i++){
		output[i] = memcpy(randomNumber, sz);//this doesnt work at all
		randomNumber = randomNumber << 8;
	}
*/
	return 0;
}

bool RngIsInit(){
	return isInit;
}
