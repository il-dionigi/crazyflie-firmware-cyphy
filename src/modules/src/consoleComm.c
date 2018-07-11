/*
 * consoleComm.c
 *
 *  Created on: Jun 29, 2018
 *      Author: bitcraze
 */
//~CYPHY~
/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 * consoleComm
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
//consoleComm.c - Used to send drone data to console
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "beaconComm.h"
#include "lpsTwrTag.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "crtp.h"
#include "crc.h"
#include "config.h"

#ifdef STM32F40_41xxx
#include "stm32f4xx.h"
#else
#include "stm32f10x.h"
#ifndef SCB_ICSR_VECTACTIVE_Msk
#define SCB_ICSR_VECTACTIVE_Msk 0x1FFUL
#endif
#endif

static void consoleCommTask(void * prm);

static CRTPPacket messageReceived;
CRTPPacket messageToPrint;
static xSemaphoreHandle consoleLock;

static const char fullMsg[] = "<F>\n";
static bool isInit;

/**
 * Send the data to the client
 * returns TRUE if successful otherwise FALSE
 */
static bool consoleCommSendMessage(void)
{

  if (crtpSendPacket(&messageToPrint) == pdTRUE)
  {
    messageToPrint.size = 0;
  }
  else
  {
    return false;
  }

  return true;
}


bool consoleCommTest(void)
{
  return isInit;
}

int consoleCommPutcharFromISR(int ch) {
  BaseType_t higherPriorityTaskWoken;

  if (xSemaphoreTakeFromISR(consoleLock, &higherPriorityTaskWoken) == pdTRUE) {
    if (messageToPrint.size < CRTP_MAX_DATA_SIZE)
    {
      messageToPrint.data[messageToPrint.size] = (unsigned char)ch;
      messageToPrint.size++;
    }
    xSemaphoreGiveFromISR(consoleLock, &higherPriorityTaskWoken);
  }

  return ch;
}

int consoleCommPutchar(int ch)
{
  int i;
  bool isInInterrupt = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;

  if (!isInit) {
    return 0;
  }

  if (isInInterrupt) {
    return consoleCommPutcharFromISR(ch);
  }

  if (xSemaphoreTake(consoleLock, portMAX_DELAY) == pdTRUE)
  {
    if (messageToPrint.size < CRTP_MAX_DATA_SIZE)
    {
      messageToPrint.data[messageToPrint.size] = (unsigned char)ch;
      messageToPrint.size++;
    }
    if (ch == '\n' || messageToPrint.size >= CRTP_MAX_DATA_SIZE)
    {
      if (crtpGetFreeTxQueuePackets() == 1)
      {
        for (i = 0; i < sizeof(fullMsg) && (messageToPrint.size - i) > 0; i++)
        {
          messageToPrint.data[messageToPrint.size - i] =
              (uint8_t)fullMsg[sizeof(fullMsg) - i - 1];
        }
      }
      consoleCommSendMessage();
    }
    xSemaphoreGive(consoleLock);
  }

  return (unsigned char)ch;
}

int consoleCommPuts(char *str)
{
  int ret = 0;

  while(*str)
    ret |= consoleCommPutchar(*str++);

  return ret;
}

void consoleCommFlush(void)
{
  if (xSemaphoreTake(consoleLock, portMAX_DELAY) == pdTRUE)
  {
    consoleCommSendMessage();
    xSemaphoreGive(consoleLock);
  }
}

void consoleCommPflush(char * str)
{
	consoleCommPuts(str);
	consoleCommFlush();
}

void consoleCommInit()
{
  if (isInit)
    return;

  messageToPrint.size = 0;
  messageToPrint.header = CRTP_HEADER(CRTP_PORT_CONSOLE, 0);
  vSemaphoreCreateBinary(consoleLock);
  xTaskCreate(consoleCommTask, CONSOLE_COMM_TASK_NAME,
  			CONSOLE_COMM_TASK_STACKSIZE, NULL, CONSOLE_COMM_TASK_PRI, NULL);
  isInit = true;
  consoleCommPflush("console comm init!");
}


//SEND CODE ABOVE. RECEIVE CODE BELOW

void consoleCommTask(void * prm)
{
	crtpInitTaskQueue(CRTP_PORT_CONSOLE);

	while(1) {
		crtpReceivePacketBlock(CRTP_PORT_CONSOLE, &messageReceived);

		if (messageReceived.channel==0){
			// currently this is data from the PC
			//as a test, send this data back
		  consoleCommFlush();
		  consoleCommPuts("got message from pc: ");
		  //messsageReceived.data is uint8_t, want to typecast to char.
		  consoleCommPflush((char*)(messageReceived.data));
		  if (messageReceived.data[0] == '~'){
			  consoleCommPflush("1! Sending to beacon; port,message:");
			  consoleCommPutchar(messageReceived.data[1]);
			  consoleCommPutchar(',');
			  consoleCommPflush((char*)(messageReceived.data+2));
			  //uint8_t newID = (messageReceived.data[1]) - 48;//48 is '0' in ascii. 48+x is 'x'
			  //beaconCommChangeID(newID);
			  beaconCommPflush((char*)(messageReceived.data+2));
		  }
		}
	}
}
