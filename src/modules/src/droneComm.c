/*
 * droneComm.c
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
 *
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
//droneComm.c - Used to send drone data to drone(?) Currently send to client
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "beaconComm.h"

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

static void droneCommTask(void * prm);

static CRTPPacket messageReceived;
CRTPPacket messageToPrint;
static xSemaphoreHandle droneLock;

static const char fullMsg[] = "<F>\n";
static bool isInit;

/**
 * Send the data to the client
 * returns TRUE if successful otherwise FALSE
 */
static bool droneCommSendMessage(void)
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


bool droneCommTest(void)
{
  return isInit;
}

int droneCommPutcharFromISR(int ch) {
  BaseType_t higherPriorityTaskWoken;

  if (xSemaphoreTakeFromISR(droneLock, &higherPriorityTaskWoken) == pdTRUE) {
    if (messageToPrint.size < CRTP_MAX_DATA_SIZE)
    {
      messageToPrint.data[messageToPrint.size] = (unsigned char)ch;
      messageToPrint.size++;
    }
    xSemaphoreGiveFromISR(droneLock, &higherPriorityTaskWoken);
  }

  return ch;
}

int droneCommPutchar(int ch)
{
  int i;
  bool isInInterrupt = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;

  if (!isInit) {
    return 0;
  }

  if (isInInterrupt) {
    return droneCommPutcharFromISR(ch);
  }

  if (xSemaphoreTake(droneLock, portMAX_DELAY) == pdTRUE)
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
      droneCommSendMessage();
    }
    xSemaphoreGive(droneLock);
  }

  return (unsigned char)ch;
}

int droneCommPuts(char *str)
{
  int ret = 0;

  while(*str)
    ret |= droneCommPutchar(*str++);

  return ret;
}

void droneCommFlush(void)
{
  if (xSemaphoreTake(droneLock, portMAX_DELAY) == pdTRUE)
  {
    droneCommSendMessage();
    xSemaphoreGive(droneLock);
  }
}

void droneCommPflush(char * str)
{
	droneCommPuts(str);
	droneCommFlush();
}

void droneCommInit()
{
  if (isInit)
    return;

  messageToPrint.size = 0;
  messageToPrint.header = CRTP_HEADER(CRTP_PORT_DRONE, 0);
  vSemaphoreCreateBinary(droneLock);
  xTaskCreate(droneCommTask, DRONE_COMM_TASK_NAME,
  			DRONE_COMM_TASK_STACKSIZE, NULL, DRONE_COMM_TASK_PRI, NULL);
  isInit = true;
  droneCommPflush("drone comm init!");
}


//SEND CODE ABOVE. RECEIVE CODE BELOW

void droneCommTask(void * prm)
{
	crtpInitTaskQueue(CRTP_PORT_DRONE);

	while(1) {
		crtpReceivePacketBlock(CRTP_PORT_DRONE, &messageReceived);

		if (messageReceived.channel==0){
			// currently this is data from the PC
			//as a test, send this data back
		  droneCommFlush();
		  droneCommPuts("CYPHY");
		  //messsageReceived.data is uint8_t, want to typecast to char.
		  droneCommPflush((char*)(messageReceived.data));
		  if (messageReceived.data[0] == '~'){
			  droneCommPflush("Got ~, send to beacon");
			  uint8_t newID = (messageReceived.data[1]) - 48;//48 is '0' in ascii. 48+x is 'x'
			  beaconCommChangeID(newID);
			  beaconCommPflush("Test");
		  }
		}
	}
}
