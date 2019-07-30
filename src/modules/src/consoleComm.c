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
//consoleComm.c - Used to send drone data to console
#include <stdbool.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "led.h"
#include "motors.h"


#include "beaconComm.h"
#include "consoleComm.h"
#include "lpsTwrTag.h"
#include "radiolink.h"

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
static int currBufferLen = 0;
static char droneData[1024];
static xSemaphoreHandle consoleLock;

static const char fullMsg[] = "<F>\n";
static bool isInit;

static char radioAddress[16] = "XXXXXXXXXXXXXXX\0";
static char radioChannel[3] = "XX\0";
static char radioDatarate[2] = "X\0";


void writeDroneData(char * str, int len) {
  if((currBufferLen + len) > 1024){
    currBufferLen = 0;
  }
	memcpy(droneData + currBufferLen, str, len);
  currBufferLen += len;
	droneData[currBufferLen] = 0;
}

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
	  /*if ( (unsigned char)ch <= 126 && (unsigned char)ch >= 32){
		  messageToPrint.data[messageToPrint.size] = (unsigned char)ch;
	  }
	  else {
		  messageToPrint.data[messageToPrint.size] = '?';
	  }*/
		  messageToPrint.data[messageToPrint.size] = (unsigned char)ch;
      messageToPrint.size++;
    }
    xSemaphoreGiveFromISR(consoleLock, &higherPriorityTaskWoken);
  }

  return ch;
}

int consoleCommPutchar(int ch)
{
  //ledSet(LED_GREEN_R, true); // DELETE ME
	//motorsPlayTone(A6, EIGHTS);
	//motorsPlayTone(B6, EIGHTS);
	//motorsPlayTone(A6, EIGHTS);


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
      /*if ( (unsigned char)ch <= 126 && (unsigned char)ch >= 32){
    	  messageToPrint.data[messageToPrint.size] = (unsigned char)ch;
      }
      else {
    	  messageToPrint.data[messageToPrint.size] = '?';
      }*/
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
  droneData[0] = 0;
  droneData[9] = 0;
  messageToPrint.size = 0;
  messageToPrint.header = CRTP_HEADER(CRTP_PORT_CONSOLE, 0);
  vSemaphoreCreateBinary(consoleLock);
  xTaskCreate(consoleCommTask, CONSOLE_COMM_TASK_NAME,
  			CONSOLE_COMM_TASK_STACKSIZE, NULL, CONSOLE_COMM_TASK_PRI, NULL);
  isInit = true;
  consoleCommPflush("console comm init!");
  if (configGENERATE_RUN_TIME_STATS == 1){
	  consoleCommPflush("cGRTS: True");
  }
  else{
	  consoleCommPflush("cGRTS: False");
  }
consoleCommPflush("Pname: F");
consoleCommPflush(P_NAME);
int numTasks = uxTaskGetNumberOfTasks();


}


//SEND CODE ABOVE. RECEIVE CODE BELOW
void consoleCommTask(void * prm)
{
	crtpInitTaskQueue(CRTP_PORT_CONSOLE);
	// uint64_t address = 0;

	// uint8_t channel = 0;
	// uint8_t dataRate = 0;
	// uint8_t slept = 0;
	char temp;
	while(1) {
		crtpReceivePacketBlock(CRTP_PORT_CONSOLE, &messageReceived);
		consoleCommPuts("got msg from pc; data/channel:");
		consoleCommPflush((char*)(messageReceived.data));
		temp = messageReceived.channel + '0';
		consoleCommPutchar(temp);
		consoleCommFlush();
		if (messageReceived.data[0] == '%' && messageReceived.data[1] == 'T' && messageReceived.data[2] == 'S'){
			changeTDMAslot(messageReceived.data[3]);
		}
		else if (messageReceived.data[0] == '%' && messageReceived.data[1] == 'O'){
			changeOrder( (messageReceived.data[2] == 'F')  );
		}
	} //while
}

void saveRadioAddress(uint64_t address) {
  int pos = 0;
  while (address != 0) {
    radioAddress[pos] = (address % 10) + '0';
    address /= 10;
    pos++;
  }
  radioAddress[pos] = '\0';
}

void saveRadioChannel(uint8_t channel) {
  radioChannel[0] = (channel / 10) + '0';
  radioChannel[1] = (channel % 10) + '0';
}

void saveRadioDatarate(uint8_t datarate) {
  radioDatarate[0] = datarate + '0';
}

void displayRadioAddress(void) {
  consoleCommPflush("Radio address: ");
  consoleCommPflush(radioAddress);
}

void displayRadioChannel(void) {
  consoleCommPflush("Radio channel: ");
  consoleCommPflush(radioChannel);
}

void displayRadioDatarate(void) {
  consoleCommPflush("Radio datarate: ");
  consoleCommPflush(radioDatarate);
}
