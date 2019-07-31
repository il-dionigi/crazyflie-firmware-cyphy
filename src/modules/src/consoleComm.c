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
#include "log.h"
#include "aes.h"


#ifdef STM32F40_41xxx
#include "stm32f4xx.h"
#else
#include "stm32f10x.h"
#ifndef SCB_ICSR_VECTACTIVE_Msk
#define SCB_ICSR_VECTACTIVE_Msk 0x1FFUL
#endif
#endif

#define ENCRYPTED_CHANNEL 2
#define PLAINTEXT_CHANNEL 0


//#include "RTOS.h"
//from Wolfssl/wolfcrypt/benchmark/benchmark.c
/*
double current_time(int reset)
{
	double time_now;
	double current_s = OS_GetTime() / 1000.0;
	double current_us = OS_GetTime_us() / 1000000.0;
	time_now = (double)( current_s + current_us);
	(void) reset;
	return time_now;
}*/
double current_time(int reset)
{
	portTickType tickCount;

	(void) reset;

	/* tick count == ms, if configTICK_RATE_HZ is set to 1000 */
	tickCount = xTaskGetTickCount();
	return (double)tickCount / 1000;
}

static void consoleCommTask(void * prm);

static CRTPPacket messageReceived;
CRTPPacket messageToPrint;
static int currBufferLen = 0;
static char droneData[1024];
char taskBuf[1000];
static xSemaphoreHandle consoleLock;

//static const char fullMsg[] = "<F>\n";
static bool isInit;

static char radioAddress[16] = "XXXXXXXXXXXXXXX\0";
static char radioChannel[3] = "XX\0";
static char radioDatarate[2] = "X\0";

static float mirr_time = 101;
static float aes_time = 101;
static uint64_t end_time = 0;
static uint64_t start_time = 0;

static bool encrypt = false;
static int encryptSent = 0;
static char encryptedData[CRTP_MAX_DATA_SIZE*2];
static char plainData[CRTP_MAX_DATA_SIZE*2];
static Aes aes;
static byte key[16] = {0x02, 0x01, 0x05, 0x10, 0x02, 0x01, 0x05, 0x10,0x02, 0x01, 0x05, 0x10,0x02, 0x01, 0x05, 0x10};
		// iv and key must be 16 bytes
static byte iv[16] = {0x02, 0x01, 0x05, 0x10, 0x02, 0x01, 0x05, 0x10,0x02, 0x01, 0x05, 0x10,0x02, 0x01, 0x05, 0x10};



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


  //int i;
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
	  while (crtpGetFreeTxQueuePackets() == 1){
		  vTaskDelay(1);
	  }
      /*if (crtpGetFreeTxQueuePackets() == 1)
      {
        for (i = 0; i < sizeof(fullMsg) && (messageToPrint.size - i) > 0; i++)
        {
          messageToPrint.data[messageToPrint.size - i] =
              (uint8_t)fullMsg[sizeof(fullMsg) - i - 1];
        }
	  }*/
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

void consoleCommEncflush(char * str, uint8_t lengthOfMessage){
	if (xSemaphoreTake(consoleLock, portMAX_DELAY) == pdTRUE)
	  {
		if (messageToPrint.size > 0){
			crtpSendPacket(&messageToPrint); //no error checking for now!!
			messageToPrint.size = 0;
		}
		encrypt = 1;
		encryptSent = 0;
		messageToPrint.header = CRTP_HEADER(CRTP_PORT_CONSOLE, ENCRYPTED_CHANNEL);
		uint8_t counter = 0;
		while (counter < lengthOfMessage){
			if (lengthOfMessage - counter < 16){
				messageToPrint.size = lengthOfMessage-counter;
				memcpy(plainData, str+counter, messageToPrint.size);
			    memcpy(plainData+messageToPrint.size, "padpadpadpadpadpad", 16-messageToPrint.size); // fill it to size 16
			    messageToPrint.size = 16;
			}
			else{
				messageToPrint.size = 16;
				memcpy(plainData, str+counter, messageToPrint.size);
			}
			counter = counter + 16;
		    wc_AesCbcEncrypt(&aes, (byte*)encryptedData, (byte*)plainData, 16);
		    memcpy(messageToPrint.data, encryptedData, 16);
		    memcpy(messageToPrint.data+16, "\0BadBadBadBad", 14);
		    crtpSendPacket(&messageToPrint);
		    encryptSent++;
		}
	    encrypt = 0;
		messageToPrint.header = CRTP_HEADER(CRTP_PORT_CONSOLE, PLAINTEXT_CHANNEL);
	    //sprintf((char*)messageToPrint.data, "ET:{%.10f}", total);
	    //messageToPrint.size = 0;
	    //while (messageToPrint.data[messageToPrint.size] != '}' && messageToPrint.size <= 30){
	    //	messageToPrint.size++;
	    //}
	    crtpSendPacket(&messageToPrint);
	    xSemaphoreGive(consoleLock);
	  }
}

void consoleCommInit()
{
  if (isInit)
    return;
  droneData[0] = 0;
  droneData[9] = 0;
  messageToPrint.size = 0;
  messageToPrint.header = CRTP_HEADER(CRTP_PORT_CONSOLE, PLAINTEXT_CHANNEL);
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
  if (configUSE_STATS_FORMATTING_FUNCTIONS == 1){
	  consoleCommPflush("cUSFF: True");
  }
  else{
	  consoleCommPflush("cUSFF: False");
  }
  if (SECRET_BIT_K == 1){
	  consoleCommPflush("ENCK: True");
  }
  else{
	  consoleCommPflush("ENCK: False");
  }
  wc_AesSetKey(&aes, key, 16, iv, AES_ENCRYPTION);
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
		} // change beacon send times
		else if (messageReceived.data[0] == '%' && messageReceived.data[1] == 'O'){
			changeOrder( (messageReceived.data[2] == 'F')  );
		} // change beacon order
		else if (messageReceived.data[0] == '%' && messageReceived.data[1] == 'P' && messageReceived.data[2] == 'D'){
			int numTasks = uxTaskGetNumberOfTasks();
			consoleCommPflush("****START****");

			memset(taskBuf, 0, 1000);
			vTaskGetRunTimeStats(taskBuf);
			taskBuf[numTasks*45] = '\0';
			consoleCommPflush(taskBuf);
			consoleCommPflush("****END****");
		} // drone profile 
		else if (messageReceived.data[0] == '%' && messageReceived.data[1] == 'T' && messageReceived.data[2] == 'A'){
				consoleCommPflush("Recorded AES time!");
				memcpy(plainData, "padpadpadpadpadpad", 16); // 16 bytes to encrypt
				start_time = usecTimestamp();
				wc_AesCbcEncrypt(&aes, (byte*)encryptedData, (byte*)plainData, 16);
				end_time = usecTimestamp();			
				aes_time = (end_time - start_time)/1000/1000; // in sec
						


		} // time AES
		else if (messageReceived.data[0] == '%' && messageReceived.data[1] == 'T' && messageReceived.data[2] == 'M'){
				consoleCommPflush("Recorded mirror time!");
				float encx = 1.23;
				float ency = -0.23;
				float encz = 0.51515;

				start_time = usecTimestamp();
				encx = -encx;
				ency = -ency;
				encz = 3-encz;
				
				end_time = usecTimestamp();			
				mirr_time = (end_time - start_time)/1000/1000; // in sec
				//send
				start_time = (int)(encx + ency + encz) ; // to disable make error (unused variable)
				sprintf((char*)encryptedData, "MT:{%f}.", (double)(mirr_time)); // disable make error


		} // time MIRROR
		else {
			consoleCommPflush("Unrecognized Command.");
		} // unrecognized command
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

LOG_GROUP_START(enc_time)
LOG_ADD(LOG_FLOAT,  mirr, &mirr_time)
LOG_ADD(LOG_FLOAT,  aes, &aes_time)
LOG_GROUP_STOP(enc_time)