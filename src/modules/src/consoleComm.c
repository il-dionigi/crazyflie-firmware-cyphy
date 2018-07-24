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
#include <openssl/evp.h>


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

//Encryption stuff
EVP_CIPHER_CTX en, de;
static char encryptedMessage[128];

/* 8 bytes to salt the key_data during key generation. This is an example of
   compiled in salt. We just read the bit pattern created by these two 4 byte
   integers on the stack as 64 bits of contigous salt material -
   ofcourse this only works if sizeof(int) >= 4 */
unsigned int salt[] = {12345, 54321};
unsigned char *key_data;
int key_data_len, i;
char *input[] = {"a", "abcd", "this is a test", "this is a bigger test",
				 "\nWho are you ?\nI am the 'Doctor'.\n'Doctor' who ?\nPrecisely!",
				 NULL};

int aes_init(unsigned char *key_data, int key_data_len, unsigned char *salt, EVP_CIPHER_CTX *e_ctx,
             EVP_CIPHER_CTX *d_ctx)
{
  int i, nrounds = 5;
  unsigned char key[32], iv[32];

  /*
   * Gen key & IV for AES 256 CBC mode. A SHA1 digest is used to hash the supplied key material.
   * nrounds is the number of times the we hash the material. More rounds are more secure but
   * slower.
   */
  i = EVP_BytesToKey(EVP_aes_256_cbc(), EVP_sha1(), salt, key_data, key_data_len, nrounds, key, iv);
  if (i != 32) {
    printf("Key size is %d bits - should be 256 bits\n", i);
    return -1;
  }

  EVP_CIPHER_CTX_init(e_ctx);
  EVP_EncryptInit_ex(e_ctx, EVP_aes_256_cbc(), NULL, key, iv);
  EVP_CIPHER_CTX_init(d_ctx);
  EVP_DecryptInit_ex(d_ctx, EVP_aes_256_cbc(), NULL, key, iv);

  return 0;
}

/*
 * Encrypt *len bytes of data
 * All data going in & out is considered binary (unsigned char[])
 */
unsigned char *aes_encrypt(EVP_CIPHER_CTX *e, unsigned char *plaintext, int *len)
{
  /* max ciphertext len for a n bytes of plaintext is n + AES_BLOCK_SIZE -1 bytes */
  int c_len = *len + AES_BLOCK_SIZE, f_len = 0;
  unsigned char *ciphertext = malloc(c_len);

  /* allows reusing of 'e' for multiple encryption cycles */
  EVP_EncryptInit_ex(e, NULL, NULL, NULL, NULL);

  /* update ciphertext, c_len is filled with the length of ciphertext generated,
    *len is the size of plaintext in bytes */
  EVP_EncryptUpdate(e, ciphertext, &c_len, plaintext, *len);

  /* update ciphertext with the final remaining bytes */
  EVP_EncryptFinal_ex(e, ciphertext+c_len, &f_len);

  *len = c_len + f_len;
  return ciphertext;
}

/*
 * Decrypt *len bytes of ciphertext
 */
unsigned char *aes_decrypt(EVP_CIPHER_CTX *e, unsigned char *ciphertext, int *len)
{
  /* plaintext will always be equal to or lesser than length of ciphertext*/
  int p_len = *len, f_len = 0;
  unsigned char *plaintext = malloc(p_len);

  EVP_DecryptInit_ex(e, NULL, NULL, NULL, NULL);
  EVP_DecryptUpdate(e, plaintext, &p_len, ciphertext, *len);
  EVP_DecryptFinal_ex(e, plaintext+p_len, &f_len);

  *len = p_len + f_len;
  return plaintext;
}

//end enc stuff


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
   unsigned int len = strlen(messageToPrint.data)+1;
   char * cipherText;
  cipherText = aes_encrypt(&en, (unsigned char *)messageToPrint.data, &len);
  len = strlen(messageToPrint.data);
  encryptedMessage[0] = "<";
  memcpy(encryptedMessage+1, cipherText, len);
  encryptedMessage[len+1] = ">";
  encryptedMessage[len+2] = "\0";
  len = len+3; //enc message has 3 more chars
  unsigned int charsSent = 0;
  int diff = 0;
  while(charsSent < len){
	  diff = len - charsSent;
	  if (diff < 30){
		  memcpy(messageToPrint.data, encryptedMessage+charsSent, diff);
		  messageToPrint.data[diff+1] = '\0';
		  messageToPrint.size = diff+1;
	  }
	  else {
		  memcpy(messageToPrint.data, encryptedMessage+charsSent, 30);
		  messageToPrint.size = 30;
	  }
	  if (crtpSendPacket(&messageToPrint) == pdTRUE)
	  {
		charsSent += 30;
	  }
	  else
	  {
		  //Error, we should light an LED here
		return false;
	  }
  }
  messageToPrint.size = 0;
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


    key_data = "ThisIsMyKey";
    key_data_len = strlen(key_data);

    /* gen key and iv. init the cipher ctx object */
    /*if (aes_init(key_data, key_data_len, (unsigned char *)&salt, &en, &de)) {
      printf("Couldn't initialize AES cipher\n");
      return -1;
    }*/
  aes_init(key_data, key_data_len, (unsigned char *)&salt, &en, &de);
  isInit = true;

  consoleCommPflush("console comm init!");
}


//SEND CODE ABOVE. RECEIVE CODE BELOW
void consoleCommTask(void * prm)
{
	crtpInitTaskQueue(CRTP_PORT_CONSOLE);
	uint64_t address = 0;

	uint8_t channel = 0;
	uint8_t dataRate = 0;
	uint8_t slept = 0;
	char temp;
	while(1) {
		crtpReceivePacketBlock(CRTP_PORT_CONSOLE, &messageReceived);
		consoleCommPuts("got msg from pc; data/channel:");
		consoleCommPflush((char*)(messageReceived.data));
		temp = messageReceived.channel + '0';
		consoleCommPutchar(temp);
		consoleCommFlush();

    switch (messageReceived.channel) {
      case C2RTP_CHANNEL_TEXT:
        displayRadioAddress();
        displayRadioChannel();
        displayRadioDatarate();
        consoleCommFlush();
        if (messageReceived.data[0] == '?'){
          consoleCommPflush("Current data in droneData:");
          consoleCommPflush(droneData);
        }
        else if (messageReceived.data[0] == '~'){
          consoleCommPflush("1! Sending to beacon; port,message:");
          consoleCommPutchar(messageReceived.data[1]);
          consoleCommPutchar(',');
          consoleCommPflush((char*)(messageReceived.data+2));
          beaconCommPflush((char*)(messageReceived.data+2));
        }
        else if (messageReceived.data[0] == '!') {
          consoleCommPflush("Putting data in droneData:");
          consoleCommPflush((char*)(messageReceived.data+1));
          memcpy(&droneData, messageReceived.data + 1, 9);
        }
        break;
      case C2RTP_CHANNEL_SWITCH:
        consoleCommPflush("Currently switching channels");
        if (!slept){
        	consoleCommPflush("Sleeping");
        	slept = 1;
        	break;
        }
        else{
        	consoleCommPflush("Waking");
        }
        int i;
        for (i = 0; i < CRTP_MAX_DATA_SIZE; i++){
          temp = messageReceived.data[i];
          if (temp == ',') {
        	i++;
            break;
          }
          else {
            address *= 10;
            address += temp - '0';
          }
        }
        for (; i < CRTP_MAX_DATA_SIZE; i++){
		  temp = messageReceived.data[i];
		  if (temp == ',') {
			i++;
			break;
		  }
		  else {
			channel *= 10;
			channel += temp - '0';
		  }
		}
        for (; i < CRTP_MAX_DATA_SIZE; i++){
		  temp = messageReceived.data[i];
		  if (temp == ',') {
			i++;
			break;
		  }
		  else {
			dataRate *= 10;
			dataRate += temp - '0';
		  }
		}
        /*crtpSwitchTarget(address, channel, dataRate);
        while(true) {
          consoleCommPflush("!d2dWorkd\0");
        }*/
        break;
      default:
        break;
    }
	}
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
