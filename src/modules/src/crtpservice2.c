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
 * crtpservice.c - Implements low level services for CRTP
 */

#include <stdbool.h>
#include <string.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
/* WolfSSL includes */
#include "ssl.h"
#include "aes.h" //maybe?
//#include "aes.h"
//#include "cyassl/ctaocrypt/aes.h"
/* STM32F4xx includes */
//#include "stm32f4xx_rng.h"
/* local includes */
#include "crtp.h"
#include "crtpservice2.h"

static bool isInit=false;

//wolfssl callbacks i dont know how to implement these.
//typedef int (*CallbackMacEncrypt)(WOLFSSL* ssl, );//Mac and Encrypt
//typedef int (*CallbackDecryptVerify)(WOLFSSL* ssl, );//decrypt and verify
//typedef int (*CallbackIORecv)(WOLFSSL* ssl, );//IO receive
//typedef int (*CallbackIOSend)(WOLFSSL* ssl, char *buf, int sz, void *ctx);//IO send

//static Aes enc;
//static Aes dec;

//be able to read from memory once it is completed
static const byte key[] = {0x57, 0x01, 0x2A, 0x12, 0xA7, 0x7A, 0x12, 0xBA, 0x57, 0x01, 0x2A, 0x12, 0xA7, 0x7A, 0x12, 0xBA};

static const byte iv[] = {0x57, 0x01, 0x2A, 0x12};
//const byte plain[] = {0x57, 0x01, 0x2A, 0x12, 0xA7, 0x7A, 0x12, 0xBA, 0x57, 0x01, 0x2A, 0x12, 0xA7, 0x7A, 0x12, 0xBA};
//byte cipher[sizeof(plain)];
static byte authTag[4];
static byte associatedData[] = {0x57, 0x01, 0x2A, 0x12};
//byte completeMessage[sizeof(iv) + sizeof(associatedData) + sizeof(authTag) + sizeof(plain)];
//Gmac gmac;


typedef enum {
  linkEcho   = 0x00,
  linkSource = 0x01,
  linkSink   = 0x02,
} LinkNbr;

void crtpservice2Handler(CRTPPacket *p);

void concatByteArrays(byte *a, word32 asz, byte *b, word32 bsz, byte *outPutArray, word32 outputLength);

void crtpservice2Init(void)
{
  if (isInit)
    return;

  // Register a callback to service the Link port
  crtpRegisterPortCB(CRTP_PORT_GCM, crtpservice2Handler);

  /* AES GCM init */ //dont know what it entails though
  //AesSetKey(&enc, key, sizeof(key), iv, AES_ENCRYPTION);
  //add RNG_Cmd() to init RNG on the stm32f4xx add STM32F4xx_RNG.h to the include files.
  //wolfSSL_Init();

  isInit = true;
}

bool crtpservice2Test(void)
{
  return isInit;
}

void crtpservice2Handler(CRTPPacket *p)
{
	byte recievedData[p->size];
	byte recIV[sizeof(iv)];
	byte recAD[sizeof(associatedData)];
	byte recTAG[sizeof(authTag)];
	byte recCipher[sizeof(recievedData)-sizeof(iv)-sizeof(associatedData)-sizeof(authTag)];//DANGER
	byte decrypted[sizeof(recCipher)];
	bzero(decrypted, sizeof(decrypted));
  switch (p->channel)
  {
    case linkEcho:
      crtpSendPacket(p);
      break;
    case linkSource:
      p->size = CRTP_MAX_DATA_SIZE;
      bzero(p->data, CRTP_MAX_DATA_SIZE);
      strcpy((char*)p->data, "enc plain sent on ch 2");
      crtpSendPacket(p);
      break;
    case linkSink:
      /* Ignore packet */
      /* Now used to test AES-GCM */ //size of AD, should AD be used at all? questions. could denote drone ID

    	wc_AesGcmSetKey(&dec, key, sizeof(key));
    	memcpy(recievedData, p->data, sizeof(recievedData));
    	byte counter = 0;

    	for(unsigned char j=0; j < sizeof(recIV);j++){
    		recIV[j] = recievedData[counter];
    		counter++;
    	}

    	for(unsigned char j=0; j < sizeof(recAD);j++){
    		recAD[j] = recievedData[counter];
    		counter++;
    	}

    	for(unsigned char j=0; j < sizeof(recTAG);j++){
    		recTAG[j] = recievedData[counter];
    		counter++;
    	}

    	for(unsigned char j=0; j < sizeof(recCipher);j++){
    		recCipher[j] = recievedData[counter];
    		counter++;
    	}
    	counter = 0;


    	wc_AesGcmDecrypt(&dec, decrypted, recCipher, sizeof(decrypted), recIV, sizeof(recIV), recTAG, sizeof(recTAG), recAD, sizeof(recAD));

    	/*
    	byte cipher[sizeof(plain)];

    	wc_AesGcmSetKey(&enc, key, sizeof(key));
    	wc_AesGcmEncrypt(&enc, cipher, plain, sizeof(plain), iv, sizeof(iv), authTag, sizeof(authTag), associatedData, sizeof(associatedData));

    	byte completeMessage[sizeof(iv) + sizeof(associatedData) + sizeof(authTag) + sizeof(cipher)];
    	counter = 0;
    	//unsigned char counter = 0;
    	for(unsigned char j=0; j < sizeof(iv);j++){
    		completeMessage[counter] = iv[j];
    		counter++;
    	}

    	for(unsigned char j=0; j < sizeof(associatedData);j++){
    		completeMessage[counter] = associatedData[j];
    		counter++;
    	}

    	for(unsigned char j=0; j < sizeof(authTag);j++){
    		completeMessage[counter] = authTag[j];
    		counter++;
    	}

    	for(unsigned char j=0; j < sizeof(cipher);j++){
    		completeMessage[counter] = cipher[j];
    		counter++;
    	}
*/
    	p->size = sizeof(decrypted);
    	bzero(p->data, sizeof(decrypted));
    	memcpy(p->data, decrypted, sizeof(decrypted));
    	crtpSendPacket(p);
      break;
    default:
      break;
  }
}

void concatByteArrays(byte *a, word32 asz, byte *b, word32 bsz, byte *outPutArray, word32 outputLength)
{

}
