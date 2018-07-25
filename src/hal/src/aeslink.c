#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "config.h"
#include "usblink.h"
#include "crtp.h"
#include "configblock.h"
#include "ledseq.h"
#include "pm.h"
#include "queue.h"
#include "syslink.h"
#include "crtp.h"
#include "radiolink.h"
#include "console.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "queuemonitor.h"
#include "semphr.h"

#include "aeslink.h"
#include "ssl.h"
#include "aes.h"
#include "consoleComm.h"

#define AD_START					0
#define IV_START					2
#define TAG_START					6
#define DATA_START					10
#define HEADER_POSITION				0
#define PID_POSITION				1

#define ADDITIONAL_BYTES  			10
#define AUTH_DATA_SIZE				2
#define AUTH_TAG_SIZE 				4
#define INIT_VECTOR_SIZE			4
#define MAX_DATA_IN_FIRST_PACKET 	20

#define PID_BYTE					1
#define HEADER_BYTE					1
#define FIRST_OF_MULTI_PACKET_MASK	0x80u
#define MULTI_PACKET_BIT_MASK		0x80u
#define PID_NBR_MASK				0x60
#define PACKET_DATA_LENGTH_MASK		0x1F
#define HEADER_PORT_MASK			0xF0u
#define HEADER_CHANNEL_MASK			0x03

#define MAX_FIRST_DATA_LENGTH		0x15

#define CIPHERED_PORT				0x0Bu
#define CIPHERED_CHANNEL			0x03
#define CHIPHERED_HEADER			0xB3u

static xQueueHandle crtpPacketDelivery;

static int aeslinkSetEnable(bool enable);
static int aeslinkReceiveCRTPPacket(CRTPPacket *p);
static void aeslinkTask(void *param);

static bool isInit = false;

static int nopFunc(void);
static struct crtpLinkOperations nopLink = {
  .setEnable         = (void*) nopFunc,
  .sendPacket        = (void*) nopFunc,
  .receivePacket     = (void*) nopFunc,
};

static struct crtpLinkOperations *link = &nopLink;

struct crtpLinkOperations aeslinkOp =
{
  .setEnable         = aeslinkSetEnable,
  .sendPacket        = aeslinkSendCRTPPacket,
  .receivePacket     = aeslinkReceiveCRTPPacket,
};

static CRTPPacket p;

//THIS IS THE ENCRYPTION AND DECRYPTION KEY
static const byte key[] = {0x57, 0x01, 0x2A, 0x12, 0xA7, 0x7A, 0x12, 0xBA, 0x57, 0x01, 0x2A, 0x12, 0xA7, 0x7A, 0x12, 0xBA};

static byte sendInitVector[] = {0x00, 0x00, 0x00, 0x00};
static Aes enc;
static Aes dec;
static byte sendPlainPackageData[CRTP_MAX_DATA_SIZE];
static byte sendCipherPackageData[CRTP_MAX_DATA_SIZE];
static byte sendAuthData[AUTH_DATA_SIZE];
static byte sendAuthTag[AUTH_TAG_SIZE];
static byte recPlainPackageData[CRTP_MAX_DATA_SIZE];
static byte recInitVector[INIT_VECTOR_SIZE];
static byte recCipherPackageData[CRTP_MAX_DATA_SIZE];
static byte recAuthData[AUTH_DATA_SIZE];
static byte recAuthTag[AUTH_TAG_SIZE];
static CRTPPacket sp;
static CRTPPacket rp;
static bool messageComplete = false;
static bool splitMessage = false;
static byte recPid = 100;

static void aeslinkTask(void *param){
	while (true)
	  {
	    if (link != &nopLink)
	    {
	      if (!link->receivePacket(&p))
	      {
	       if(p.size > 2)
	       {
	    	   /*
	    	    * A received packet is dissasembled into their different constitutents, put into arrays
	    	    * and prepared for decryption.
	    	    * */
			   if((p.data[PID_POSITION] & PID_NBR_MASK) != recPid)
			   {
				   byte datalength = 0;

				   if((p.data[PID_POSITION] & PACKET_DATA_LENGTH_MASK) <= MAX_DATA_IN_FIRST_PACKET)
				   {
					   datalength = p.data[PID_POSITION] & PACKET_DATA_LENGTH_MASK;
					   messageComplete = true;
				   } else
				   {
					   datalength = MAX_DATA_IN_FIRST_PACKET;
				   }

					recAuthData[HEADER_POSITION] = p.data[HEADER_POSITION] & (HEADER_CHANNEL_MASK | HEADER_PORT_MASK);
					recAuthData[PID_POSITION] = p.data[PID_POSITION];

					memcpy(&recInitVector, &p.data[IV_START], INIT_VECTOR_SIZE);
					memcpy(&recAuthTag, &p.data[TAG_START], AUTH_TAG_SIZE);
					memcpy(&recCipherPackageData, &p.data[DATA_START], datalength);
			   }

			   /*
			    * If a packet is divided into two parts, this second part receives the second packet and adds it
			    * to the arrays for decryption.
			    * */
			   if((p.data[PID_POSITION] & PID_NBR_MASK) == recPid)
			   {
				   byte datalength = 0;
				   datalength = (p.data[PID_POSITION] & PACKET_DATA_LENGTH_MASK) - MAX_DATA_IN_FIRST_PACKET;
				   if(datalength <= 10){
					   memcpy(&recCipherPackageData[MAX_DATA_IN_FIRST_PACKET], &p.data[2], datalength);
					   messageComplete = true;
				   }

			   }

			   /*
			    * Once the complete packet has been received the data is deciphered.
			    * */

			   if(messageComplete)
			   {
				   int failedDecrypt = 0;
				   failedDecrypt = wc_AesGcmDecrypt(&dec,
						   recPlainPackageData,
						   recCipherPackageData,
						   p.data[PID_BYTE] & PACKET_DATA_LENGTH_MASK,
						   recInitVector,
						   INIT_VECTOR_SIZE,
						   recAuthTag,
						   AUTH_TAG_SIZE,
						   recAuthData,
						   AUTH_DATA_SIZE);

				   //This part will only be run if the decryption was successful.
				   if(!(failedDecrypt))
				   {
					   rp.port = (p.data[HEADER_POSITION] & HEADER_PORT_MASK) >> 4;
					   rp.channel = p.data[HEADER_POSITION] & HEADER_CHANNEL_MASK;
					   rp.size = p.data[PID_BYTE] & PACKET_DATA_LENGTH_MASK;
					   consoleCommPflush("Decrypted data:\n");
					   consoleCommPflush((char*)recPlainPackageData);
					   memcpy(&rp.data, &recPlainPackageData, p.data[PID_BYTE] & PACKET_DATA_LENGTH_MASK);
					   xQueueSend(crtpPacketDelivery, &rp, 0);
				   }
				   messageComplete = false;
				   recPid = 100;
			   }
		  }
	    }
	  }
	    else
	    {
	      vTaskDelay(M2T(10));
	    }
	  }
}

static int aeslinkReceiveCRTPPacket(CRTPPacket *p)
{

	if (xQueueReceive(crtpPacketDelivery, p, M2T(100)) == pdTRUE)
	{

		return 0;
	}

	return -1;
}

/*
 * A packet prepares to be sent. A PID is generated, the length is determined, IV is generated,
 * AAD is added and message is encrypted generating a MAC.
 * */
int aeslinkSendCRTPPacket(CRTPPacket *p)
{
	static byte sendPid = 0;
	byte dataLength = 0;



	if((p->channel | p->port) == 0xF3){
		link->sendPacket(p);
		return 1;
	}

	if(sendPid >3){
		sendPid = 0;
	}

	if((p->size)>MAX_DATA_IN_FIRST_PACKET){
		dataLength = MAX_DATA_IN_FIRST_PACKET;
		splitMessage = true;
	} else {
		dataLength = p->size;
	}

	sp.size = dataLength + ADDITIONAL_BYTES;
	sp.port = CIPHERED_PORT;
	sp.channel = CIPHERED_CHANNEL;

	sendAuthData[HEADER_POSITION] = ((p->port<<4) | (p->channel));
	if(splitMessage){
		sendAuthData[PID_POSITION] = FIRST_OF_MULTI_PACKET_MASK | ((sendPid<<5)&PID_NBR_MASK) | (p->size);
	} else {
		sendAuthData[PID_POSITION] = ((sendPid<<5)&PID_NBR_MASK) | (p->size);
	}

	memcpy(&sendPlainPackageData, p->data, p->size);

	int failedEncrypt;

	failedEncrypt = wc_AesGcmEncrypt(&enc,
				sendCipherPackageData,
				sendPlainPackageData,
				p->size,
				sendInitVector,
				INIT_VECTOR_SIZE,
				sendAuthTag,
				AUTH_TAG_SIZE,
				sendAuthData,
				AUTH_DATA_SIZE);

	//if the encryption is failed the function returns 0.
	if(failedEncrypt){
		return 0;
	}

	memcpy(&sp.data[AD_START], &sendAuthData, AUTH_DATA_SIZE);
	memcpy(&sp.data[IV_START], &sendInitVector, INIT_VECTOR_SIZE);
	memcpy(&sp.data[TAG_START], &sendAuthTag, AUTH_TAG_SIZE);
	memcpy(&sp.data[DATA_START], &sendCipherPackageData, dataLength);

	link->sendPacket(&sp);

	/*
	 * If the original packet is too large to fit a single packet a second packet is sent.
	 * */

	if(splitMessage){
		dataLength = (p->size)-MAX_DATA_IN_FIRST_PACKET;
		sp.size = dataLength + PID_BYTE + HEADER_BYTE;

		memcpy(&sp.data[AD_START], &sendAuthData, AUTH_DATA_SIZE);
		memcpy(&sp.data[IV_START], &sendCipherPackageData[MAX_DATA_IN_FIRST_PACKET], dataLength);
		link->sendPacket(&sp);
	}

	//The IV is a sequential counter of 4 bytes.

	if(sendInitVector[0] == 0xFF){
		sendInitVector[0] = 0;
		sendInitVector[1]++;
	}else {
		sendInitVector[0]++;
	}
	if(sendInitVector[1] == 0xFF){
		sendInitVector[1] = 0;
		sendInitVector[2]++;
	}
	if(sendInitVector[2] == 0xFF){
		sendInitVector[2] = 0;
		sendInitVector[3]++;
	}
	if(sendInitVector[3] == 0xFF){
		sendInitVector[3] = 0;
	}

	sendPid++;
	splitMessage = false;
	return 1;
}

static int aeslinkSetEnable(bool enable)
{
	return 0;
}

//public functions
void aeslinkInit()
{
	if(isInit)
		return;

	crtpPacketDelivery = xQueueCreate(5, sizeof(CRTPPacket));
	DEBUG_QUEUE_MONITOR_REGISTER(crtpPacketDelivery);

	xTaskCreate(aeslinkTask, AESLINK_TASK_NAME ,
	            AESLINK_TASK_STACKSIZE, NULL, AESLINK_TASK_PRI, NULL);

	wc_AesGcmSetKey(&dec, key, sizeof(key));
	wc_AesGcmSetKey(&enc, key, sizeof(key));

	isInit = true;
}

bool aeslinkTest()
{
	return isInit;
}

void aesEnableTunnel(){
	//crtpGetLink(link);
	link = radiolinkGetLink();
	//crtpSetLink(&aeslinkOp);
	aeslinkInit();
}

static int nopFunc(){
	return 0;
}

