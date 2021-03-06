/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2016, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/* uwb_twr_anchor.c: Uwb two way ranging anchor implementation */


#include <string.h>
#include <math.h>

#include "lpsTwrTag.h"

#include "../../../modules/interface/consoleComm.h"
#include "lpsTdma.h"

#include "FreeRTOS.h"
#include "task.h"

#include "log.h"
#include "crtp_localization_service.h"

#include "stabilizer_types.h"
#include "estimator_kalman.h"
#include "arm_math.h"

//CYPHY
#include "beaconComm.h"
#include "consoleComm.h"

// Outlier rejection
#define RANGING_HISTORY_LENGTH 32
#define OUTLIER_TH 4
#define LPS_MAX_DATA_SIZE 30

//cyphy
uint32_t KEY_DELTA = 2000; // the key, anchor adds this to t3 when data is sent
static uint32_t last_send_time[20] = { 0 };
static uint16_t ticksPerMsg = 3500;
static char anchors[9] = "xxxxxxxx\0";
static uint32_t ts[20] = {0};
static uint32_t delta_p = 1234;
static uint8_t delta_ph8 = 56;
static uint32_t singleRanging = 1234;
static uint32_t allRangings = 1234;
static uint32_t betweenRounds = 1234;
static uint32_t betweenRangings = 1234;
static uint8_t singleRanging_h8 = 56;
static uint8_t allRangings_h8 = 56;
static uint8_t betweenRounds_h8 = 56;
static uint8_t betweenRangings_h8 = 56;
static uint8_t anchor_order[LOCODECK_NR_OF_ANCHORS];
static uint8_t anchor_index = 0;
static bool fixedOrder = true;
//single ranging: time beacon0 report received - poll sent
//all rangings: time between beacon7 report received - beacon0 poll sent
//between rounds: time between beacon0 poll1 - beacon0 poll2
//between rangings: time between beacon1 poll sent - beacon0 report received
//delta_p: time beacon0 poll2 - report1
static float est_tof_add = 0; // eve guess of 2*TOF = (t10-t6)-delta_b
static float est_tof_mult = 0;
static float actual_tof_add = 0; //actual TOF using addition formula
static float actual_tof_mult = 0; //actual TOF using multiplication formula
static double DELTA_D_CONST = 0.00051219576;
static double DELTA_B_CONST = 0.00056281076;
static double DELTA_P_CONST = 0.06530620845;


//static uint8_t delta_delay = 10;
//static uint32_t delta_delay_counter = 0;
static uint32_t delta_bs[8] = {0};
  dwTime_t startTime = {.full = 0};
  dwTime_t endTime;
static uint16_t delta_p_slot = 0;

static struct {
  float32_t history[RANGING_HISTORY_LENGTH];
  size_t ptr;
} rangingStats[LOCODECK_NR_OF_ANCHORS];


// Rangin statistics
static uint8_t rangingPerSec[LOCODECK_NR_OF_ANCHORS];
static uint8_t rangingSuccessRate[LOCODECK_NR_OF_ANCHORS];
// Used to calculate above values
static uint8_t succededRanging[LOCODECK_NR_OF_ANCHORS];
static uint8_t failedRanging[LOCODECK_NR_OF_ANCHORS];

// Timestamps for ranging
static dwTime_t poll_tx;
static dwTime_t poll_rx;
static dwTime_t answer_tx;
static dwTime_t answer_rx;
static dwTime_t final_tx;
static dwTime_t final_rx;
static dwTime_t report_rx = {.full = 0};



static packet_t txPacket;
static volatile uint8_t curr_seq = 0;
static int current_anchor = 0;

static bool ranging_complete = false;
static bool lpp_transaction = false;

static lpsLppShortPacket_t lppShortPacket;

static lpsAlgoOptions_t* options;

// TDMA handling
static bool tdmaSynchronized;
static dwTime_t frameStart;

static bool rangingOk;

static int messageExpected[LOCODECK_NR_OF_ANCHORS] = {0};
static int messageToSend[LOCODECK_NR_OF_ANCHORS] = {0};
static char message[LPS_MAX_DATA_SIZE];

void randomizeOrder(dwDevice_t *dev){
	uint8_t i, j, tmp ;
	if (fixedOrder){
		for (i = 0; i < LOCODECK_NR_OF_ANCHORS; i++){
	  		anchor_order[i] = i;
  		}
	}
	else{
		for (i = LOCODECK_NR_OF_ANCHORS-1; i >= 1; i-- ){
			//generate random number
			dwNewTransmit(dev);
			dwSetDefaults(dev);
			dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2+28);
			dwWaitForResponse(dev, false);
			dwStartTransmit(dev);
			dwGetTransmitTimestamp(dev, &startTime);
			j = (startTime.low32 % 10) % (i+1);
			tmp = anchor_order[j];
			anchor_order[j] = anchor_order[i];
			anchor_order[i] = tmp;
		}
	}
}

void changeTDMAslot(uint8_t slot){
	options->tdmaSlot = slot*5;
	delta_p_slot = slot*5;
        if (slot == 0){
	    options->useTdma = false;
	    consoleCommPflush("DISABLED TDMA"); 
	}
	else {
	    options->useTdma = true;
	    consoleCommPflush("USING TDMA"); 
	}
        char slotStr[6];
	slotStr[0] = '.';
	slotStr[1] = '.';
	slotStr[2] = '.';
	slotStr[3] = '.';
	slotStr[4] = '.';
	slotStr[5] = '0';
	uint8_t tmp_i = 0;
        uint16_t tmp_slot = delta_p_slot;
	while (tmp_slot > 0){
		slotStr[5-tmp_i] = (tmp_slot % 10) + '0';
		tmp_slot = tmp_slot / 10;
		tmp_i = tmp_i + 1;
	}
	consoleCommPuts("Slot:");
	consoleCommPuts(slotStr);
	consoleCommFlush();
}

void changeOrder(bool fixed){
	fixedOrder = fixed;
	if (fixed){
		consoleCommPflush("Fixed beacon order");
	}
	else {
		consoleCommPflush("Random beacon order");
	}
}

void sendMessageToBeacon(char * msg){
	int ii = 0;
	for (ii = 0; ii < LOCODECK_NR_OF_ANCHORS; ii++){
		messageToSend[ii] = 1;
	}
	consoleCommPflush("3! About to send this to all beacons:");
	consoleCommPflush(msg);
	memcpy(message, msg, LPS_MAX_DATA_SIZE);
}

static void txcallback(dwDevice_t *dev)
{
	/*if (last_send_time[9] + ticksPerMsg < xTaskGetTickCount()){
		consoleCommPflush("tx callback (9)");
		last_send_time[9] = xTaskGetTickCount();
	}*/
  dwTime_t departure;
  dwGetTransmitTimestamp(dev, &departure);
  departure.full += (options->antennaDelay / 2);

  switch (txPacket.payload[0]) {
    case LPS_TWR_POLL:
      poll_tx = departure;
      if (current_anchor == 1){
	betweenRangings = poll_tx.low32 - ts[7];
	betweenRangings_h8 = poll_tx.high8 - ts[8];
      }
      break;
    case LPS_TWR_FINAL:
      final_tx = departure;
      break;
  }

}

//CYPHY changed
static uint32_t rxcallback(dwDevice_t *dev) {
	char chAnchor = current_anchor + '0';
	if (last_send_time[current_anchor] + ticksPerMsg < xTaskGetTickCount()){
		
		anchors[current_anchor] = chAnchor;
		last_send_time[current_anchor] = xTaskGetTickCount();
	}
	if (last_send_time[15] + 2*ticksPerMsg < xTaskGetTickCount()){
		/*consoleCommPuts("(TWR)Anchors:");
		consoleCommPuts(anchors);
		consoleCommFlush();
		last_send_time[15] = xTaskGetTickCount();
		uint16_t ii = 0;
		for (ii = 0; ii < 8; ii++){
			anchors[ii] = 'x';
		}*/
	}

  dwTime_t arival = { .full=0 };
  int dataLength = dwGetDataLength(dev);

  if (dataLength == 0) return 0;

  packet_t rxPacket;
  memset(&rxPacket, 0, MAC802154_HEADER_LENGTH);

  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

  if (rxPacket.destAddress != options->tagAddress) {
    dwNewReceive(dev);
    dwSetDefaults(dev);
    dwStartReceive(dev);
    return MAX_TIMEOUT;
  }

  txPacket.destAddress = rxPacket.sourceAddress;
  txPacket.sourceAddress = rxPacket.destAddress;
  //CYPHY, last case statement
  //beaconAnalyzePayload((char*)rxPacket.payload);
  //Print header;
  char hdr[2];
  hdr[0] = rxPacket.payload[LPS_TWR_TYPE];
  hdr[1] = '\0';
  //consoleCommPflush("beaconDataHeader:");
  //consoleCommPflush(hdr);
  if (messageExpected[current_anchor] && (rxPacket.payload[LPS_TWR_TYPE] != LPS_TWR_RELAY_B2D)){
	  consoleCommPuts("expected B2D(5), anchor:");
	  consoleCommPutchar(chAnchor);
	  consoleCommFlush();
	  return 0;
  }
  else if (hdr[0] > 4){
	  hdr[0] += '0';
	  consoleCommPflush("beaconDataHeader:");
	  consoleCommPflush(hdr);
  }
  switch(rxPacket.payload[LPS_TWR_TYPE]) {
    // Tag received messages
    case LPS_TWR_ANSWER:
		//Add delay of delta ~cyphy~, received answer, sending final
		//uint32_t delay = 100;
		//for (uint16_t ii = 0; ii < delay; ii++){
			//each instruction has a few nanosec delay(?)
		//}
		//uint32_t ticks = 1; // each tick is 1 ms
		//vTaskDelay(ticks);

      if (rxPacket.payload[LPS_TWR_SEQ] != curr_seq) {
        return 0;
      }

      if (dataLength - MAC802154_HEADER_LENGTH > 3) {
        if (rxPacket.payload[LPS_TWR_LPP_HEADER] == LPP_HEADER_SHORT_PACKET) {
          int srcId = -1;

          for (int i=0; i<LOCODECK_NR_OF_ANCHORS; i++) {
            if (rxPacket.sourceAddress == options->anchorAddress[i]) {
              srcId = i;
              break;
            }
          }

          if (srcId >= 0) {
            lpsHandleLppShortPacket(srcId, &rxPacket.payload[LPS_TWR_LPP_TYPE],
                                    dataLength - MAC802154_HEADER_LENGTH - 3);
          }
        }
      }

      txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_FINAL;
      txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];

      dwGetReceiveTimestamp(dev, &arival);
      arival.full -= (options->antennaDelay / 2);
      answer_rx = arival;

      dwNewTransmit(dev);
      dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

      dwWaitForResponse(dev, true);
      dwStartTransmit(dev);

      break;
    case LPS_TWR_REPORT:
    {
      lpsTwrTagReportPayload_t *report = (lpsTwrTagReportPayload_t *)(rxPacket.payload+2);
      double tround1, treply1, treply2, tround2, tprop_ctn, tprop;

      if (rxPacket.payload[LPS_TWR_SEQ] != curr_seq) {
        return 0;
      }
	/* if (last_send_time[current_anchor] + ticksPerMsg < xTaskGetTickCount()){
		char chAnchor = current_anchor + '0';
		consoleCommPflush("Current anchor is");
		consoleCommPutchar(chAnchor);
	}*/
	  dwGetReceiveTimestamp(dev, &arival);
      arival.full -= (options->antennaDelay / 2);
      report_rx = arival;
      if (current_anchor == LOCODECK_NR_OF_ANCHORS-1){
	allRangings = report_rx.low32 - ts[0];
	allRangings_h8 = report_rx.high8 - ts[9];
      }
      memcpy(&poll_rx, &report->pollRx, 5);
      memcpy(&answer_tx, &report->answerTx, 5);
      memcpy(&final_rx, &report->finalRx, 5);
	  answer_tx.low32 -= KEY_DELTA;
      tround1 = answer_rx.low32 - poll_tx.low32;
      treply1 = answer_tx.low32 - poll_rx.low32;
      tround2 = final_rx.low32 - answer_tx.low32;
      treply2 = final_tx.low32 - answer_rx.low32;
      tprop_ctn = ((tround1*tround2) - (treply1*treply2)) / (tround1 + tround2 + treply1 + treply2);

      tprop = tprop_ctn / LOCODECK_TS_FREQ;
	delta_bs[current_anchor] = treply1;
	
	  if (current_anchor == 0){
		  //if (last_send_time[16] + 500 < xTaskGetTickCount()){
			//t6 is old ts[5], t10 is new ts[1]=poll_rx.low32,
			est_tof_add = (float)( (poll_rx.low32 - ts[6])/LOCODECK_TS_FREQ - DELTA_P_CONST)/2;
                        ////est_tof_mult 
			  tround1 = answer_rx.low32-(DELTA_P_CONST+ts[7]);
			  treply1 = (poll_rx.low32+DELTA_B_CONST) - poll_rx.low32;
      			  tround2 = final_rx.low32 - (poll_rx.low32+DELTA_B_CONST);
      			  treply2 = (answer_rx.low32+DELTA_D_CONST) - answer_rx.low32;
			  tprop_ctn = ((tround1*tround2) - (treply1*treply2)) / (tround1 + tround2 + treply1 + treply2);
			est_tof_mult = (float)(tprop_ctn / LOCODECK_TS_FREQ);
			////
			actual_tof_mult = tprop;
			delta_p = poll_tx.low32 - ts[7]; //new t1 - old t8
			delta_ph8 = poll_tx.high8 - ts[8]; 
			betweenRounds = poll_tx.low32 - ts[0];
			betweenRounds_h8 = poll_tx.high8 - ts[9];
			//delta_p =  report_rx.low32 - ts[7]; //new t8 - old t8
			ts[0] =  poll_tx.low32;
			ts[1] =  poll_rx.low32;
			ts[2] =  answer_tx.low32;
			ts[3] =  answer_rx.low32;
			ts[4] =  final_tx.low32;
			ts[5] =  final_rx.low32;
			ts[6] =  final_rx.low32 + (answer_tx.low32 - poll_rx.low32); //report_tx.low32 = t7 = t6 + delta_b
			ts[7] =  report_rx.low32;
			ts[8] =  report_rx.high8;
			ts[9] =  poll_tx.high8;
                        singleRanging = ts[7] - ts[0];
			singleRanging_h8 = ts[8] - poll_tx.high8;

			actual_tof_add = ((ts[1]-ts[0]) + (ts[3]-ts[2]) + (ts[5]-ts[4]) + (ts[7]-ts[6]))/4.0/LOCODECK_TS_FREQ;
			//last_send_time[16] = xTaskGetTickCount();
		  //}
	  }

      options->distance[current_anchor] = SPEED_OF_LIGHT * tprop;
      options->pressures[current_anchor] = report->asl;

      // Outliers rejection
      rangingStats[current_anchor].ptr = (rangingStats[current_anchor].ptr + 1) % RANGING_HISTORY_LENGTH;
      float32_t mean;
      float32_t stddev;

      arm_std_f32(rangingStats[current_anchor].history, RANGING_HISTORY_LENGTH, &stddev);
      arm_mean_f32(rangingStats[current_anchor].history, RANGING_HISTORY_LENGTH, &mean);
      float32_t diff = fabsf(mean - options->distance[current_anchor]);

      rangingStats[current_anchor].history[rangingStats[current_anchor].ptr] = options->distance[current_anchor];

      rangingOk = true;

      if ((options->combinedAnchorPositionOk || options->anchorPosition[current_anchor].timestamp) &&
          (diff < (OUTLIER_TH*stddev))) {
        distanceMeasurement_t dist;
        dist.distance = options->distance[current_anchor];
        dist.x = options->anchorPosition[current_anchor].x;
        dist.y = options->anchorPosition[current_anchor].y;
        dist.z = options->anchorPosition[current_anchor].z;
        dist.stdDev = 0.25;
        estimatorKalmanEnqueueDistance(&dist);
      }

      if (options->useTdma && current_anchor == 0) {
        // Final packet is sent by us and received by the anchor
        // We use it as synchonisation time for TDMA
        dwTime_t offset = { .full =final_tx.full - final_rx.full };
        frameStart.full = TDMA_LAST_FRAME(final_rx.full) + offset.full;
        tdmaSynchronized = true;
      }
	  //vTaskDelay(delta_delay);
      ranging_complete = true;
      return 0;
      break;
    }
    //CYPHY
    case LPS_TWR_RELAY_B2D:
    {
    	beaconAnalyzePayload((char*)rxPacket.payload);
    	ranging_complete = true;
    	consoleCommPuts("msg sent to: A");
		consoleCommPutchar(chAnchor);
		consoleCommFlush();
    	messageToSend[current_anchor] = 0;
    	messageExpected[current_anchor] = 0;      
    	return 0;
    	break;
    }
  }
  return MAX_TIMEOUT;
}

/* Adjust time for schedule transfer by DW1000 radio. Set 9 LSB to 0 */
static uint32_t adjustTxRxTime(dwTime_t *time)
{
  uint32_t added = (1<<9) - (time->low32 & ((1<<9)-1));
  time->low32 = (time->low32 & ~((1<<9)-1)) + (1<<9);
  return added;
}

/* Calculate the transmit time for a given timeslot in the current frame */
static dwTime_t transmitTimeForSlot(int slot)
{
  dwTime_t transmitTime = { .full = 0 };
  // Calculate start of the slot
  transmitTime.full = frameStart.full + slot*TDMA_SLOT_LEN;

  // DW1000 can only schedule time with 9 LSB at 0, adjust for it
  adjustTxRxTime(&transmitTime);
  return transmitTime;
}

//CYPHY changed
static void initiateRanging(dwDevice_t *dev)
{

  if (!options->useTdma || tdmaSynchronized) {
    if (options->useTdma) {
      // go to next TDMA frame
      frameStart.full += TDMA_FRAME_LEN;
    }

    //current_anchor ++;
	anchor_index++;
	/* if (last_send_time[current_anchor] + ticksPerMsg < xTaskGetTickCount()){
		char chAnchor = current_anchor + '0';
		consoleCommPflush("Current anchor is");
		consoleCommPutchar(chAnchor);
		last_send_time[current_anchor] = xTaskGetTickCount();
	}*/
    /*if (current_anchor >= LOCODECK_NR_OF_ANCHORS) {
      current_anchor = 0; 
    }*/
	if (anchor_index >= LOCODECK_NR_OF_ANCHORS) {
		anchor_index = 0; 
		randomizeOrder(dev);
	}
  } else {
    current_anchor = 0;
  }
  current_anchor = anchor_order[anchor_index];
  dwIdle(dev);
  if (messageToSend[current_anchor]){
	  messageExpected[current_anchor] = 1;
	  memcpy(txPacket.payload, message, LPS_MAX_DATA_SIZE);
	  txPacket.payload[LPS_TWR_TYPE] =  LPS_TWR_RELAY_D2B;
	  if (message[2] == 'K' && message[3] == 'D'){
		if (last_send_time[14] + ticksPerMsg < xTaskGetTickCount()){
			last_send_time[14] = xTaskGetTickCount();
		
		  KEY_DELTA = message[4];
		  consoleCommPuts("(drone14) key now: ");
		  consoleCommPutchar(KEY_DELTA);
		  consoleCommFlush();
		}
	  }

  }
  else{
	  txPacket.payload[LPS_TWR_TYPE] =  LPS_TWR_POLL;
  }
/*
if (current_anchor == 1){
 dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2+28);
  dwWaitForResponse(dev, false);
  dwStartTransmit(dev);

  dwGetTransmitTimestamp(dev, &endTime);
delta_bs[0] = endTime.low32 - startTime.low32;

startTime = endTime;
	
}
else if (current_anchor == 0){
 dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2+28);
  dwWaitForResponse(dev, false);
  dwStartTransmit(dev);

  dwGetTransmitTimestamp(dev, &endTime);
  
delta_bs[0] = endTime.low32 - startTime.low32;
}*/

  txPacket.payload[LPS_TWR_SEQ] = ++curr_seq;

  txPacket.sourceAddress = options->tagAddress;
  txPacket.destAddress = options->anchorAddress[current_anchor];

  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2+28);

  if (options->useTdma && tdmaSynchronized) {
    dwTime_t txTime = transmitTimeForSlot(options->tdmaSlot);
    dwSetTxRxTime(dev, txTime);
  }

 

  dwWaitForResponse(dev, true);
  
  dwStartTransmit(dev);
}

static void sendLppShort(dwDevice_t *dev, lpsLppShortPacket_t *packet)
{
  dwIdle(dev);

  txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_LPP_SHORT;
  memcpy(&txPacket.payload[LPS_TWR_SEND_LPP_PAYLOAD], packet->data, packet->length);

  txPacket.sourceAddress = options->tagAddress;
  txPacket.destAddress = options->anchorAddress[packet->dest];

  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+1+packet->length);

  dwWaitForResponse(dev, false);
  dwStartTransmit(dev);
}

static uint32_t twrTagOnEvent(dwDevice_t *dev, uwbEvent_t event)
{
  static uint32_t statisticStartTick = 0;

  /*if (last_send_time[10] + ticksPerMsg < xTaskGetTickCount()){
		consoleCommPflush("got event (10)");
		last_send_time[10] = xTaskGetTickCount();
  }*/

  if (statisticStartTick == 0) {
    statisticStartTick = xTaskGetTickCount();
  }

  switch(event) {
    case eventPacketReceived:
      return rxcallback(dev);
      break;
    case eventPacketSent:
      txcallback(dev);

      if (lpp_transaction) {
        return 0;
      }
	  /*if (last_send_time[11] + ticksPerMsg < xTaskGetTickCount()){
		consoleCommPflush("packet sent timeout (11)");
		last_send_time[11] = xTaskGetTickCount();
	  }*/
      return MAX_TIMEOUT;
      break;
    case eventTimeout:  // Comes back to timeout after each ranging attempt
	/*if (last_send_time[12] + ticksPerMsg < xTaskGetTickCount()){
		consoleCommPflush("event timeout (12)");
		last_send_time[12] = xTaskGetTickCount();
	}*/
      if (!ranging_complete && !lpp_transaction) {
        options->rangingState &= ~(1<<current_anchor);
        if (options->failedRanging[current_anchor] < options->rangingFailedThreshold) {
          options->failedRanging[current_anchor] ++;
          options->rangingState |= (1<<current_anchor);
        }

        locSrvSendRangeFloat(current_anchor, NAN);
        failedRanging[current_anchor]++;
      } else {
        options->rangingState |= (1<<current_anchor);
        options->failedRanging[current_anchor] = 0;

        locSrvSendRangeFloat(current_anchor, options->distance[current_anchor]);
        succededRanging[current_anchor]++;
      }

      // Handle ranging statistic
      if (xTaskGetTickCount() > (statisticStartTick+1000)) {
        statisticStartTick = xTaskGetTickCount();

        for (int i=0; i<LOCODECK_NR_OF_ANCHORS; i++) {
          rangingPerSec[i] = failedRanging[i] + succededRanging[i];
          if (rangingPerSec[i] > 0) {
            rangingSuccessRate[i] = 100.0f*(float)succededRanging[i] / (float)rangingPerSec[i];
          } else {
            rangingSuccessRate[i] = 0.0f;
          }

          failedRanging[i] = 0;
          succededRanging[i] = 0;
        }
      }


      if (lpsGetLppShort(&lppShortPacket)) {
        lpp_transaction = true;
        sendLppShort(dev, &lppShortPacket);
      } else {
        lpp_transaction = false;
        ranging_complete = false;
        initiateRanging(dev);
        //initiateCommunication(dev);
      }
      return MAX_TIMEOUT;
      break;
    case eventReceiveTimeout:
    case eventReceiveFailed:
      return 0;
      break;
    default:
      configASSERT(false);
  }

  return MAX_TIMEOUT;
}

static void twrTagInit(dwDevice_t *dev, lpsAlgoOptions_t* algoOptions)
{
  uint8_t i = 0;
  for (i = 0; i < LOCODECK_NR_OF_ANCHORS; i++){
	  anchor_order[i] = i;
  }
  options = algoOptions;
  //options->useTdma = false;
  //options->tdmaSlot = delta_p_slot;
  // Initialize the packet in the TX buffer
  memset(&txPacket, 0, sizeof(txPacket));
  MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
  txPacket.pan = 0xbccf;

  memset(&poll_tx, 0, sizeof(poll_tx));
  memset(&poll_rx, 0, sizeof(poll_rx));
  memset(&answer_tx, 0, sizeof(answer_tx));
  memset(&answer_rx, 0, sizeof(answer_rx));
  memset(&final_tx, 0, sizeof(final_tx));
  memset(&final_rx, 0, sizeof(final_rx));

  curr_seq = 0;
  current_anchor = 0;

  options->rangingState = 0;
  ranging_complete = false;

  tdmaSynchronized = false;

  memset(options->distance, 0, sizeof(options->distance));
  memset(options->pressures, 0, sizeof(options->pressures));
  memset(options->failedRanging, 0, sizeof(options->failedRanging));

  dwSetReceiveWaitTimeout(dev, TWR_RECEIVE_TIMEOUT);

  dwCommitConfiguration(dev);
  rangingOk = false;
  consoleCommPflush("lpsTwrTag init success0");
}

static bool isRangingOk()
{
  return rangingOk;
}

uwbAlgorithm_t uwbTwrTagAlgorithm = {
  .init = twrTagInit,
  .onEvent = twrTagOnEvent,
  .isRangingOk = isRangingOk,
};


/*LOG_GROUP_START(twrBeacons)
LOG_ADD(LOG_UINT32,  delta_b0, &delta_bs[0])
LOG_ADD(LOG_UINT32,  delta_b1, &delta_bs[1])
LOG_ADD(LOG_UINT32,  delta_b2, &delta_bs[2])
LOG_ADD(LOG_UINT32,  delta_b3, &delta_bs[3])
LOG_ADD(LOG_UINT32,  delta_b4, &delta_bs[4])
LOG_ADD(LOG_UINT32,  delta_b5, &delta_bs[5])
LOG_GROUP_STOP(twrBeacons)*/

LOG_GROUP_START(twrOther)
LOG_ADD(LOG_UINT32,  singleRanging, &singleRanging)
LOG_ADD(LOG_UINT32,  allRangings, &allRangings)
LOG_ADD(LOG_UINT32,  betweenRounds, &betweenRounds)
LOG_ADD(LOG_UINT32,  betweenRangings, &betweenRangings)
LOG_ADD(LOG_UINT8,  singleRanging_h8, &singleRanging_h8)
LOG_ADD(LOG_UINT8,  allRangings_h8, &allRangings_h8)
LOG_ADD(LOG_UINT8,  betweenRounds_h8, &betweenRounds_h8)
//LOG_ADD(LOG_UINT8,  betweenRangings_h8, &betweenRangings_h8)
LOG_GROUP_STOP(twrOther)


LOG_GROUP_START(twr)
LOG_ADD(LOG_UINT32,  t1, &ts[0])
LOG_ADD(LOG_UINT32,  t2, &ts[1])
LOG_ADD(LOG_UINT32,  t3, &ts[2])
LOG_ADD(LOG_UINT32,  t4, &ts[3])
LOG_ADD(LOG_UINT32,  t5, &ts[4])
LOG_ADD(LOG_UINT32,  delta_p, &delta_p)
LOG_ADD(LOG_UINT8,  delta_ph8, &delta_ph8)
LOG_GROUP_STOP(twr)


//what does eve know? assume receive times known. compare est TOF to real TOF
LOG_GROUP_START(twr_eve)
LOG_ADD(LOG_FLOAT,  est_tof_add, &est_tof_add)
LOG_ADD(LOG_FLOAT,  est_tof_mult, &est_tof_mult)
LOG_ADD(LOG_FLOAT,  actual_tof_add, &actual_tof_add)
LOG_ADD(LOG_FLOAT,  actual_tof_mult, &actual_tof_mult)
LOG_GROUP_STOP(twr_eve)

/* 
LOG_ADD(LOG_UINT8, rangingSuccessRate0, &rangingSuccessRate[0])
LOG_ADD(LOG_UINT8, rangingPerSec0, &rangingPerSec[0])
LOG_ADD(LOG_UINT8, rangingSuccessRate1, &rangingSuccessRate[1])
LOG_ADD(LOG_UINT8, rangingPerSec1, &rangingPerSec[1])
LOG_ADD(LOG_UINT8, rangingSuccessRate2, &rangingSuccessRate[2])
LOG_ADD(LOG_UINT8, rangingPerSec2, &rangingPerSec[2])
LOG_ADD(LOG_UINT8, rangingSuccessRate3, &rangingSuccessRate[3])
LOG_ADD(LOG_UINT8, rangingPerSec3, &rangingPerSec[3])
LOG_ADD(LOG_UINT8, rangingSuccessRate4, &rangingSuccessRate[4])
LOG_ADD(LOG_UINT8, rangingPerSec4, &rangingPerSec[4])
LOG_ADD(LOG_UINT8, rangingSuccessRate5, &rangingSuccessRate[5])
LOG_ADD(LOG_UINT8, rangingPerSec5, &rangingPerSec[5])*/

/*
LOG_GROUP_START(twr_time)
LOG_ADD(LOG_UINT32,  t1, &ts[0])
LOG_ADD(LOG_UINT32,  t2, &ts[1])
LOG_ADD(LOG_UINT32,  t3, &ts[2])
LOG_ADD(LOG_UINT32,  t4, &ts[3])
LOG_ADD(LOG_UINT32,  t5, &ts[4])
LOG_ADD(LOG_UINT32,  t6, &ts[5])
LOG_GROUP_STOP(twr_time)
*/