/*
 * beaconComm.c
 *
 *  Created on: Jul 3, 2018
 *      Author: bitcraze
 *
 *      Intended to communicate between drone and beacon
 */
/* USES THE FOLLOWING FUNCTIONS FROM LOCODECK.C
 *
bool lpsSendLppShort(uint8_t destId, void* data, size_t length)
{
  bool result = false;

  if (isInit)
  {
    lppShortPacket.dest = destId;
    lppShortPacket.length = length;
    memcpy(lppShortPacket.data, data, length);
    result = xQueueSend(lppShortQueue, &lppShortPacket,0) == pdPASS;
  }

  return result;
}
(if 1 then got packet, else did not)
bool lpsGetLppShort(lpsLppShortPacket_t* shortPacket)
{
  return xQueueReceive(lppShortQueue, shortPacket, 0) == pdPASS;
}
(check isInit to see if everything good)
static bool dwm1000Test()
{
  if (!isInit) {
    DEBUG_PRINT("Error while initializing DWM1000\n");
  }

  return isInit;
}
 *
 */

#include "locodeck.h"


