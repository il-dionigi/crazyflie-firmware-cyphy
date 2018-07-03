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
 * droneComm.h - Used to send drone data to the client (will soon go to other drone)
 * ~CYPHY~
 */

#ifndef beaconComm_H_
#define beaconComm_H_

#include <stdbool.h>
#include "eprintf.h"

/**
 * Initialize the beaconComm
 */
void beaconCommInit(void);

bool beaconCommTest(void);

/**
 * Put a character to the beaconComm buffer
 *
 * @param ch character that shall be printed
 * @return The character casted to unsigned int or EOF in case of error
 */
int beaconCommPutchar(int ch);

/**
 * Put a character to the beaconComm buffer
 *
 * @param ch character that shall be printed
 * @return The character casted to unsigned int or EOF in case of error
 *
 * @note This version can be called by interrup. In such case the internal
 * buffer is going to be used. If a task currently is printing or if the
 * interrupts prints too much the data will be ignored.
 */
int beaconCommPutcharFromISR(int ch);

/**
 * Put a null-terminated string on the beaconComm buffer
 *
 * @param str Null terminated string
 * @return a nonnegative number on success, or EOF on error.
 */
int beaconCommPuts(char *str);

/**
 * Flush the beaconComm buffer
 */
void beaconCommFlush(void);


//puts + flush
void beaconCommPflush(char *);

//pass in data from  beacon, interpret it here
void beaconAnalyzePayload(char * data);

/**
 * Macro implementing beaconCommPrintf with eprintf
 *
 * @param FMT String format
 * @patam ... Parameters to print
 */

#define beaconCommPrintf(FMT, ...) eprintf(beaconCommPutchar, FMT, ## __VA_ARGS__)

#endif /*beaconComm_H_*/
