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

#ifndef droneComm_H_
#define droneComm_H_

#include <stdbool.h>
#include "eprintf.h"

/**
 * Initialize the droneComm
 */
void droneCommInit(void);

bool droneCommTest(void);

/**
 * Put a character to the droneComm buffer
 *
 * @param ch character that shall be printed
 * @return The character casted to unsigned int or EOF in case of error
 */
int droneCommPutchar(int ch);

/**
 * Put a character to the droneComm buffer
 *
 * @param ch character that shall be printed
 * @return The character casted to unsigned int or EOF in case of error
 *
 * @note This version can be called by interrup. In such case the internal
 * buffer is going to be used. If a task currently is printing or if the
 * interrupts prints too much the data will be ignored.
 */
int droneCommPutcharFromISR(int ch);

/**
 * Put a null-terminated string on the droneComm buffer
 *
 * @param str Null terminated string
 * @return a nonnegative number on success, or EOF on error.
 */
int droneCommPuts(char *str);

/**
 * Flush the droneComm buffer
 */
void droneCommFlush(void);

/**
 * Macro implementing droneCommPrintf with eprintf
 *
 * @param FMT String format
 * @patam ... Parameters to print
 */
#define droneCommPrintf(FMT, ...) eprintf(droneCommPutchar, FMT, ## __VA_ARGS__)

#endif /*droneComm_H_*/
