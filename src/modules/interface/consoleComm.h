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
 * consoleComm.h - Used to send drone data to the client (will soon go to other drone)
 * ~CYPHY~
 */

#ifndef consoleComm_H_
#define consoleComm_H_

#include <stdbool.h>
#include "eprintf.h"

typedef enum { // can only go up to 0x03
    C2RTP_CHANNEL_TEXT    = 0x00,
    C2RTP_CHANNEL_SWITCH  = 0x01,
} C2RTPChannel;

void saveRadioAddress(uint64_t address);
void saveRadioChannel(uint8_t channel);
void saveRadioDatarate(uint8_t datarate);
void displayRadioAddress(void);
void displayRadioChannel(void);
void displayRadioDatarate(void);
void writeDroneData(char * str, int len);

/**
 * Initialize the consoleComm
 */
void consoleCommInit(void);

bool consoleCommTest(void);

/**
 * Put a character to the consoleComm buffer
 *
 * @param ch character that shall be printed
 * @return The character casted to unsigned int or EOF in case of error
 */
int consoleCommPutchar(int ch);

/**
 * Put a character to the consoleComm buffer
 *
 * @param ch character that shall be printed
 * @return The character casted to unsigned int or EOF in case of error
 *
 * @note This version can be called by interrup. In such case the internal
 * buffer is going to be used. If a task currently is printing or if the
 * interrupts prints too much the data will be ignored.
 */
int consoleCommPutcharFromISR(int ch);

/**
 * Put a null-terminated string on the consoleComm buffer
 *
 * @param str Null terminated string
 * @return a nonnegative number on success, or EOF on error.
 */
int consoleCommPuts(char *str);

/**
 * Flush the consoleComm buffer
 */
void consoleCommFlush(void);

//puts + flush
void consoleCommPflush(char *);

/**
 * Macro implementing consoleCommPrintf with eprintf
 *
 * @param FMT String format
 * @patam ... Parameters to print
 */
#define consoleCommPrintf(FMT, ...) eprintf(consoleCommPutchar, FMT, ## __VA_ARGS__)

#endif /*consoleComm_H_*/
