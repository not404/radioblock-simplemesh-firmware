/*
 * Copyright (c) 2011 - 2012, SimpleMesh AUTHORS
 * Eric Gnoske,
 * Colin O'Flynn
 * Blake Leverett,
 * Rob Fries,
 * Colorado Micro Devices Inc..
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1) Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *
 *   2) Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 *   3) Neither the name of the SimpleMesh AUTHORS nor the names of its contributors
 *       may be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _HAL_UART_H_
#define _HAL_UART_H_

#include "sysTypes.h"

/*****************************************************************************
*****************************************************************************/
#define HAL_UART_BAUDRATE(dl, div, mul) ((mul << 20) | (div << 16) | dl)

/*****************************************************************************
*****************************************************************************/
enum
{
	/*UCSZ01 UCSZ00*/
  HAL_UART_BITS_5 = (1 << 0),
  HAL_UART_BITS_6 = (1 << 1),
  HAL_UART_BITS_7 = (2 << 1),
  HAL_UART_BITS_8 = ((3 << 1) | (2 << 1)),
};

enum
{
	/*USBS1*/
  HAL_UART_STOP_BITS_1 = (0 << 3),
  HAL_UART_STOP_BITS_2 = (1 << 3),
};

enum
{
	/*UPM11:10*/
  HAL_UART_PARITY_NONE    = ((0 << 4) | (0 << 5)),
  HAL_UART_PARITY_ODD     = ((1 << 4) | (1 << 5)),
  HAL_UART_PARITY_EVEN    = ((0 << 4) | (1 << 5)),
};

// If FOSC = 16MHz
enum
{
  HAL_UART_BAUDRATE_AUTO   = 0,
  HAL_UART_BAUDRATE_50     = 0,
  HAL_UART_BAUDRATE_75     = 0,
  HAL_UART_BAUDRATE_110    = 0,
  HAL_UART_BAUDRATE_150    = 0,
  HAL_UART_BAUDRATE_300    = 0,
  HAL_UART_BAUDRATE_1200   = 0,
  HAL_UART_BAUDRATE_2400   = 0,
  HAL_UART_BAUDRATE_4800   = 0,
  HAL_UART_BAUDRATE_9600   = 103,
  HAL_UART_BAUDRATE_19200  = 51, 
  HAL_UART_BAUDRATE_38400  = 25,
  HAL_UART_BAUDRATE_57600  = 16,
  HAL_UART_BAUDRATE_115200 = 8,
  HAL_UART_BAUDRATE_230400 = 0,
  HAL_UART_BAUDRATE_460800 = 0,
};



typedef struct HAL_Uart_t
{
  uint8_t   bits;
  uint8_t   parity;
  uint8_t   stop;
  uint32_t  baudrate;

  uint8_t   *txBuffer;
  uint16_t  txBufferSize;

  uint8_t   *rxBuffer;
  uint16_t  rxBufferSize;
  void      (*rxCallback)(uint16_t);
  void      (*txCallback)(void);
} HAL_Uart_t;

/*****************************************************************************
*****************************************************************************/
void HAL_UartInit(HAL_Uart_t *uart);
void HAL_UartClose(void);
void HAL_UartWriteByte(uint8_t byte);
uint8_t HAL_UartReadByte(void);
uint16_t HAL_UartGetFreeSize(void);
void HAL_UartBytesReceived(uint16_t bytes);
void HAL_UartTaskHandler(void);

// ETG To make AVR Stuido happy
void HAL_UART_IrqHandler(void);
void halUartRxTaskHandler(void);
void halUartTxTaskHandler(void);
#endif // _HAL_UART_H_

