/*
 * Copyright (c) 2011, SimpleMesh AUTHORS
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

#ifndef _HAL_I2C_H_
#define _HAL_I2C_H_

/*****************************************************************************
*****************************************************************************/

/*****************************************************************************
*****************************************************************************/
typedef struct HAL_I2C_t
{
  //uint8_t   bits;
  //uint8_t   parity;
  //uint8_t   stop;
  uint32_t  baudrate; 

  uint8_t   *txBuffer;
  uint16_t  txBufferSize;

  uint8_t   *rxBuffer;
  uint16_t  rxBufferSize;
  void      (*rxCallback)(uint16_t);
  void      (*txCallback)(void);
} HAL_I2C_t;

/*****************************************************************************
*****************************************************************************/
void HAL_I2CInit(HAL_Uart_t *uart);
void HAL_UartWriteByte(uint8_t byte);
uint8_t HAL_UartReadByte(void);
uint16_t HAL_UartGetFreeSize(void);
void HAL_UartBytesReceived(uint16_t bytes);
void HAL_UartTaskHandler(void);

#endif // _HAL_UART_H_

