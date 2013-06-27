/*
 * Copyright (c) 2011 - 2012, SimpleMesh AUTHORS
 * Eric Gnoske,
 * Colin O'Flynn,
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

#include <stdlib.h>
#include <stdbool.h>
#include "hal.h"
#include "halUart.h"
#include "config.h"
#include "sysTaskManager.h"
#include "sysTypes.h"
//#include <asf.h>
#include <avr/io.h>
#include <avr/interrupt.h>

/*****************************************************************************
*****************************************************************************/
typedef struct
{
  uint16_t  head;
  uint16_t  tail;
  uint16_t  size;
  uint16_t  bytes;
  uint8_t   *data;
} FifoBuffer_t;

/*****************************************************************************
*****************************************************************************/
static FifoBuffer_t txFifo;
static volatile FifoBuffer_t rxFifo;
static HAL_Uart_t *halUart = NULL;

/*****************************************************************************
*****************************************************************************/
void HAL_UartInit(HAL_Uart_t *uart)
{
  txFifo.data = uart->txBuffer;
  txFifo.size = uart->txBufferSize;
  txFifo.bytes = 0;
  txFifo.head = 0;
  txFifo.tail = 0;

  rxFifo.data = uart->rxBuffer;
  rxFifo.size = uart->rxBufferSize;
  rxFifo.bytes = 0;
  rxFifo.head = 0;
  rxFifo.tail = 0;

  halUart = uart;
  
  /* 
    Just use USART 1, PD2 (Rx) and PD3 (Tx). 
    We are running from the external 16MHz crystal so 
    no need to initialize the uart clock...?
    
    sysclk_enable_module(POWER_RED_REG1, PRUSART1_bm);
    
    And, no need to include <asf.h> stuff.
  */
   
  // Turn on interrupts.
  UCSR1B = ((1 << RXCIE1) | (1 << TXCIE1));
  
  // Running at 8MHz, almost all BAUD settings have error rates.
  UBRR1 = halUart->baudrate; //HAL_UART_BAUDRATE_115200;
  
  // Setup the number of bits, start and stop - 8N1.
  UCSR1C = ((1 << UCSZ11) | ( 1<< UCSZ10));
  
  // Enable.
  UCSR1B |= ((1 << RXEN1) | (1 << TXEN1));
    
  //sei();
}

/*****************************************************************************
*****************************************************************************/
void HAL_UartClose(void)
{
  if (NULL != halUart)
  {
		UCSR1B = 0;
    halUart = NULL;
    
  }
}

/*****************************************************************************
*****************************************************************************/
void HAL_UartWriteByte(uint8_t byte)
{
  if (txFifo.bytes == txFifo.size)
    return;

  txFifo.data[txFifo.tail++] = byte;
  if (txFifo.tail == txFifo.size)
    txFifo.tail = 0;
  txFifo.bytes++;

  SYS_TaskSet(HAL_UART_TX_TASK);
}

/*****************************************************************************
*****************************************************************************/
uint8_t HAL_UartReadByte(void)
{
  uint8_t byte;

  ATOMIC_SECTION_ENTER
    byte = rxFifo.data[rxFifo.head++];
    if (rxFifo.head == rxFifo.size)
      rxFifo.head = 0;
    rxFifo.bytes--;
  ATOMIC_SECTION_LEAVE

  return byte;
}

/*****************************************************************************
*****************************************************************************/
uint16_t HAL_UartGetFreeSize(void)
{
  return (txFifo.size - txFifo.bytes);
}

/*****************************************************************************
*****************************************************************************/
uint8_t bull[128];
uint8_t bugbuf[32];

ISR(USART1_RX_vect)
{
//  uint8_t tmp;
//  static uint8_t i=0;
//  static uint8_t zap = 0;
  
//  bugbuf[zap++] = 10;
//  while (UCSR1A & (1<<RXC1)) // While there is unread data...
//  {
    if (rxFifo.bytes == rxFifo.size)
      //continue;
      return;

//		tmp = UDR1;
//    bugbuf[zap++] = 11;
    
    rxFifo.data[rxFifo.tail++] = UDR1;
//    bull[i++] = tmp;
    if (rxFifo.tail == rxFifo.size)
      rxFifo.tail = 0;
    rxFifo.bytes++;
//  }

//  bugbuf[zap++] = 12;
  SYS_TaskSet(HAL_UART_RX_TASK);

}

ISR(USART1_TX_vect)
{
	SYS_TaskSet(HAL_UART_TX_TASK);
}
/*****************************************************************************
*****************************************************************************/
void halUartTxTaskHandler(void)
{
  if (txFifo.bytes)
  {
      uint8_t byte;

      byte = txFifo.data[txFifo.head++];
      if (txFifo.head == txFifo.size)
        txFifo.head = 0;
      txFifo.bytes--;

      UDR1 = byte;
  }
  else
  {
    if (NULL != halUart->txCallback)
      halUart->txCallback();
  }
}

/*****************************************************************************
*****************************************************************************/
void halUartRxTaskHandler(void)
{
  uint16_t bytes;

  ATOMIC_SECTION_ENTER
    bytes = rxFifo.bytes;
  ATOMIC_SECTION_LEAVE

  if (NULL != halUart->rxCallback)
    halUart->rxCallback(bytes);
}

