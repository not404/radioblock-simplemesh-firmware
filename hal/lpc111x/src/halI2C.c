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

#include <stdlib.h>
#include <stdbool.h>
#include "hal.h"
#include "halI2C.h"
#include "config.h"
#include "sysTaskManager.h"

//#ifdef HAL_I2C_ENABLE

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
static HAL_I2C_t *halI2C = NULL;

/*****************************************************************************
*****************************************************************************/
void HAL_I2CInit(HAL_I2C_t *i2c)
{
  txFifo.data = i2c->txBuffer;
  txFifo.size = i2c->txBufferSize;
  txFifo.bytes = 0;
  txFifo.head = 0;
  txFifo.tail = 0;

  rxFifo.data = i2c->rxBuffer;
  rxFifo.size = i2c->rxBufferSize;
  rxFifo.bytes = 0;
  rxFifo.head = 0;
  rxFifo.tail = 0;

  halI2C = i2c;

  //Enable I2C Interrupt
  NVIC_EnableIRQ(I2C_IRQn);
  LPC_I2C->CONSET = 0x40;

}

/*****************************************************************************
*****************************************************************************/
void HAL_UartClose(void)
{
  if (NULL != halUart)
  {
    NVIC_DisableIRQ(UART_IRQn);
    LPC_UART->IER = 0;
    LPC_SYSCON->SYSAHBCLKCTRL &= ~(1 << 12);
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
void HAL_UART_IrqHandler(void)
{
  uint8_t dummy, id;
  bool rx = false;

  id = (LPC_UART->IIR >> 1) & 7;

  if (0x03 == id) // Receive Line Status (RLS)
  {
    uint8_t lsr = LPC_UART->LSR;

    if (lsr & ((1 << 1/*OE*/) | (1 << 2/*PE*/) | (1 << 3/*FE*/) | (1 << 4/*BI*/) | (1 << 7/*RXFE*/)))
    {
      dummy = LPC_UART->RBR;
      (void)dummy;
      return;
    }

    rx = true;
  }
  else if (0x02 == id) // Receive Data Available (RDA)
  {
    rx = true;
  }
  else if (0x06 == id) // Character Time-out Indicator (CTI)
  {
    rx = true;
  }
  else if (0x01 == id) // THRE Interrupt
  {
    if (LPC_UART->LSR & (1 << 5/*THRE*/))
      SYS_TaskSet(HAL_UART_TX_TASK);
  }

  if (rx)
  {
    while (LPC_UART->LSR & (1 << 0/*RDR*/))
    {
      if (rxFifo.bytes == rxFifo.size)
        continue;

      rxFifo.data[rxFifo.tail++] = LPC_UART->RBR;
      if (rxFifo.tail == rxFifo.size)
        rxFifo.tail = 0;
      rxFifo.bytes++;
    }

    SYS_TaskSet(HAL_UART_RX_TASK);
  }
}

/*****************************************************************************
*****************************************************************************/
void halUartTxTaskHandler(void)
{
  if (txFifo.bytes)
  {
    if ((LPC_UART->LSR & (1 << 5/*THRE*/)))
    {
      uint8_t byte;

      byte = txFifo.data[txFifo.head++];
      if (txFifo.head == txFifo.size)
        txFifo.head = 0;
      txFifo.bytes--;

      LPC_UART->THR = byte;
    }
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

#endif // HAL_UART_ENABLE

