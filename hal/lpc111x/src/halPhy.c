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
 *   3) Neither the name of the FIP AUTHORS nor the names of its contributors
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "hal.h"
#include "phy.h"
#include "sysTaskManager.h"
#include "sysTypes.h"

/*****************************************************************************
*****************************************************************************/
uint8_t HAL_PhySpiWriteByte(uint8_t value)
{
  return HAL_PhySpiWriteByteInline(value);
}

/*****************************************************************************
*****************************************************************************/
inline static void halDelay(uint8_t us)
{
  for (uint8_t i = 0; i < us; i++) // TODO: do better than this
    asm("nop");
}

/*****************************************************************************
*****************************************************************************/
void HAL_PhyReset(void)
{
  halPhyRstClr();
  halDelay(10);
  halPhyRstSet();
}

/*****************************************************************************
*****************************************************************************/
void HAL_PhyInit(void)
{
  halPhySlpTrOut();
  halPhySlpTrClr();

  halPhyRstOut();
  halPhyRstSet();

  halPhyIrqIn();
  halPhyIrqClr();

  halPhySsOut();
  halPhySsSet();
  halDelay(255);
  halDelay(255);

  halPhyMisoIn();
  halPhyMosiOut();
  halPhySckOut();

  LPC_SYSCON->PRESETCTRL |= (1 << 0);          // Release SSP0 reset
  LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 11);      // Enable SSP0 clock
  LPC_SYSCON->SSP0CLKDIV = 1;

  LPC_IOCON->PIO0_8 = (1 << 0);                // MISO SSP0
  LPC_IOCON->PIO0_9 = (1 << 0);                // MOSI SSP0
  LPC_IOCON->SCK_LOC = (2 << 0);               // SCK SSP0 at PIO0.6
  LPC_IOCON->PIO0_6 = (2 << 0);                // SCK SSP0

  LPC_SSP0->CPSR = 2;                          // SSP0 prescaler
  LPC_SSP0->CR0 = (7 << 0/*DSS*/);             // 8-bit
  LPC_SSP0->CR1 = (1 << 1/*SSE*/);             // Enable SSP0 master

  LPC_GPIO1->IEV |= (1 << 2);                  // Interrupt on rising edge
  LPC_GPIO1->IE |= (1 << 2);                   // Enable interrupt
  NVIC_EnableIRQ(EINT1_IRQn);
}

/*****************************************************************************
*****************************************************************************/
void HAL_PIOINT1_IrqHandler(void)
{
  LPC_GPIO1->IC = (1 << 2);
  phyInterruptHandler();
}

