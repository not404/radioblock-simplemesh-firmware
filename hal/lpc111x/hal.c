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

#ifdef HAL_LPC111X

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "hal.h"
#include "phy.h"
#include "sysTaskManager.h"
#include "compiler.h"

/*****************************************************************************
*****************************************************************************/
#define TIMER_INTERVAL      10ul

/*****************************************************************************
*****************************************************************************/
static volatile uint8_t timerIrqCount = 0ul;

/*****************************************************************************
*****************************************************************************/
static void halTimerInit(void)
{
  LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 7/*CT16B0*/);

  LPC_TMR16B0->PR  = F_CPU / 1000000;
  LPC_TMR16B0->MR0 = TIMER_INTERVAL * 1000;
  LPC_TMR16B0->MCR = (1 << 0) | (1 << 1); // interrupt and reset on MR0 match
  LPC_TMR16B0->TCR = (1 << 0); // enable timer

  NVIC_EnableIRQ(TIMER_16_0_IRQn);
}

/*****************************************************************************
*****************************************************************************/
void TIMER16_0_IRQHandler(void)
{
  LPC_TMR16B0->IR = 1;
  timerIrqCount++;
  SYS_TaskSetInline(SYS_TIMER_TASK);
}

/*****************************************************************************
*****************************************************************************/
uint16_t HAL_GetElapsedTime(void)
{
  uint8_t cnt;

  ATOMIC_SECTION_ENTER
    cnt = timerIrqCount;
    timerIrqCount = 0;
  ATOMIC_SECTION_LEAVE

  return cnt * TIMER_INTERVAL;
}

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
void halPhyInit(void)
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
  LPC_IOCON->SCK_LOC = (1 << 0);               // SCK SSP0 at PIO2.11
  LPC_IOCON->PIO2_11 = (1 << 0);               // SCK SSP0

  LPC_SSP0->CPSR = 2;                          // SSP0 prescaler
  LPC_SSP0->CR0 = (7 << 0/*DSS*/);             // 8-bit
  LPC_SSP0->CR1 = (1 << 1/*SSE*/);             // Enable SSP0 master

  LPC_GPIO1->IEV |= (1 << 2);                  // Interrupt on rising edge
  LPC_GPIO1->IE |= (1 << 2);                   // Enable interrupt
  NVIC_EnableIRQ(EINT1_IRQn);
}

/*****************************************************************************
*****************************************************************************/
void PIOINT1_IRQHandler(void)
{
  LPC_GPIO1->IC = (1 << 2);
  phyInterruptHandler();
}

/*****************************************************************************
*****************************************************************************/
void HAL_Init(void)
{
  LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16/*IOCON*/);
  LPC_SYSCON->SYSAHBCLKDIV = 1;

  halTimerInit();
  halPhyInit();

  __enable_irq();
}

#endif // HAL_LPC111X

