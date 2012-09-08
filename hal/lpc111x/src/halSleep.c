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

#include "sysTypes.h"
#include "hal.h"

/*****************************************************************************
*****************************************************************************/
void HAL_Sleep(uint32_t time)
{
  uint32_t sysAhbClkCtrl;

  sysAhbClkCtrl = LPC_SYSCON->SYSAHBCLKCTRL;
  LPC_SYSCON->SYSAHBCLKCTRL = (1 << 0/*SYS*/) | (1 << 1/*ROM*/) | (1 << 2/*RAM*/) |
      (1 << 3/*FLASHREQ*/) | (1 << 4/*FLASHARRAY*/) |  (1 << 6/*GPIO*/) |
      (1 << 9/*CT32B0*/) | (1 << 16/*IOCON*/);

  LPC_IOCON->R_PIO0_11 = (3 << 0/*CT32B0_MAT3*/) | (1 << 7/*ADMODE*/);

  LPC_SYSCON->STARTAPRP0 |= (1 << 11/*Rising edge*/);
  LPC_SYSCON->STARTRSRP0CLR |= (1 << 11/*Reset start signal*/);
  LPC_SYSCON->STARTERP0 |= (1 << 11/*Wakeup source*/);

  LPC_SYSCON->PDRUNCFG &= ~(1 << 6/*WDTOSC_PD*/);
  LPC_SYSCON->PDAWAKECFG = LPC_SYSCON->PDRUNCFG;

  LPC_SYSCON->WDTOSCCTRL = (0x1f/*64*/ << 0/*DIVSEL*/) | (1/*0.5 MHz*/ << 5/*FREQ*/);

  LPC_SYSCON->MAINCLKSEL = (2/*WDT oscillator*/ << 0/*SEL*/);
  LPC_SYSCON->MAINCLKUEN = (1 << 0/*ENA*/);
  LPC_SYSCON->MAINCLKUEN = (0 << 0/*ENA*/);
  LPC_SYSCON->MAINCLKUEN = (1 << 0/*ENA*/);

  while (!(LPC_SYSCON->MAINCLKUEN & (1 << 0/*ENA*/)));

  LPC_TMR32B0->MR3 = (time * 125) / 16; // time * (500kHz / 64)
  LPC_TMR32B0->MCR |= (1 << 9/*MR3I*/) | (1 << 10/*MR3R*/);
  LPC_TMR32B0->EMR |= (2 << 10/*set EMC3 high*/);
  LPC_TMR32B0->TCR |= (1 << 0/*CEn*/);
  NVIC_EnableIRQ(WAKEUP11_IRQn);

  SCB->SCR |= (1 << 2/*SLEEPDEEP*/);
  asm volatile ("wfi");

  LPC_SYSCON->SYSAHBCLKCTRL = sysAhbClkCtrl;
}

/*****************************************************************************
*****************************************************************************/
void HAL_WAKEUP0_11_IrqHandler(void)
{
  SCB->SCR &= ~(1 << 2/*SLEEPDEEP*/);

  LPC_SYSCON->MAINCLKSEL = (0/*IRC oscillator*/ << 0/*SEL*/);
  LPC_SYSCON->MAINCLKUEN = (1 << 0/*ENA*/);
  LPC_SYSCON->MAINCLKUEN = (0 << 0/*ENA*/);
  LPC_SYSCON->MAINCLKUEN = (1 << 0/*ENA*/);
  while (!(LPC_SYSCON->MAINCLKUEN & (1 << 0/*ENA*/)));

  LPC_TMR32B0->EMR = 0;
  LPC_TMR32B0->TCR = 0;
  LPC_TMR32B0->MR3 = 0;
  LPC_TMR32B0->MCR = 0;

  LPC_SYSCON->STARTRSRP0CLR |= (1 << 11);

  NVIC_DisableIRQ(WAKEUP11_IRQn);
}

