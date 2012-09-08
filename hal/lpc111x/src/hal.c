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
#include "config.h"
#include "halTimer.h"
#include "halPhy.h"
#include "halUart.h"
#include "sysTaskManager.h"
#include "sysTypes.h"

/*****************************************************************************
*****************************************************************************/
void HAL_Init(void)
{
  LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16/*IOCON*/);
  LPC_SYSCON->SYSAHBCLKDIV = 1;

  HAL_TimerInit();
  HAL_PhyInit();

  __enable_irq();
}

/*****************************************************************************
*****************************************************************************/
void HAL_WarmReset(void)
{
  LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 15/*WDT*/);
  LPC_SYSCON->WDTCLKDIV = 1;
  LPC_SYSCON->WDTCLKSEL = (0/*IRC oscillator*/ << 0/*SEL*/);
  LPC_SYSCON->WDTCLKUEN = (1 << 0/*ENA*/);
  LPC_SYSCON->WDTCLKUEN = (0 << 0/*ENA*/);
  LPC_SYSCON->WDTCLKUEN = (1 << 0/*ENA*/);

  LPC_WDT->MOD = (1 << 0/*WDEN*/) | (1 << 1/*WDRESET*/);
  LPC_WDT->FEED = 0xaa;
  LPC_WDT->FEED = 0x55;

  while (1);
}

