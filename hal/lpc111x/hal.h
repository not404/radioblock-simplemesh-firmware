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
 
#ifndef _HAL_H_
#define _HAL_H_

#include <LPC11xx.h>
#include <core_cm0.h>
#include "compiler.h"

/*****************************************************************************
*****************************************************************************/
#define ATOMIC_SECTION_ENTER   { volatile uint32_t __atomic = __get_PRIMASK(); __disable_irq();
#define ATOMIC_SECTION_LEAVE   __set_PRIMASK(__atomic); }

#define halPhySlpTrSet()       LPC_GPIO1->MASKED_ACCESS[(1 << 4)] = (1 << 4);
#define halPhySlpTrClr()       LPC_GPIO1->MASKED_ACCESS[(1 << 4)] = (0 << 4);
#define halPhySlpTrOut()       LPC_GPIO1->DIR |= (1 << 4)
#define halPhySlpTrIn()        LPC_GPIO1->DIR &= ~(1 << 4)

#define halPhyRstSet()         LPC_GPIO1->MASKED_ACCESS[(1 << 11)] = (1 << 11);
#define halPhyRstClr()         LPC_GPIO1->MASKED_ACCESS[(1 << 11)] = (0 << 11);
#define halPhyRstOut()         LPC_GPIO1->DIR |= (1 << 11)
#define halPhyRstIn()          LPC_GPIO1->DIR &= ~(1 << 11)

#define halPhyIrqSet()         LPC_GPIO1->MASKED_ACCESS[(1 << 2)] = (1 << 2);
#define halPhyIrqClr()         LPC_GPIO1->MASKED_ACCESS[(1 << 2)] = (0 << 2);
#define halPhyIrqOut()         LPC_GPIO1->DIR |= (1 << 2)
#define halPhyIrqIn()          LPC_GPIO1->DIR &= ~(1 << 2)

#define halPhySsSet()          LPC_GPIO0->MASKED_ACCESS[(1 << 2)] = (1 << 2);
#define halPhySsClr()          LPC_GPIO0->MASKED_ACCESS[(1 << 2)] = (0 << 2);
#define halPhySsOut()          LPC_GPIO0->DIR |= (1 << 2)
#define halPhySsIn()           LPC_GPIO0->DIR &= ~(1 << 2)

#define halPhyMisoIn()         LPC_GPIO0->DIR &= ~(1 << 8)
#define halPhyMosiOut()        LPC_GPIO0->DIR |= (1 << 9)
#define halPhySckOut()         LPC_GPIO2->DIR |= (1 << 11)

#define SSPSR_TNF              (1 << 1)
#define SSPSR_RNE              (1 << 2)
#define SSPSR_BSY              (1 << 4)

/*****************************************************************************
*****************************************************************************/
void HAL_Init(void);
uint16_t HAL_GetElapsedTime(void);

uint8_t HAL_PhySpiWriteByte(uint8_t value);
void HAL_PhyReset(void);

/*****************************************************************************
*****************************************************************************/
INLINE uint8_t HAL_PhySpiWriteByteInline(uint8_t value)
{
  while ((LPC_SSP0->SR & (SSPSR_BSY | SSPSR_TNF)) != SSPSR_TNF);
  LPC_SSP0->DR = value;
  while ((LPC_SSP0->SR & (SSPSR_BSY | SSPSR_RNE)) != SSPSR_RNE);
  return LPC_SSP0->DR;
}

/*****************************************************************************
*****************************************************************************/
INLINE void HAL_PhySpiSelect(void)
{
  halPhySsClr();
}

/*****************************************************************************
*****************************************************************************/
INLINE void HAL_PhySpiDeselect(void)
{
  halPhySsSet();
}

/*****************************************************************************
*****************************************************************************/
INLINE void HAL_PhySlpTrAssert(void)
{
  halPhySlpTrSet();
  halPhySlpTrClr();
}

#endif // _HAL_H_

