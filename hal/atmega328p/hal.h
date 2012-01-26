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

#include <avr/io.h>
#include <avr/interrupt.h>

/*****************************************************************************
*****************************************************************************/
#define ATOMIC_SECTION_ENTER   { uint8_t __atomic = SREG; cli();
#define ATOMIC_SECTION_LEAVE   SREG = __atomic; }

#define INLINE                 static inline __attribute__ ((always_inline))

#define halPhySlpTrSet()       PORTC |= (1 << 4)
#define halPhySlpTrClr()       PORTC &= ~(1 << 4)
#define halPhySlpTrOut()       DDRC |= (1 << 4)
#define halPhySlpTrIn()        DDRC &= ~(1 << 4)

#define halPhyRstSet()         PORTC |= (1 << 5)
#define halPhyRstClr()         PORTC &= ~(1 << 5)
#define halPhyRstOut()         DDRC |= (1 << 5)
#define halPhyRstIn()          DDRC &= ~(1 << 5)

#define halPhyIrqSet()         PORTB |= (1 << 1)
#define halPhyIrqClr()         PORTB &= ~(1 << 1)
#define halPhyIrqOut()         DDRB |= (1 << 1)
#define halPhyIrqIn()          DDRB &= ~(1 << 1)

#define halPhySsSet()          PORTB |= (1 << 2)
#define halPhySsClr()          PORTB &= ~(1 << 2)
#define halPhySsOut()          DDRB |= (1 << 2)
#define halPhySsIn()           DDRB &= ~(1 << 2)

#define halPhyMisoIn()         DDRB &= ~(1 << 4)
#define halPhyMosiOut()        DDRB |= (1 << 3)
#define halPhySckOut()         DDRB |= (1 << 5)

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
  SPDR = value;
  while (!(SPSR & (1 << SPIF)));
  return SPDR;
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

