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

#ifdef HAL_ATMEGA128RFA1

#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "hal.h"
#include "sysTaskManager.h"

/*****************************************************************************
*****************************************************************************/
#define TIMER_PRESCALER     8
#define TIMER_INTERVAL      10ul // ms

/*****************************************************************************
*****************************************************************************/
static volatile uint8_t timerIrqCount = 0ul;

/*****************************************************************************
*****************************************************************************/
static void halTimerInit(void)
{
  OCR4A = ((F_CPU / 1000ul) / TIMER_PRESCALER) * TIMER_INTERVAL;
  TCCR4B = (1 << WGM12);              // CTC mode
  TCCR4B |= (1 << CS11);              // Prescaler 8
  TIMSK4 |= (1 << OCIE4A);            // Enable TC4 interrupt
}

/*****************************************************************************
*****************************************************************************/
ISR(TIMER4_COMPA_vect)
{
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
void HAL_Init(void)
{
  halTimerInit();
  sei();
}

#endif // HAL_ATMEGA128RFA1

