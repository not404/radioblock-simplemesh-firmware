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
 
#include <stdlib.h>
#include "sysTimer.h"
#include "assert.h"
#include "hal.h"

/*****************************************************************************
*****************************************************************************/
static SYS_Timer_t *timers = NULL;

/*****************************************************************************
*****************************************************************************/
static void placeTimer(SYS_Timer_t *timer)
{
  if (timers)
  {
    SYS_Timer_t *prev = NULL;
    uint32_t timeout = timer->interval;

    for (SYS_Timer_t *t = timers; t; t = t->next)
    {
      if (timeout < t->timeout)
      {
         t->timeout -= timeout;
         break;
      }
      else
        timeout -= t->timeout;

      prev = t;
    }

    timer->timeout = timeout;

    if (prev)
    {
      timer->next = prev->next;
      prev->next = timer;
    }
    else
    {
      timer->next = timers;
      timers = timer;
    }
  }
  else
  {
    timer->next = NULL;
    timer->timeout = timer->interval;
    timers = timer;
  }
}

/*****************************************************************************
*****************************************************************************/
void sysTimerTaskHandler(void)
{
  uint32_t elapsed;

  elapsed = HAL_GetElapsedTime();

  while (timers && (timers->timeout <= elapsed))
  {
    SYS_Timer_t *timer = timers;

    elapsed -= timers->timeout;
    timers = timers->next;
    if (SYS_TIMER_PERIODIC_MODE == timer->mode)
      placeTimer(timer);
    timer->handler(timer);
  }

  if (timers)
    timers->timeout -= elapsed;
}

/*****************************************************************************
*****************************************************************************/
void SYS_TimerStart(SYS_Timer_t *timer)
{
  if (!SYS_TimerStarted(timer))
    placeTimer(timer);
}

/*****************************************************************************
*****************************************************************************/
void SYS_TimerStop(SYS_Timer_t *timer)
{
  SYS_Timer_t *prev = NULL;

  for (SYS_Timer_t *t = timers; t; t = t->next)
  {
    if (t == timer)
    {
      if (prev)
        prev->next = t->next;
      else
        timers = t->next;

      if (t->next)
        t->next->timeout += timer->timeout;

      break;
    }
    prev = t;
  }
}

/*****************************************************************************
*****************************************************************************/
bool SYS_TimerStarted(SYS_Timer_t *timer)
{
  for (SYS_Timer_t *t = timers; t; t = t->next)
    if (t == timer)
      return true;
  return false;
}

