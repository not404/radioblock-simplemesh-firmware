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
 
#include <stdint.h>
#include "sysTaskManager.h"
#include "hal.h"

/*****************************************************************************
*****************************************************************************/
void sysTimerTaskHandler(void);
void phyTaskHandler(void);
void nwkTxTaskHandler(void);
void nwkRxTaskHandler(void);
void nwkDataReqTaskHandler(void);
void APP_TaskHandler(void);

/*****************************************************************************
*****************************************************************************/
volatile uint16_t sysTasksPending = 0;

static void (*sysTaskHandlers[])() =
{
  phyTaskHandler,
  sysTimerTaskHandler,
  nwkTxTaskHandler,
  nwkRxTaskHandler,
  nwkDataReqTaskHandler,
  APP_TaskHandler,

#ifdef APP_TASK_HANDLER_1
  APP_TASK_1,
#endif
};

/*****************************************************************************
*****************************************************************************/
void SYS_TaskSet(SYS_TaskId_t id)
{
  SYS_TaskSetInline(id);
}

/*****************************************************************************
*****************************************************************************/
void SYS_TaskRun(void)
{
  for (uint8_t i = 0; i < (sizeof(sysTaskHandlers) / sizeof(sysTaskHandlers[0])); i++)
  {
    if (sysTasksPending & (1 << i))
    {
      ATOMIC_SECTION_ENTER
        sysTasksPending ^= (1 << i);
      ATOMIC_SECTION_LEAVE

      sysTaskHandlers[i]();
      break;
    }
  }
}

