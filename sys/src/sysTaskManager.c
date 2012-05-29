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
#include "nwkConfig.h"
#include "hal.h"

/*****************************************************************************
*****************************************************************************/
void sysTimerTaskHandler(void);
void phyTaskHandler(void);
void nwkTxTaskHandler(void);
void nwkRxTaskHandler(void);
void nwkDataReqTaskHandler(void);
void sysPortTaskHandler(void);
#ifdef HAL_UART_ENABLE
void halUartTxTaskHandler(void);
void halUartRxTaskHandler(void);
#endif

/*****************************************************************************
*****************************************************************************/
volatile uint16_t sysTasksPending = 0;
static uint16_t sysPortTasksPending = 0;

static void (*sysTaskHandlers[])(void) =
{
  phyTaskHandler,
#ifdef HAL_UART_ENABLE
  halUartTxTaskHandler,
  halUartRxTaskHandler,
#endif
  sysTimerTaskHandler,
  nwkTxTaskHandler,
  nwkRxTaskHandler,
  nwkDataReqTaskHandler,
  sysPortTaskHandler,
};

static void (*sysPortTaskHandlers[NWK_MAX_PORTS_AMOUNT])();

/*****************************************************************************
*****************************************************************************/
void sysPortTaskHandler(void)
{
  for (uint8_t i = 0; i < NWK_MAX_PORTS_AMOUNT; i++)
  {
    if (sysPortTasksPending & (1 << i))
    {
      ATOMIC_SECTION_ENTER
        sysPortTasksPending ^= (1 << i);
      ATOMIC_SECTION_LEAVE

      sysPortTaskHandlers[i]();
      break;
    }
  }
}

/*****************************************************************************
*****************************************************************************/
void SYS_TaskSet(SYS_TaskId_t id)
{
  SYS_TaskSetInline(id);
}

/*****************************************************************************
*****************************************************************************/
void SYS_PortSet(uint8_t port)
{
  ATOMIC_SECTION_ENTER
    sysPortTasksPending |= (1 << port);
  ATOMIC_SECTION_LEAVE
  SYS_TaskSet(SYS_PORT_TASK);
}

/*****************************************************************************
*****************************************************************************/
void SYS_PortTaskRegister(uint8_t port, void (*handler)(void))
{
  sysPortTaskHandlers[port] = handler;
}

/*****************************************************************************
*****************************************************************************/
void SYS_TaskRun(void)
{
  for (uint8_t i = 0; i < ARRAY_SIZE(sysTaskHandlers); i++)
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

