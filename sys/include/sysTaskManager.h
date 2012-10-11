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

#ifndef _SYS_TASK_MANAGER_H_
#define _SYS_TASK_MANAGER_H_

#include <stdint.h>
#include "config.h"
#include "hal.h"

/*****************************************************************************
*****************************************************************************/
typedef enum
{
  PHY_TASK,
#ifdef HAL_UART_ENABLE
  HAL_UART_TX_TASK,
  HAL_UART_RX_TASK,
#endif
  SYS_TIMER_TASK,
  NWK_TX_TASK,
  NWK_RX_TASK,
  NWK_DATA_REQ_TASK,
  SYS_PORT_TASK,
} SYS_TaskId_t;

/*****************************************************************************
*****************************************************************************/
extern volatile uint16_t sysTasksPending;

/*****************************************************************************
*****************************************************************************/
static inline void SYS_TaskSetInline(SYS_TaskId_t id)
{
  ATOMIC_SECTION_ENTER
    sysTasksPending |= (1 << id);
  ATOMIC_SECTION_LEAVE
}

/*****************************************************************************
*****************************************************************************/
void SYS_TaskSet(SYS_TaskId_t id);
void SYS_PortSet(uint8_t port);
void SYS_PortTaskRegister(uint8_t port, void (*handler)(void));
void SYS_TaskRun(void);

#endif // _SYS_TASK_MANAGER_H_

