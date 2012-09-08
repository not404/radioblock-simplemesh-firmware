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

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "nwkPrivate.h"
#include "hal.h"
#include "phy.h"
#include "sysMem.h"
#include "sysTaskManager.h"

/*****************************************************************************
*****************************************************************************/
static bool nwkCommandDataInd(NWK_DataInd_t *ind);

/*****************************************************************************
*****************************************************************************/
NwkIb_t nwkIb;

/*****************************************************************************
*****************************************************************************/
void NWK_Init(void)
{
  HAL_Init();
  SYS_MemInit();
  PHY_Init();

  nwkIb.addr = 0;
  nwkIb.panId = 0;
  nwkIb.nwkSeqNum = 0;
  nwkIb.macSeqNum = 0;

  for (uint8_t i = 0; i < NWK_MAX_PORTS_AMOUNT; i++)
    nwkIb.ind[i] = NULL;

  nwkTxInit();
  nwkRxInit();
  nwkRouteInit();
  nwkDataReqInit();

  NWK_PortOpen(0, NULL, nwkCommandDataInd);
}

/*****************************************************************************
*****************************************************************************/
void NWK_SetAddr(uint16_t addr)
{
  nwkIb.addr = addr;
  PHY_SetAddr(addr);
}

/*****************************************************************************
*****************************************************************************/
void NWK_SetPanId(uint16_t panId)
{
  nwkIb.panId = panId;
  PHY_SetPanId(panId);
}

/*****************************************************************************
*****************************************************************************/
void NWK_SetSecurityKey(uint8_t *key)
{
  memcpy((uint8_t *)nwkIb.key, key, 16);
}

/*****************************************************************************
*****************************************************************************/
void NWK_PortOpen(uint8_t port, void (*handler)(void), bool (*ind)(NWK_DataInd_t *))
{
  nwkIb.ind[port] = ind;
  if (NULL != handler)
  {
    SYS_PortTaskRegister(port, handler);
    SYS_PortSet(port);
  }
}

/*****************************************************************************
*****************************************************************************/
void NWK_SleepReq(void)
{
  nwkRxSleepReq();
}

/*****************************************************************************
*****************************************************************************/
void nwkRxSleepConf(void)
{
  nwkTxSleepReq();
}

/*****************************************************************************
*****************************************************************************/
void nwkTxSleepConf(void)
{
  PHY_SleepReq();
}

/*****************************************************************************
*****************************************************************************/
void PHY_SleepConf(void)
{
  NWK_SleepConf();
}

/*****************************************************************************
*****************************************************************************/
WEAK void NWK_SleepConf(void)
{
}

/*****************************************************************************
*****************************************************************************/
void NWK_WakeupReq(void)
{
  nwkTxWakeupReq();
}

/*****************************************************************************
*****************************************************************************/
void nwkTxWakeupConf(void)
{
  nwkRxWakeupReq();
}

/*****************************************************************************
*****************************************************************************/
void nwkRxWakeupConf(void)
{
  PHY_WakeupReq();
}

/*****************************************************************************
*****************************************************************************/
void PHY_WakeupConf(void)
{
  NWK_WakeupConf();
}

/*****************************************************************************
*****************************************************************************/
WEAK void NWK_WakeupConf(void)
{
}

/*****************************************************************************
*****************************************************************************/
static bool nwkCommandDataInd(NWK_DataInd_t *ind)
{
  uint8_t command;

  if (0 == ind->size)
    return false;

  command = ind->data[0];

  if (NWK_COMMAND_ACK == command)
    nwkTxAckReceived(ind);
  else if (NWK_COMMAND_ROUTE_ERROR == command)
    nwkRouteErrorReceived(ind);
  else
    return false;

  return true;
}

