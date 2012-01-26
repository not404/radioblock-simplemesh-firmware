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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "nwkPrivate.h"
#include "hal.h"
#include "phy.h"
#include "assert.h"
#include "sysMem.h"
#include "sysTaskManager.h"

#include "nwkRx.c"
#include "nwkTx.c"
#include "nwkFrame.c"
#include "nwkRoute.c"
#include "nwkDataReq.c"

/*****************************************************************************
*****************************************************************************/
NwkIb_t nwkIb;

/*****************************************************************************
*****************************************************************************/
void NWK_Init(void)
{
  HAL_Init();
  SYS_MemInit();

  nwkIb.nwkSeqNum = 0;
  nwkIb.macSeqNum = 0;

  nwkTxInit();
  nwkRxInit();
  nwkRouteInit();
  nwkDataReqInit();
}

/*****************************************************************************
*****************************************************************************/
void NWK_StartReq(NWK_StartReq_t *req)
{
  nwkIb.channel = req->channel;
  nwkIb.addr = req->addr;
  nwkIb.panId = req->panId;

  PHY_Init();
  PHY_SetChannel(nwkIb.channel);
  PHY_SetPanId(nwkIb.panId);
  PHY_SetShortAddr(nwkIb.addr);
  PHY_SetRxState(true);

  nwkRxStart();
}

