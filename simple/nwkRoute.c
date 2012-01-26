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
#include "nwkConfig.h"
#include "nwkPrivate.h"
#include "sysMem.h"
#include "sysTimer.h"
#include "assert.h"
#include "compiler.h"

/*****************************************************************************
*****************************************************************************/
typedef struct PACK NwkRouteErrorCommand_t
{
  uint8_t    id;
  uint16_t   srcAddr;
  uint16_t   dstAddr;
} NwkRouteErrorCommand_t;

typedef struct NwkRouteTableRecord_t
{
  uint16_t   dst;
  uint16_t   nextHop;
  uint8_t    score;
  uint8_t    lqi;
} NwkRouteTableRecord_t;

/*****************************************************************************
*****************************************************************************/
static void nwkRouteTxFrameConf(NwkFrame_t *frame);
static void nwkRouteSendRouteError(uint16_t src, uint16_t dst);
static void nwkRouteErrorConf(NwkFrame_t *frame);

/*****************************************************************************
*****************************************************************************/
static NwkRouteTableRecord_t nwkRouteTable[NWK_ROUTE_TABLE_SIZE];

/*****************************************************************************
*****************************************************************************/
void nwkRouteInit(void)
{
  for (uint8_t i = 0; i < NWK_ROUTE_TABLE_SIZE; i++)
    nwkRouteTable[i].dst = NWK_ROUTE_UNKNOWN;
}

/*****************************************************************************
*****************************************************************************/
static NwkRouteTableRecord_t *nwkRouteFindRecord(uint16_t dst)
{
/*
  for (NwkRouteTableRecord_t *rec = nwkRouteRtBegin; rec < nwkRouteRtEnd; rec++)
    if (rec->dst == dst)
      return rec;

  if (0xffff == dst)
    return nwkRouteRtEnd - 1;
*/
  for (uint8_t i = 0; i < NWK_ROUTE_TABLE_SIZE; i++)
    if (nwkRouteTable[i].dst == dst)
      return &nwkRouteTable[i];

  if (NWK_ROUTE_UNKNOWN == dst)
    return &nwkRouteTable[NWK_ROUTE_TABLE_SIZE - 1];

  return NULL;
}

/*****************************************************************************
*****************************************************************************/
void nwkRouteFrameReceived(NwkFrame_t *frame)
{
  NwkRouteTableRecord_t *rec;

  rec = nwkRouteFindRecord(frame->header->nwkSrcAddr);
  if (rec)
  {
    if (rec->nextHop != frame->header->macSrcAddr && frame->rx.lqi >= rec->lqi)
    {
      rec->nextHop = frame->header->macSrcAddr;
      rec->score = NWK_ROUTE_DEFAULT_SCORE;
    }
  }
  else
  {
    rec = nwkRouteFindRecord(NWK_ROUTE_UNKNOWN);

    rec->dst = frame->header->nwkSrcAddr;
    rec->nextHop = frame->header->macSrcAddr;
    rec->score = NWK_ROUTE_DEFAULT_SCORE;
  }

  rec->lqi = frame->rx.lqi;

  (void)frame;
}

/*****************************************************************************
*****************************************************************************/
void nwkRouteFrameSent(NwkFrame_t *frame)
{
  NwkRouteTableRecord_t *rec;

  rec = nwkRouteFindRecord(frame->header->nwkDstAddr);
  if (NULL == rec)
    return;

  if (NWK_SUCCESS_STATUS == frame->tx.status)
  {
    rec->score = NWK_ROUTE_DEFAULT_SCORE;
  }
  else
  {
    rec->score--;
    if (0 == rec->score)
    {
      rec->dst = NWK_ROUTE_UNKNOWN;
      return;
    }
  }

  if ((rec - &nwkRouteTable[0]) > 0)
  {
    NwkRouteTableRecord_t *prev = rec - 1;
    NwkRouteTableRecord_t tmp;

    tmp = *prev;
    *prev = *rec;
    *rec = tmp;
  }
}

/*****************************************************************************
*****************************************************************************/
uint16_t nwkRouteNextHop(uint16_t dst)
{
  for (uint8_t i = 0; i < NWK_ROUTE_TABLE_SIZE; i++)
    if (nwkRouteTable[i].dst == dst)
      return nwkRouteTable[i].nextHop;

  return NWK_ROUTE_UNKNOWN;
}

/*****************************************************************************
*****************************************************************************/
void nwkRouteFrame(NwkFrame_t *frame)
{
  if (NWK_ROUTE_UNKNOWN != nwkRouteNextHop(frame->header->nwkDstAddr))
  {
    frame->tx.confirm = nwkRouteTxFrameConf;
    nwkTxFrame(frame);
  }
  else
  {
    nwkRouteSendRouteError(frame->header->nwkSrcAddr, frame->header->nwkDstAddr);
    nwkFrameFree(frame);
  }
}

/*****************************************************************************
*****************************************************************************/
static void nwkRouteTxFrameConf(NwkFrame_t *frame)
{
  nwkFrameFree(frame);
}

/*****************************************************************************
*****************************************************************************/
static void nwkRouteSendRouteError(uint16_t src, uint16_t dst)
{
  NwkFrame_t *frame;
  NwkRouteErrorCommand_t *command;

  if (NULL == (frame = nwkFrameAlloc(sizeof(NwkRouteErrorCommand_t))))
    return;

  nwkFrameCommandInit(frame);

  frame->tx.confirm = nwkRouteErrorConf;

  frame->header->nwkDstAddr = src;

  command = (NwkRouteErrorCommand_t *)frame->payload;

  command->id = NWK_COMMAND_ROUTE_ERROR;
  command->srcAddr = src;
  command->dstAddr = dst;

  nwkTxFrame(frame);
}

/*****************************************************************************
*****************************************************************************/
static void nwkRouteErrorConf(NwkFrame_t *frame)
{
  nwkFrameFree(frame);
}

/*****************************************************************************
*****************************************************************************/
void nwkRouteErrorReceived(NwkFrame_t *frame)
{
  NwkRouteErrorCommand_t *command = (NwkRouteErrorCommand_t *)frame->payload;
  NwkRouteTableRecord_t *rec;

  rec = nwkRouteFindRecord(command->dstAddr);
  if (rec)
    rec->dst = NWK_ROUTE_UNKNOWN;
}

/*****************************************************************************
*****************************************************************************/
uint16_t NWK_RouteNextHop(uint16_t dst)
{
  return nwkRouteNextHop(dst);
}

