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
#include "sysTaskManager.h"
#include "sysMem.h"
#include "sysQueue.h"

/*****************************************************************************
*****************************************************************************/
enum
{
  NWK_DATA_REQ_STATE_INITIAL,
  NWK_DATA_REQ_STATE_WAIT_CONF,
  NWK_DATA_REQ_STATE_CONFIRM,
};

/*****************************************************************************
*****************************************************************************/
static void nwkDataReqTxConf(NwkFrame_t *frame);

/*****************************************************************************
*****************************************************************************/
static SYS_Queue_t *nwkDataReqQueue;

/*****************************************************************************
*****************************************************************************/
void nwkDataReqInit(void)
{
  SYS_QueueInit(&nwkDataReqQueue);
}

/*****************************************************************************
*****************************************************************************/
void NWK_DataReq(NWK_DataReq_t *req)
{
  req->state = NWK_DATA_REQ_STATE_INITIAL;
  req->status = NWK_SUCCESS_STATUS;

  SYS_QueueAppend(&nwkDataReqQueue, req);
  SYS_TaskSet(NWK_DATA_REQ_TASK);
}

/*****************************************************************************
*****************************************************************************/
static void nwkDataReqSendFrame(NWK_DataReq_t *req)
{
  NwkFrame_t *frame;
  uint8_t size = req->size;

  if (req->options & NWK_OPT_SECURITY_ENABLED)
    size += sizeof(uint32_t);

  if (NULL == (frame = nwkFrameAlloc(size)))
  {
    req->state = NWK_DATA_REQ_STATE_CONFIRM;
    req->status = NWK_OUT_OF_MEMORY_STATUS;
    SYS_TaskSet(NWK_DATA_REQ_TASK);
    return;
  }

  frame->tx.dataReq = req;
  frame->tx.confirm = nwkDataReqTxConf;

  frame->header->nwkFcf.ackRequest = req->options & NWK_OPT_ACK_REQUEST ? 1 : 0;
  frame->header->nwkFcf.securityEnabled = req->options & NWK_OPT_SECURITY_ENABLED ? 1 : 0;
  frame->header->nwkFcf.dstPort = req->port;
  frame->header->nwkFcf.reserved = 0;
  frame->header->nwkSeq = ++nwkIb.nwkSeqNum;
  frame->header->nwkSrcAddr = nwkIb.addr;
  frame->header->nwkDstAddr = req->dst;

  memcpy(frame->payload, req->data, req->size);

  nwkTxFrame(frame);
}

/*****************************************************************************
*****************************************************************************/
static void nwkDataReqTxConf(NwkFrame_t *frame)
{
  frame->tx.dataReq->status = frame->tx.status;
  frame->tx.dataReq->state = NWK_DATA_REQ_STATE_CONFIRM;
  nwkFrameFree(frame);
  SYS_TaskSet(NWK_DATA_REQ_TASK);
}

/*****************************************************************************
*****************************************************************************/
void nwkDataReqTaskHandler(void)
{
  NWK_DataReq_t *req, *next;

  req = SYS_QueueHead(&nwkDataReqQueue);
  while (req)
  {
    next = (NWK_DataReq_t *) SYS_QueueNext((void *) req);

    switch (req->state)
    {
      case NWK_DATA_REQ_STATE_INITIAL:
      {
        req->state = NWK_DATA_REQ_STATE_WAIT_CONF;
        nwkDataReqSendFrame(req);
      } break;

      case NWK_DATA_REQ_STATE_WAIT_CONF:
        break;

      case NWK_DATA_REQ_STATE_CONFIRM:
      {
        SYS_QueueRemove(&nwkDataReqQueue, req);
        req->confirm(req);
      } break;

      default:
        break;
    };

    req = next;
  }
}

