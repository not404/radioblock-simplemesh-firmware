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
#include "nwk.h"
#include "nwkPrivate.h"
#include "phy.h"
#include "sysMem.h"
#include "sysQueue.h"
#include "sysTimer.h"
#include "sysTaskManager.h"

/*****************************************************************************
*****************************************************************************/
enum
{
  NWK_RX_STATE_RECEIVED,
  NWK_RX_STATE_FINISH,
  NWK_RX_STATE_ACCEPT,
  NWK_RX_STATE_INDICATE,
  NWK_RX_STATE_ACKNOWLEDGE,
  NWK_RX_STATE_ROUTE,
};

enum
{
  NWK_RX_SLEEP_REQUEST   = 1 << 0,
  NWK_RX_WAKEUP_REQUEST  = 1 << 1,
  NWK_RX_NO_REQUESTS     = 0,
};

typedef struct NwkDuplicateRejectionRecord_t
{
  uint16_t   src;
  uint8_t    seq;
  uint8_t    ttl;
} NwkDuplicateRejectionRecord_t;

/*****************************************************************************
*****************************************************************************/
// ETG To make AVR Studio happy
void nwkRxTaskHandler(void);

/*****************************************************************************
*****************************************************************************/
static void nwkRxSendAckConf(NwkFrame_t *frame);
static void nwkRxDuplicateRejectionTimerHandler(SYS_Timer_t *timer);

/*****************************************************************************
*****************************************************************************/
static SYS_Queue_t *nwkRxQueue;
static NwkDuplicateRejectionRecord_t nwkRxDuplicateRejectionTable[NWK_DUPLICATE_REJECTION_TABLE_SIZE];
static SYS_Timer_t nwkRxDuplicateRejectionTimer;
static uint8_t nwkRxRequests = NWK_RX_NO_REQUESTS;

/*****************************************************************************
*****************************************************************************/
void nwkRxInit(void)
{
  SYS_QueueInit(&nwkRxQueue);

  for (uint8_t i = 0; i < NWK_DUPLICATE_REJECTION_TABLE_SIZE; i++)
  {
    nwkRxDuplicateRejectionTable[i].ttl = 0;
    nwkRxDuplicateRejectionTable[i].src = 0xffff;
  }

  nwkRxDuplicateRejectionTimer.interval = NWK_DUPLICATE_REJECTION_TIMER_INTERVAL;
  nwkRxDuplicateRejectionTimer.mode = SYS_TIMER_PERIODIC_MODE;
  nwkRxDuplicateRejectionTimer.handler = nwkRxDuplicateRejectionTimerHandler;
  SYS_TimerStart(&nwkRxDuplicateRejectionTimer);
}

/*****************************************************************************
*****************************************************************************/
void PHY_DataInd(PHY_DataInd_t *ind)
{
  NwkFrame_t *frame;

  if (NULL == (frame = nwkFrameAlloc(ind->size - sizeof(NwkFrameHeader_t))))
    return;

  frame->rx.lqi = ind->lqi;
  frame->rx.rssi = ind->rssi;
  memcpy(frame->data, ind->data, ind->size);

  frame->state = NWK_RX_STATE_RECEIVED;
  SYS_QueueAppend(&nwkRxQueue, frame);
  SYS_TaskSet(NWK_RX_TASK);
}

/*****************************************************************************
*****************************************************************************/
static void nwkRxSendAck(NwkFrame_t *frame)
{
  NwkFrame_t *ackFrame;

  if (NULL == (ackFrame = nwkFrameAlloc(2)))
    return;

  nwkFrameCommandInit(ackFrame);

  ackFrame->tx.confirm = nwkRxSendAckConf;

  ackFrame->header->nwkDstAddr = frame->header->nwkSrcAddr;

  ackFrame->payload[0] = NWK_COMMAND_ACK;
  ackFrame->payload[1] = frame->header->nwkSeq;

  nwkTxFrame(ackFrame);
}

/*****************************************************************************
*****************************************************************************/
static void nwkRxSendAckConf(NwkFrame_t *frame)
{
  nwkFrameFree(frame);
}

/*****************************************************************************
*****************************************************************************/
static void nwkRxDuplicateRejectionTimerHandler(SYS_Timer_t *timer)
{
  for (uint8_t i = 0; i < NWK_DUPLICATE_REJECTION_TABLE_SIZE; i++)
  {
    if (nwkRxDuplicateRejectionTable[i].ttl)
      nwkRxDuplicateRejectionTable[i].ttl--;
  }
  (void)timer;
}

/*****************************************************************************
*****************************************************************************/
static bool nwkRxRejectDuplicate(NwkFrame_t *frame)
{
  NwkFrameHeader_t *header = frame->header;
  int8_t free = -1;

  if (header->nwkSrcAddr == nwkIb.addr)
    return true;

  for (uint8_t i = 0; i < NWK_DUPLICATE_REJECTION_TABLE_SIZE; i++)
  {
    if (nwkRxDuplicateRejectionTable[i].ttl)
    {
      if (header->nwkSrcAddr == nwkRxDuplicateRejectionTable[i].src)
      {
        int8_t diff = (int8_t)header->nwkSeq - nwkRxDuplicateRejectionTable[i].seq;

        if (diff > 0)
        {
          nwkRxDuplicateRejectionTable[i].seq = header->nwkSeq;
          nwkRxDuplicateRejectionTable[i].ttl = NWK_DUPLICATE_REJECTION_TTL;
          return false;
        }
        else
        {
          if (nwkIb.addr == header->macDstAddr)
            nwkRouteRemove(header->nwkDstAddr);
          return true;
        }
      }
    }
    else // ttl == 0
    {
      free = i;
    }
  }

  if (-1 == free)
    return true;

  nwkRxDuplicateRejectionTable[free].src = header->nwkSrcAddr;
  nwkRxDuplicateRejectionTable[free].seq = header->nwkSeq;
  nwkRxDuplicateRejectionTable[free].ttl = NWK_DUPLICATE_REJECTION_TTL;

  return false;
}

/*****************************************************************************
*****************************************************************************/
static uint8_t nwkRxProcessReceivedFrame(NwkFrame_t *frame)
{
  NwkFrameHeader_t *header = frame->header;

  nwkRouteFrameReceived(frame);

  if (nwkIb.addr == header->nwkSrcAddr)
    return NWK_RX_STATE_FINISH;

  if (nwkRxRejectDuplicate(frame))
    return NWK_RX_STATE_FINISH;

  if (0xffff == header->nwkDstAddr && header->nwkFcf.ackRequest)
    return NWK_RX_STATE_FINISH;

  if (header->nwkFcf.dstPort >= NWK_MAX_PORTS_AMOUNT || NULL == nwkIb.ind[header->nwkFcf.dstPort])
    return NWK_RX_STATE_FINISH;

  if (0xffff == header->macDstAddr && nwkIb.addr != header->nwkDstAddr)
    nwkTxBroadcastFrame(frame);

  if (nwkIb.addr == header->nwkDstAddr || 0xffff == header->nwkDstAddr)
    return NWK_RX_STATE_ACCEPT;

  if (nwkIb.addr == header->macDstAddr)
    return NWK_RX_STATE_ROUTE;

  return NWK_RX_STATE_FINISH;
}

/*****************************************************************************
*****************************************************************************/
static void nwkRxProcessAcceptedFrame(NwkFrame_t *frame)
{
  frame->state = NWK_RX_STATE_INDICATE;

  if (frame->header->nwkFcf.securityEnabled)
  {
    if (false == nwkSecurityDecryptFrame(frame))
      frame->state = NWK_RX_STATE_FINISH;
    frame->size -= sizeof(uint32_t);
  }

  SYS_TaskSet(NWK_RX_TASK);
}

/*****************************************************************************
*****************************************************************************/
static void nwkRxIndicateFrame(NwkFrame_t *frame)
{
  NWK_DataInd_t ind;

  ind.src = frame->header->nwkSrcAddr;
  ind.data = frame->payload;
  ind.port = frame->header->nwkFcf.dstPort;
  ind.size = frame->size - sizeof(NwkFrameHeader_t);
  ind.options.ack = frame->header->nwkFcf.ackRequest;
  ind.options.security = frame->header->nwkFcf.securityEnabled;
  ind.options.reserved = 0;
  ind.lqi = frame->rx.lqi;
  ind.rssi = frame->rx.rssi;

  if (nwkIb.ind[ind.port](&ind) && frame->header->nwkFcf.ackRequest)
    frame->state = NWK_RX_STATE_ACKNOWLEDGE;
  else
    frame->state = NWK_RX_STATE_FINISH;

  if (0xffff == frame->header->macDstAddr && nwkIb.addr == frame->header->nwkDstAddr)
    frame->state = NWK_RX_STATE_ACKNOWLEDGE;

  SYS_TaskSet(NWK_RX_TASK);
}

/*****************************************************************************
*****************************************************************************/
void nwkRxSleepReq(void)
{
  nwkRxRequests |= NWK_RX_SLEEP_REQUEST;
  SYS_TaskSet(NWK_RX_TASK);
}

/*****************************************************************************
*****************************************************************************/
void nwkRxWakeupReq(void)
{
  nwkRxRequests |= NWK_RX_WAKEUP_REQUEST;
  SYS_TaskSet(NWK_RX_TASK);
}

/*****************************************************************************
*****************************************************************************/
static void nwkRxHandleRequests(void)
{
  if (SYS_QueueHead(&nwkRxQueue))
    return;

  if (nwkRxRequests & NWK_RX_SLEEP_REQUEST)
  {
    SYS_TimerStop(&nwkRxDuplicateRejectionTimer);
    nwkRxSleepConf();
  };

  if (nwkRxRequests & NWK_RX_WAKEUP_REQUEST)
  {
    SYS_TimerStart(&nwkRxDuplicateRejectionTimer);
    nwkRxWakeupConf();
  };

  nwkRxRequests = NWK_RX_NO_REQUESTS;
}

/*****************************************************************************
*****************************************************************************/
void nwkRxTaskHandler(void)
{
  NwkFrame_t *frame, *next;

  frame = SYS_QueueHead(&nwkRxQueue);
  while (frame)
  {
    next = (NwkFrame_t *) SYS_QueueNext((void *) frame);

    switch (frame->state)
    {
      case NWK_RX_STATE_RECEIVED:
      {
        frame->state = nwkRxProcessReceivedFrame(frame);
        SYS_TaskSet(NWK_RX_TASK);
      } break;

      case NWK_RX_STATE_ACCEPT:
      {
        nwkRxProcessAcceptedFrame(frame);
      } break;

      case NWK_RX_STATE_INDICATE:
      {
        nwkRxIndicateFrame(frame);
      } break;

      case NWK_RX_STATE_ACKNOWLEDGE:
      {
        nwkRxSendAck(frame);
        frame->state = NWK_RX_STATE_FINISH;
        SYS_TaskSet(NWK_RX_TASK);
      } break;

      case NWK_RX_STATE_ROUTE:
      {
        SYS_QueueRemove(&nwkRxQueue, frame);
        nwkRouteFrame(frame);
      } break;

      case NWK_RX_STATE_FINISH:
      {
        SYS_QueueRemove(&nwkRxQueue, frame);
        nwkFrameFree(frame);
      } break;

      default:
        break;
    };
    frame = next;
  }

  nwkRxHandleRequests();
}

