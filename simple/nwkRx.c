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
#include "nwkConfig.h"
#include "nwkPrivate.h"
#include "phy.h"
#include "assert.h"
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
  NWK_RX_STATE_WAIT_DECRYPT_CONF,
  NWK_RX_STATE_WAIT_ACK_CONF,
  NWK_RX_STATE_INDICATE,
  NWK_RX_STATE_ROUTE,
};

typedef struct NwkDuplicateRejectionRecord_t
{
  uint16_t   src;
  uint8_t    seq;
  uint8_t    ttl;
} NwkDuplicateRejectionRecord_t;

/*****************************************************************************
*****************************************************************************/
static void nwkRxSendAckConf(NwkFrame_t *frame);
static void nwkRxDuplicateRejectionTimerHandler(SYS_Timer_t *timer);

/*****************************************************************************
*****************************************************************************/
static SYS_Queue_t *nwkRxQueue;
static NwkDuplicateRejectionRecord_t nwkRxDuplicateRejectionTable[NWK_DUPLICATE_REJECTION_TABLE_SIZE];
static SYS_Timer_t nwkRxDuplicateRejectionTimer;

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
}

/*****************************************************************************
*****************************************************************************/
void nwkRxStart(void)
{
  SYS_TimerStart(&nwkRxDuplicateRejectionTimer);
}

/*****************************************************************************
*****************************************************************************/
void PHY_DataInd(PHY_DataInd_t *ind)
{
  NwkFrame_t *frame;

  if (0x88 != ind->data[1] || (0x61 != ind->data[0] && 0x41 != ind->data[0]) ||
      ind->size < sizeof(NwkFrameHeader_t))
    return;

  if (NULL == (frame = nwkFrameAlloc(ind->size - sizeof(NwkFrameHeader_t))))
    return;

  frame->rx.lqi = ind->lqi;
  frame->rx.rssi = ind->rssi;
  memcpy(frame->data, ind->data, ind->size);

  frame->state = NWK_RX_STATE_RECEIVED;
  SYS_QueueAppend(&nwkRxQueue, frame);
  SYS_TaskSet(NWK_TASK_RX);
}

/*****************************************************************************
*****************************************************************************/
static void nwkRxDecryptFrame(NwkFrame_t *frame)
{
  (void)frame;
}

/*****************************************************************************
*****************************************************************************/
static bool nwkRxSendAck(NwkFrame_t *frame)
{
  NwkFrame_t *ackFrame;

  if (NULL == (ackFrame = nwkFrameAlloc(2)))
    return false;

  nwkFrameCommandInit(ackFrame);

  ackFrame->tx.confirm = nwkRxSendAckConf;

  ackFrame->header->nwkDstAddr = frame->header->nwkSrcAddr;

  ackFrame->payload[0] = NWK_COMMAND_ACK;
  ackFrame->payload[1] = frame->header->nwkSeq;

  ackFrame->tx.linkedFrame = frame;

  nwkTxFrame(ackFrame);
  return true;
}

/*****************************************************************************
*****************************************************************************/
static void nwkRxSendAckConf(NwkFrame_t *frame)
{
  assert(NWK_RX_STATE_WAIT_ACK_CONF == frame->tx.linkedFrame->state);

  if (NWK_SUCCESS_STATUS == frame->tx.status)
    frame->tx.linkedFrame->state = NWK_RX_STATE_INDICATE;
  else
    frame->tx.linkedFrame->state = NWK_RX_STATE_FINISH;

  nwkFrameFree(frame);
  SYS_TaskSet(NWK_TASK_RX);
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
static bool nwkRxRejectDuplicate(uint16_t src, uint8_t seq)
{
  int8_t free = -1;

  if (src == nwkIb.addr)
    return true;

  for (uint8_t i = 0; i < NWK_DUPLICATE_REJECTION_TABLE_SIZE; i++)
  {
    if (nwkRxDuplicateRejectionTable[i].ttl)
    {
      if (src == nwkRxDuplicateRejectionTable[i].src)
      {
        int8_t diff = (int8_t)seq - nwkRxDuplicateRejectionTable[i].seq;

        if (diff > 0)
        {
          nwkRxDuplicateRejectionTable[i].seq = seq;
          nwkRxDuplicateRejectionTable[i].ttl = NWK_DUPLICATE_REJECTION_TTL;
          return false;
        }
        else
        {
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

  nwkRxDuplicateRejectionTable[free].src = src;
  nwkRxDuplicateRejectionTable[free].seq = seq;
  nwkRxDuplicateRejectionTable[free].ttl = NWK_DUPLICATE_REJECTION_TTL;

  return false;
}

/*****************************************************************************
*****************************************************************************/
static uint8_t nwkRxProcessReceivedFrame(NwkFrame_t *frame)
{
  nwkRouteFrameReceived(frame);

  if (nwkIb.addr == frame->header->nwkSrcAddr)
    return NWK_RX_STATE_FINISH;

  if (nwkRxRejectDuplicate(frame->header->nwkSrcAddr, frame->header->nwkSeq))
    return NWK_RX_STATE_FINISH;

  if (0xffff == frame->header->nwkDstAddr && frame->header->nwkFcf.ackRequest)
    return NWK_RX_STATE_FINISH;

  if (0xffff == frame->header->macDstAddr && nwkIb.addr != frame->header->nwkDstAddr)
    nwkTxBroadcastFrame(frame);

  if (nwkIb.addr == frame->header->nwkDstAddr || 0xffff == frame->header->nwkDstAddr)
    return NWK_RX_STATE_ACCEPT;

  if (nwkIb.addr == frame->header->macDstAddr)
    return NWK_RX_STATE_ROUTE;

  return NWK_RX_STATE_FINISH;
}

/*****************************************************************************
*****************************************************************************/
static void nwkRxProcessAcceptedFrame(NwkFrame_t *frame)
{
  if (frame->header->nwkFcf.securityEnabled)
  {
    nwkRxDecryptFrame(frame);
    frame->state = NWK_RX_STATE_WAIT_DECRYPT_CONF;
  }
  else if (frame->header->nwkFcf.ackRequest)
  {
    if (nwkRxSendAck(frame))
      frame->state = NWK_RX_STATE_WAIT_ACK_CONF;
    else
      frame->state = NWK_RX_STATE_FINISH;
  }
  else
  {
    frame->state = NWK_RX_STATE_INDICATE;

    if (0xffff == frame->header->macDstAddr && nwkIb.addr == frame->header->nwkDstAddr)
      nwkRxSendAck(frame);
  }

  SYS_TaskSet(NWK_TASK_RX);
}

/*****************************************************************************
*****************************************************************************/
static void nwkRxCommand(NwkFrame_t *frame)
{
 // TODO: check frame->size

  switch (frame->payload[0])
  {
    case NWK_COMMAND_ACK:
    {
      nwkTxAckReceived(frame);
    } break;

    case NWK_COMMAND_ROUTE_ERROR:
    {
      nwkRouteErrorReceived(frame);
    } break;

    default:
      break;
  }
}

/*****************************************************************************
*****************************************************************************/
static void nwkRxIndicateFrame(NwkFrame_t *frame)
{
  if (NWK_FCF_FRAME_TYPE_DATA == frame->header->nwkFcf.frameType)
  {
    NWK_DataInd_t ind;

    ind.src = frame->header->nwkSrcAddr;
    ind.data = frame->payload;
    ind.size = frame->size - sizeof(NwkFrameHeader_t);
    ind.options.ack = frame->header->nwkFcf.ackRequest;
    ind.options.security = frame->header->nwkFcf.securityEnabled;
    ind.options.reserved = 0;
    ind.lqi = frame->rx.lqi;
    ind.rssi = frame->rx.rssi;

    NWK_DataInd(&ind);
  }
  else
  {
    nwkRxCommand(frame);
  }

  frame->state = NWK_RX_STATE_FINISH;
  SYS_TaskSet(NWK_TASK_RX);
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
        SYS_TaskSet(NWK_TASK_RX);
      } break;

      case NWK_RX_STATE_ACCEPT:
      {
        nwkRxProcessAcceptedFrame(frame);
      } break;

      case NWK_RX_STATE_WAIT_DECRYPT_CONF:
        break;

      case NWK_RX_STATE_WAIT_ACK_CONF:
        break;

      case NWK_RX_STATE_INDICATE:
      {
        nwkRxIndicateFrame(frame);
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
        assert(false);
    };
    frame = next;
  }
}

