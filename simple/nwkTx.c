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
#define NWK_TX_WAIT_TIMER_INTERVAL   32 // ms

/*****************************************************************************
*****************************************************************************/
enum
{
  NWK_TX_STATE_SEND,
  NWK_TX_STATE_WAIT_CONF,
  NWK_TX_STATE_SENT,
  NWK_TX_STATE_WAIT_ACK,
  NWK_TX_STATE_CONFIRM,
};

/*****************************************************************************
*****************************************************************************/
static void nwkTxBroadcastConf(NwkFrame_t *frame);
static void nwkTxStartWaitTimer(NwkFrame_t *frame);
static void nwkTxWaitTimerHandler(SYS_Timer_t *timer);

/*****************************************************************************
*****************************************************************************/
static SYS_Queue_t *nwkTxQueue;
static NwkFrame_t *nwkPhyActiveFrame;
static SYS_Timer_t nwkTxWaitTimer;

/*****************************************************************************
*****************************************************************************/
void nwkTxInit(void)
{
  SYS_QueueInit(&nwkTxQueue);
  nwkPhyActiveFrame = NULL;

  nwkTxWaitTimer.interval = NWK_TX_WAIT_TIMER_INTERVAL;
  nwkTxWaitTimer.mode = SYS_TIMER_PERIODIC_MODE;
  nwkTxWaitTimer.handler = nwkTxWaitTimerHandler;
}

/*****************************************************************************
*****************************************************************************/
void nwkTxFrame(NwkFrame_t *frame)
{
  frame->state = NWK_TX_STATE_SEND;
  frame->tx.status = NWK_SUCCESS_STATUS;

  frame->header->macDstAddr = nwkRouteNextHop(frame->header->nwkDstAddr);
  frame->header->macDstPanId = nwkIb.panId;
  frame->header->macSrcAddr = nwkIb.addr;
  frame->header->macSeq = ++nwkIb.macSeqNum;

  if (0xffff == frame->header->macDstAddr)
    frame->header->macFcf = 0x8841;
  else
    frame->header->macFcf = 0x8861;

  SYS_QueueAppend(&nwkTxQueue, frame);
  SYS_TaskSet(NWK_TASK_TX);
}

/*****************************************************************************
*****************************************************************************/
void nwkTxBroadcastFrame(NwkFrame_t *frame)
{
  NwkFrame_t *newFrame;

  if (NULL == (newFrame = nwkFrameAlloc(frame->size - sizeof(NwkFrameHeader_t))))
    return;

  newFrame->tx.confirm = nwkTxBroadcastConf;
  memcpy(newFrame->data, frame->data, frame->size);

  newFrame->state = NWK_TX_STATE_SEND;
  newFrame->tx.status = NWK_SUCCESS_STATUS;

  newFrame->header->macFcf = 0x8841;
  newFrame->header->macDstAddr = 0xffff;
  newFrame->header->macDstPanId = nwkIb.panId;
  newFrame->header->macSrcAddr = nwkIb.addr;
  newFrame->header->macSeq = ++nwkIb.macSeqNum;

  SYS_QueueAppend(&nwkTxQueue, newFrame);
  SYS_TaskSet(NWK_TASK_TX);
}

/*****************************************************************************
*****************************************************************************/
static void nwkTxBroadcastConf(NwkFrame_t *frame)
{
  nwkFrameFree(frame);
}

/*****************************************************************************
*****************************************************************************/
void nwkTxAckReceived(NwkFrame_t *frame)
{
  for (NwkFrame_t *f = SYS_QueueHead(&nwkTxQueue); f; f = f->next)
  {
    if (NWK_TX_STATE_WAIT_ACK == f->state && f->header->nwkSeq == frame->payload[1])
    {
      f->state = NWK_TX_STATE_CONFIRM;
      SYS_TaskSet(NWK_TASK_TX);
      return;
    }
  }
}

/*****************************************************************************
*****************************************************************************/
static void nwkTxStartWaitTimer(NwkFrame_t *frame)
{
  frame->tx.waitTime = NWK_ACK_WAIT_TIME / NWK_TX_WAIT_TIMER_INTERVAL + 1;
  SYS_TimerStart(&nwkTxWaitTimer);
}

/*****************************************************************************
*****************************************************************************/
static void nwkTxWaitTimerHandler(SYS_Timer_t *timer)
{
  bool stop = true;

  for (NwkFrame_t *f = SYS_QueueHead(&nwkTxQueue); f; f = f->next)
  {
    if (NWK_TX_STATE_WAIT_ACK == f->state && 0 == --f->tx.waitTime)
    {
      f->state = NWK_TX_STATE_CONFIRM;
      f->tx.status = NWK_NO_ACK_STATUS;
      SYS_TaskSet(NWK_TASK_TX);
    }

    if (NWK_TX_STATE_WAIT_ACK == f->state)
      stop = false; 
  }

  if (stop)
    SYS_TimerStop(timer);
}

/*****************************************************************************
*****************************************************************************/
void nwkTxTaskHandler(void)
{
  for (NwkFrame_t *frame = SYS_QueueHead(&nwkTxQueue); frame; frame = frame->next)
  {
    switch (frame->state)
    {
      case NWK_TX_STATE_SEND:
      {
        if (nwkPhyActiveFrame)
          break;

        nwkPhyActiveFrame = frame;
        frame->state = NWK_TX_STATE_WAIT_CONF;
        PHY_DataReq(frame->data, frame->size);
      } break;

      case NWK_TX_STATE_WAIT_CONF:
        break;

      case NWK_TX_STATE_SENT:
      {
        if (NWK_SUCCESS_STATUS == frame->tx.status)
        {
          if (frame->header->nwkSrcAddr == nwkIb.addr && frame->header->nwkFcf.ackRequest)
          {
            frame->state = NWK_TX_STATE_WAIT_ACK;
            nwkTxStartWaitTimer(frame);
          }
          else
          {
            frame->state = NWK_TX_STATE_CONFIRM;
            SYS_TaskSet(NWK_TASK_TX);
          }
        }
        else
        {
          frame->state = NWK_TX_STATE_CONFIRM;
          SYS_TaskSet(NWK_TASK_TX);
	}
      } break;

      case NWK_TX_STATE_WAIT_ACK:
        break;

      case NWK_TX_STATE_CONFIRM:
      {
        nwkRouteFrameSent(frame);
        SYS_QueueRemove(&nwkTxQueue, frame);
	nwkPhyActiveFrame = NULL;
        frame->tx.confirm(frame);
      } break;

      default:
        assert(false);
    };
  }
}

/*****************************************************************************
*****************************************************************************/
static uint8_t convertPhyStatus(uint8_t status)
{
  if (TRAC_STATUS_SUCCESS == status ||
      TRAC_STATUS_SUCCESS_DATA_PENDING == status ||
      TRAC_STATUS_SUCCESS_WAIT_FOR_ACK == status)
    return NWK_SUCCESS_STATUS;

  else if (TRAC_STATUS_CHANNEL_ACCESS_FAILURE == status)
    return NWK_PHY_CHANNEL_ACCESS_FAILURE_STATUS;

  else if (TRAC_STATUS_NO_ACK == status)
    return NWK_PHY_NO_ACK_STATUS;

  else
    return NWK_ERROR_STATUS;
}

/*****************************************************************************
*****************************************************************************/
void PHY_DataConf(uint8_t status)
{
  nwkPhyActiveFrame->tx.status = convertPhyStatus(status);
  nwkPhyActiveFrame->state = NWK_TX_STATE_SENT;
  SYS_TaskSet(NWK_TASK_TX);
}

