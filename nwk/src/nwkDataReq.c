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

