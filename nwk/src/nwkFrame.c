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
#include "sysMem.h"

/*****************************************************************************
*****************************************************************************/
NwkFrame_t *nwkFrameAlloc(uint8_t size)
{
  NwkFrame_t *frame;
  uint8_t    *buf;

  buf = SYS_MemAlloc(sizeof(NwkFrame_t) + sizeof(NwkFrameHeader_t) + size);
  if (NULL == buf)
    return NULL;

  frame = (NwkFrame_t *)buf;

  frame->data = &buf[sizeof(NwkFrame_t)];
  frame->size = sizeof(NwkFrameHeader_t) + size;
  frame->header = (NwkFrameHeader_t *)frame->data;
  frame->payload = &frame->data[sizeof(NwkFrameHeader_t)];

  return frame;
}

/*****************************************************************************
*****************************************************************************/
void nwkFrameFree(NwkFrame_t *frame)
{
  SYS_MemFree((uint8_t*)frame);
}

/*****************************************************************************
*****************************************************************************/
void nwkFrameCommandInit(NwkFrame_t *frame)
{
  frame->tx.status = NWK_SUCCESS_STATUS;
  frame->tx.dataReq = NULL;
  frame->tx.waitTime = 0;
  frame->tx.confirm = NULL;

  frame->header->nwkFcf.ackRequest = 0;
  frame->header->nwkFcf.securityEnabled = 0;
  frame->header->nwkFcf.dstPort = 0;
  frame->header->nwkFcf.reserved = 0;
  frame->header->nwkSeq = ++nwkIb.nwkSeqNum;
  frame->header->nwkSrcAddr = nwkIb.addr;
  frame->header->nwkDstAddr = 0;
}

