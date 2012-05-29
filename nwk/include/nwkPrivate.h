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
 
#ifndef _NWK_PRIVATE_H_
#define _NWK_PRIVATE_H_

#include <stdint.h>
#include "nwk.h"
#include "sysTypes.h"

/*****************************************************************************
*****************************************************************************/
enum
{
  NWK_COMMAND_ACK              = 0x00,
  NWK_COMMAND_ROUTE_ERROR      = 0x01,
};

/*****************************************************************************
*****************************************************************************/
typedef struct PACK NwkFrameHeader_t
{
  uint16_t    macFcf;
  uint8_t     macSeq;
  uint16_t    macDstPanId;
  uint16_t    macDstAddr;
  uint16_t    macSrcAddr;

  struct PACK
  {
    uint8_t   ackRequest       : 1;
    uint8_t   securityEnabled  : 1;
    uint8_t   reserved         : 2;
    uint8_t   dstPort          : 4;
  }           nwkFcf;
  uint8_t     nwkSeq;
  uint16_t    nwkSrcAddr;
  uint16_t    nwkDstAddr;
} NwkFrameHeader_t;

typedef struct NwkFrame_t
{
  void                   *next;
  uint8_t                state;
  uint8_t                *data;
  uint8_t                size;
  NwkFrameHeader_t       *header;
  uint8_t                *payload;

  union
  {
    struct
    {
      uint8_t            lqi;
      int8_t             rssi;
    } rx;

    struct
    {
      uint8_t            status;
      NWK_DataReq_t      *dataReq;
      uint8_t            waitTime;
      void               (*confirm)(struct NwkFrame_t *frame);
    } tx;
  };
} NwkFrame_t;

typedef struct NwkIb_t
{
  uint16_t     panId;
  uint16_t     addr;
  uint8_t      nwkSeqNum;
  uint8_t      macSeqNum;
  uint32_t     key[4];

  bool         (*ind[NWK_MAX_PORTS_AMOUNT])(NWK_DataInd_t *ind);
} NwkIb_t;

/*****************************************************************************
*****************************************************************************/
extern NwkIb_t nwkIb;

/*****************************************************************************
*****************************************************************************/
//#define PRIVATE static
#define PRIVATE

PRIVATE NwkFrame_t *nwkFrameAlloc(uint8_t size);
PRIVATE void nwkFrameFree(NwkFrame_t *frame);
PRIVATE void nwkFrameCommandInit(NwkFrame_t *frame);

PRIVATE void nwkRouteInit(void);
PRIVATE void nwkRouteRemove(uint16_t dst);
PRIVATE void nwkRouteFrameReceived(NwkFrame_t *frame);
PRIVATE void nwkRouteFrameSent(NwkFrame_t *frame);
PRIVATE uint16_t nwkRouteNextHop(uint16_t dst);
PRIVATE void nwkRouteFrame(NwkFrame_t *frame);
PRIVATE void nwkRouteErrorReceived(NWK_DataInd_t *ind);

PRIVATE void nwkRxInit(void);
PRIVATE void nwkRxSleepReq(void);
PRIVATE void nwkRxSleepConf(void);
PRIVATE void nwkRxWakeupReq(void);
PRIVATE void nwkRxWakeupConf(void);

PRIVATE void nwkTxInit(void);
PRIVATE void nwkTxFrame(NwkFrame_t *frame);
PRIVATE void nwkTxBroadcastFrame(NwkFrame_t *frame);
PRIVATE void nwkTxAckReceived(NWK_DataInd_t *ind);
PRIVATE void nwkTxSleepReq(void);
PRIVATE void nwkTxSleepConf(void);
PRIVATE void nwkTxWakeupReq(void);
PRIVATE void nwkTxWakeupConf(void);

PRIVATE void nwkDataReqInit(void);

PRIVATE void nwkSecurityEncryptFrame(NwkFrame_t *frame);
PRIVATE bool nwkSecurityDecryptFrame(NwkFrame_t *frame);

#endif // _NWK_PRIVATE_H_

