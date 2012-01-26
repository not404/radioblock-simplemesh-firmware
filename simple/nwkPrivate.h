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
#include "compiler.h"

/*****************************************************************************
*****************************************************************************/
#define NWK_ROUTE_UNKNOWN     0xffff

#define NWK_FRAME_MAX_PAYLOAD_SIZE           (127 - sizeof(NwkFrameHeader_t) - 2)
#define NWK_FRAME_MAX_SECURED_PAYLOAD_SIZE   (127 - sizeof(NwkFrameHeader_t) - sizeof(NwkFrameSecurityHeader_t) - 2)

/*****************************************************************************
*****************************************************************************/
enum
{
  NWK_FCF_FRAME_TYPE_DATA      = 0,
  NWK_FCF_FRAME_TYPE_COMMAND   = 1,
};

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
    uint8_t   frameType        : 1;
    uint8_t   ackRequest       : 1;
    uint8_t   securityEnabled  : 1;
    uint8_t   reserved         : 5;
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
      struct NwkFrame_t  *linkedFrame;
      uint8_t            waitTime;
      void               (*confirm)(struct NwkFrame_t *frame);
    } tx;
  };
} NwkFrame_t;

typedef struct NwkIb_t
{
  uint16_t     panId;
  uint16_t     addr;
  uint8_t      channel;
  uint8_t      nwkSeqNum;
  uint8_t      macSeqNum;
} NwkIb_t;

/*****************************************************************************
*****************************************************************************/
extern NwkIb_t nwkIb;

/*****************************************************************************
*****************************************************************************/
static NwkFrame_t *nwkFrameAlloc(uint8_t size);
static void nwkFrameFree(NwkFrame_t *frame);
static void nwkFrameCommandInit(NwkFrame_t *frame);

static void nwkRouteInit(void);
static void nwkRouteFrameReceived(NwkFrame_t *frame);
static void nwkRouteFrameSent(NwkFrame_t *frame);
static uint16_t nwkRouteNextHop(uint16_t dst);
static void nwkRouteFrame(NwkFrame_t *frame);
static void nwkRouteErrorReceived(NwkFrame_t *frame);

static void nwkRxInit(void);
static void nwkRxStart(void);

static void nwkTxInit(void);
static void nwkTxFrame(NwkFrame_t *frame);
static void nwkTxBroadcastFrame(NwkFrame_t *frame);
static void nwkTxAckReceived(NwkFrame_t *frame);

static void nwkDataReqInit(void);

#endif // _NWK_PRIVATE_H_

