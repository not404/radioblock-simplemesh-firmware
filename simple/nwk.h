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
 
#ifndef _NWK_H_
#define _NWK_H_

#include <stdint.h>
#include "nwkConfig.h"
#include "sysQueue.h"

/*****************************************************************************
*****************************************************************************/
typedef enum
{
  NWK_SUCCESS_STATUS                      = 0x00,
  NWK_ERROR_STATUS                        = 0x01,
  NWK_OUT_OF_MEMORY_STATUS                = 0x02,

  NWK_NO_ACK_STATUS                       = 0x11,

  NWK_PHY_CHANNEL_ACCESS_FAILURE_STATUS   = 0x40,
  NWK_PHY_NO_ACK_STATUS                   = 0x41,
} NWK_Status_t;

enum
{
  NWK_OPT_ACK_REQUEST          = 1 << 0,
  NWK_OPT_SECURITY_ENABLED     = 1 << 1,
} NWK_Option_t;

typedef struct NWK_DataReq_t
{
  // service fields
  void         *next;
  uint8_t      state;

  // request parameters
  uint16_t     dst;
  uint8_t      options;

  uint8_t      *data;
  uint8_t      size;

  void         (*confirm)(struct NWK_DataReq_t *req);

  // confirmation parameters
  uint8_t      status;
} NWK_DataReq_t;

typedef struct NWK_DataInd_t
{
  uint16_t     src;
  uint8_t      *data;
  uint8_t      size;
  struct
  {
    uint8_t    ack      : 1;
    uint8_t    security : 1;
    uint8_t    reserved : 6;
  }            options;
  uint8_t      lqi;
  int8_t       rssi;
} NWK_DataInd_t;

typedef struct NWK_StartReq_t
{
  uint16_t     panId;
  uint16_t     addr;
  uint8_t      channel;
} NWK_StartReq_t;

/*****************************************************************************
*****************************************************************************/
void NWK_Init(void);
void NWK_StartReq(NWK_StartReq_t *req);
void NWK_DataReq(NWK_DataReq_t *req);
void NWK_DataInd(NWK_DataInd_t *ind);
uint16_t NWK_RouteNextHop(uint16_t dst);

#endif // _NWK_H_

