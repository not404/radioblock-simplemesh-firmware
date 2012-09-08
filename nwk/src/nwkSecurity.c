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
#include <string.h>
#include <stdbool.h>
#include <nwkPrivate.h>

/*****************************************************************************
*****************************************************************************/
static void xtea(uint32_t text[2], uint32_t const key[4])
{
  uint32_t t0 = text[0];
  uint32_t t1 = text[1];
  uint32_t sum = 0;
  uint32_t delta = 0x9e3779b9;

  for (uint8_t i = 0; i < 32; i++)
  {
    t0 += (((t1 << 4) ^ (t1 >> 5)) + t1) ^ (sum + key[sum & 3]);
    sum += delta;
    t1 += (((t0 << 4) ^ (t0 >> 5)) + t0) ^ (sum + key[(sum >> 11) & 3]);
  }
  text[0] = t0;
  text[1] = t1;
}

/*****************************************************************************
*****************************************************************************/
static bool nwkSecurityProcessFrame(NwkFrame_t *frame, bool decrypt)
{
  uint32_t vector[2];
  uint8_t *vectorb = (uint8_t *)vector;
  uint8_t size = frame->size - sizeof(NwkFrameHeader_t) - sizeof(uint32_t);
  uint8_t block;
  uint32_t mic;

  vector[0] = ((uint32_t)frame->header->macDstPanId << 16) |
              ((uint32_t)(*(uint8_t *)&frame->header->nwkFcf) << 8) |
              ((uint32_t)frame->header->nwkSeq);
  vector[1] = ((uint32_t)frame->header->nwkDstAddr << 16) |
              ((uint32_t)frame->header->nwkSrcAddr);

  for (uint8_t j = 0; size > 0; j++)
  {
    xtea(vector, nwkIb.key);

    block = (size < 8) ? size : 8;

    for (uint8_t i = 0; i < block; i++)
    {
      frame->payload[j*8 + i] ^= vectorb[i];

      if (decrypt)
        vectorb[i] ^= frame->payload[j*8 + i];
      else
        vectorb[i] = frame->payload[j*8 + i];
    }

    size -= block;
  }

  if (decrypt)
  {
    memcpy((uint8_t *)&mic, &frame->data[frame->size]-sizeof(mic), sizeof(mic));
    return mic == (vector[0] ^ vector[1]);
  }
  else
  {
    mic = vector[0] ^ vector[1];
    memcpy(&frame->data[frame->size]-sizeof(mic), (uint8_t *)&mic, sizeof(mic));
  }

  return true;
}

/*****************************************************************************
*****************************************************************************/
void nwkSecurityEncryptFrame(NwkFrame_t *frame)
{
  nwkSecurityProcessFrame(frame, false);
}

/*****************************************************************************
*****************************************************************************/
bool nwkSecurityDecryptFrame(NwkFrame_t *frame)
{
  return nwkSecurityProcessFrame(frame, true);
}

