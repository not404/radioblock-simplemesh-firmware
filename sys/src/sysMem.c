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
 
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "config.h"
#include "sysMem.h"
#include "sysTypes.h"

/*lint --e{826}  Surpress 'pointer conversion' messages, which occur when we
 *               convert from the mem_pool dec to that type */

/*****************************************************************************
*****************************************************************************/
#ifndef SYS_MEM_POOL_SIZE
#define SYS_MEM_POOL_SIZE    500
#endif

/*****************************************************************************
*****************************************************************************/
typedef struct PACK SysMemChunk_t
{
  uint8_t   free;
  uint8_t   filler;
  uint16_t  size;
} SysMemChunk_t;

/*****************************************************************************
*****************************************************************************/
/* Pointer to 32-bit variable resulted in memory overwrite. Adding to
 * (uint32_t *) type results in pointer incrementing by 4x what code was
 * expecting. Changed pointers to uint8_t * for now - this is potentially
 * less efficient as we always generate properly aligned pointers, but the resulting
 * code may not know that, and use the slower always-safe access.
 *TODO: check this & fix if needed for more efficient code. */
static uint8_t sysMemPool[SYS_MEM_POOL_SIZE];
static uint8_t *sysMemPoolEnd;

/*****************************************************************************
*****************************************************************************/
void SYS_MemInit(void)
{
  SysMemChunk_t *chunk = (SysMemChunk_t *)sysMemPool;

  chunk->free = 1;
  chunk->size = (sizeof(sysMemPool) & ~(sizeof(SysMemChunk_t) - 1));

  sysMemPoolEnd = sysMemPool + chunk->size;
}

/*****************************************************************************
*****************************************************************************/
uint8_t *SYS_MemAlloc(uint8_t size)
{
  SysMemChunk_t *chunk;
  uint8_t *ptr = sysMemPool;

  size = (size + sizeof(SysMemChunk_t) + 3) & ~3; // TODO: replace by proper alignment mechanism

  while (ptr != sysMemPoolEnd)
  {
    chunk = (SysMemChunk_t *)ptr;

    if (chunk->free && chunk->size >= size)
    {
      SysMemChunk_t *new = (SysMemChunk_t *)(ptr + size);

      if ((chunk->size - size) > 0)
      {
        new->free = 1;
        new->size = chunk->size - size;
      }

      chunk->free = 0;
      chunk->size = size;

      return (uint8_t *)ptr + sizeof(SysMemChunk_t);
    }

    ptr += chunk->size;
  }

  return NULL;
}

/*****************************************************************************
*****************************************************************************/
void SYS_MemFree(uint8_t *mem)
{
  SysMemChunk_t *chunk, *prev = NULL;
  uint8_t *ptr = sysMemPool;

  mem -= sizeof(SysMemChunk_t);

  while (ptr != sysMemPoolEnd)
  {
    chunk = (SysMemChunk_t *)ptr;

    if ((uint8_t *)mem == ptr)
    {
      SysMemChunk_t *next = (SysMemChunk_t *)(ptr + chunk->size);

      chunk->free = 1;

      if (next->free)
        chunk->size += next->size;

      if (prev && prev->free)
        prev->size += chunk->size;

      return;
    }

    prev = chunk;
    ptr += chunk->size;
  }
}

