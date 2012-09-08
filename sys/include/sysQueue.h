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

#ifndef _SYS_QUEUE_H_
#define _SYS_QUEUE_H_

#include <stdlib.h>

/*****************************************************************************
*****************************************************************************/
typedef struct SYS_Queue_t
{
  struct SYS_Queue_t *next;
} SYS_Queue_t;

/*****************************************************************************
*****************************************************************************/
static inline void SYS_QueueInit(SYS_Queue_t **queue)
{
  *queue = NULL;
}

/*****************************************************************************
*****************************************************************************/
static inline void *SYS_QueueHead(SYS_Queue_t **queue)
{
  return *queue;
}

/*****************************************************************************
*****************************************************************************/
static inline void *SYS_QueueNext(void *item)
{
  return item ? ((SYS_Queue_t*) item)->next : NULL;
}

/*****************************************************************************
*****************************************************************************/
void  SYS_QueueAppend(SYS_Queue_t **queue, void *item);
void *SYS_QueueRemove(SYS_Queue_t **queue, void *item);

#endif // _SYS_QUEUE_H_

