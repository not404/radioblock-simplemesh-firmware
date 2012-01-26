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
 
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "nwk.h"
#include "sysTimer.h"
#include "sysMem.h"
#include "sysTaskManager.h"

/*****************************************************************************
*****************************************************************************/
#ifndef ADDR
#define ADDR   1
#endif

/*****************************************************************************
*****************************************************************************/
typedef struct
{
  uint8_t      id;
  uint16_t     nextHop;
} AppMessage_t;

/*****************************************************************************
*****************************************************************************/
SYS_Timer_t timer0;
SYS_Timer_t timer1;
SYS_Timer_t timer2;

NWK_DataReq_t nwkDataReq;
AppMessage_t msg;

/*****************************************************************************
*****************************************************************************/
void log_open(void)
{
#if ADDR == 0
  UBRR1H = 0;
  UBRR1L = 25;
  UCSR1A = (1 << U2X1);
  UCSR1B = (1 << TXEN1);
  UCSR1C = (3 << UCSZ10);
#endif
}

/*****************************************************************************
*****************************************************************************/
void write(uint8_t message)
{
#if ADDR == 0
  UDR1 = message;
  while (!(UCSR1A & (1 << UDRE1)));
#endif
  (void)message;
}

/*****************************************************************************
*****************************************************************************/
void print(const char *format, ...)
{
#if ADDR == 0
  uint8_t str[70];
  char *ptr = (char *)str;
  uint8_t len;
  va_list ap;

  va_start(ap, format);
  len = vsnprintf(ptr, sizeof(str), format, ap);
  while (len--)
    write(*ptr++);
#endif
  (void)format;
}

static int32_t d0 = 0, d1 = 0;

/*****************************************************************************
*****************************************************************************/
void dataConf(NWK_DataReq_t *req)
{
//  PORTE &= ~(1 << 3);

  if (NWK_SUCCESS_STATUS != req->status)
  {
    print("dataConf: 0x%02x\r\n", req->status);
//    PORTE ^= (1 << 4);
    d0++;
    //print("%ld / %ld\r\n", d0, d1);
  }
  else
  {
    d1++;
  }

  SYS_TimerStart(&timer0);
}

uint16_t nwkRouteNextHop(uint16_t dst);

/*****************************************************************************
*****************************************************************************/
void timer0cb(SYS_Timer_t *timer)
{
  msg.id++;
  msg.nextHop = NWK_RouteNextHop(0);

  nwkDataReq.dst = 0;//1-ADDR;
  nwkDataReq.options = NWK_OPT_ACK_REQUEST;// | NWK_OPT_SECURITY_ENABLED;
  nwkDataReq.data = (uint8_t *)&msg;
  nwkDataReq.size = sizeof(msg);
  nwkDataReq.confirm = dataConf;

#if ADDR != 0
//  PORTE |= (1 << 3);
  NWK_DataReq(&nwkDataReq);
#endif

  (void)timer;
}

/*****************************************************************************
*****************************************************************************/
void NWK_DataInd(NWK_DataInd_t *ind)
{
  AppMessage_t *m = (AppMessage_t *)ind->data;
 
  print("0x%04x, lqi=%u, rssi=%d: id=0x%02x, hop=0x%04x\r\n", ind->src, ind->lqi, ind->rssi, m->id, m->nextHop);

/*
  write('+');
  for (uint8_t i = 0; i < ind->size; i++)
    write(ind->data[i]);
  write('\r');
  write('\n');
*/
}

/*****************************************************************************
*****************************************************************************/
void timer1cb(SYS_Timer_t *timer)
{
//  PORTE ^= (1 << 3);
 // print("---\r\n");

  (void)timer;
}

uint8_t cnt = 0;

/*****************************************************************************
*****************************************************************************/
void timer2cb(SYS_Timer_t *timer)
{
//  print("timer 2\r\n");

//  PORTE ^= (1 << 2);
/*
  static bool on = true;

  on = !on;

  if (on)
    LPC_GPIO0->MASKED_ACCESS[(1 << 7)] = (1 << 7);
  else
    LPC_GPIO0->MASKED_ACCESS[(1 << 7)] = (0 << 7);
*/

#if ADDR != 0
//  PORTD ^= 0xff;
#endif
  (void)timer;
}

/*****************************************************************************
*****************************************************************************/
void APP_TaskHandler(void)
{
  // ...
}

/*****************************************************************************
*****************************************************************************/
int main(void)
{
  NWK_StartReq_t req;
//  LPC_GPIO0->DIR |= (1 << 7);

/*
  for (int j = 0; j < 5; j++)
  {
    for (int i = 0; i < 1000000; i++) asm("nop");
    LPC_GPIO0->MASKED_ACCESS[(1 << 7)] = (0 << 7);
    for (int i = 0; i < 1000000; i++) asm("nop");
    LPC_GPIO0->MASKED_ACCESS[(1 << 7)] = (1 << 7);
  }
*/

//  DDRE = 0xff;

#if ADDR != 0
//  DDRD = 0xff;
//  PORTD = 0xff;
#endif

  log_open();
  print("-----------------------------------\r\n");

  NWK_Init();

  req.channel = 0x0f;
  req.panId = 0x1234;
  req.addr = ADDR;
  NWK_StartReq(&req);

  timer0.interval = 2000;
  timer0.mode = SYS_TIMER_INTERVAL_MODE;
  timer0.handler = timer0cb;
  SYS_TimerStart(&timer0);

  timer1.interval = 1234;
  timer1.mode = SYS_TIMER_PERIODIC_MODE;
  timer1.handler = timer1cb;
  SYS_TimerStart(&timer1);

  timer2.interval = 32;
  timer2.mode = SYS_TIMER_PERIODIC_MODE;
  timer2.handler = timer2cb;
  SYS_TimerStart(&timer2);

  while (1)
  {
    SYS_TaskRun();
  }

  return 0;
}

