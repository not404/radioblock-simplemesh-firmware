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

#include <stdio.h>
#include <string.h>
#include "config.h"
#include "nwk.h"
#include "phy.h"
#include "sysTimer.h"
#include "sysMem.h"
#include "sysTaskManager.h"
#include "halUart.h"
#include "halSleep.h"
#include "led.h"

/*****************************************************************************
*****************************************************************************/
#if APP_ADDR == 0
  #define APP_NODE_TYPE   0
  #define APP_COORDINATOR 1
#elif APP_ADDR < 0x8000
  #define APP_NODE_TYPE   1
  #define APP_ROUTER      1
#else
  #define APP_NODE_TYPE   2
  #define APP_ENDDEVICE   1
#endif

#define UART_TX_BUFFER_SIZE  100
#define UART_RX_BUFFER_SIZE  10

/*****************************************************************************
*****************************************************************************/
typedef enum AppState_t
{
  APP_STATE_INITIAL,
  APP_STATE_SEND_DATA,
  APP_STATE_WAIT_DATA_CONF,
  APP_STATE_DATA_SENT,
  APP_STATE_WAIT_SENDING_TIMER,
  APP_STATE_PREPARE_TO_SLEEP,
  APP_STATE_WAIT_SLEEP_CONF,
  APP_STATE_SLEEP,
  APP_STATE_WAKEUP,
  APP_STATE_WAIT_WAKEUP_CONF,
} AppState_t;

typedef struct PACK AppMessage_t
{
  uint8_t     type;
  uint8_t     nodeType;
  uint64_t    extAddr;
  uint16_t    addr;
  uint32_t    version;
  uint32_t    channelMask;
  uint16_t    panId;
  uint8_t     channel;
  uint16_t    parent;
  uint8_t     lqi;
  int8_t      rssi;

  struct PACK
  {
    uint8_t   type;
    uint8_t   size;
    int32_t   battery;
    int32_t   temperature;
    int32_t   light;
  } sensors;
} AppMessage_t;

/*****************************************************************************
*****************************************************************************/
#if APP_ROUTER || APP_ENDDEVICE
static void appDataConf(NWK_DataReq_t *req);
#endif
#if APP_ROUTER || APP_COORDINATOR
static void appDataSendingTimerHandler(SYS_Timer_t *timer);
#endif

/*****************************************************************************
*****************************************************************************/
static AppState_t appState = APP_STATE_INITIAL;
static AppMessage_t appMessage;

#if APP_ROUTER || APP_ENDDEVICE
static NWK_DataReq_t appDataReq;
#endif

#if APP_COORDINATOR
static HAL_Uart_t appUart;
static uint8_t appUartTxBuffer[UART_TX_BUFFER_SIZE];
static uint8_t appUartRxBuffer[UART_RX_BUFFER_SIZE];
#endif

#if APP_ROUTER || APP_COORDINATOR
static SYS_Timer_t appDataSendingTimer;
#endif

/*****************************************************************************
*****************************************************************************/
static uint16_t rand_next(void)
{
  static uint32_t a = 1;
  a = (a * 0x7fcf + 3) % 0x7fed;
  return a & 0xffff;
}

#if APP_COORDINATOR
/*****************************************************************************
*****************************************************************************/
static void appSendMessageToUart(uint8_t *data, uint8_t size)
{
  uint8_t csum = 0;

  HAL_UartWriteByte(0x10);
  HAL_UartWriteByte(0x02);

  for (uint8_t i = 0; i < size; i++)
  {
    if (data[i] == 0x10)
    {
      HAL_UartWriteByte(0x10);
      csum += 0x10;
    }
    HAL_UartWriteByte(data[i]);
    csum += data[i];
  }

  HAL_UartWriteByte(0x10);
  HAL_UartWriteByte(0x03);
  csum += 0x10 + 0x02 + 0x10 + 0x03;

  HAL_UartWriteByte(csum);
}

/*****************************************************************************
*****************************************************************************/
void appUartRxCallback(uint16_t bytes)
{
  for (uint16_t i = 0; i < bytes; i++)
    HAL_UartReadByte();
}
#endif

/*****************************************************************************
*****************************************************************************/
static bool appDataInd(NWK_DataInd_t *ind)
{
#if APP_COORDINATOR
  AppMessage_t *msg = (AppMessage_t *)ind->data;

  msg->lqi = ind->lqi;
  msg->rssi = ind->rssi;

  appSendMessageToUart(ind->data, ind->size);

  ledToggle();
#else
  (void)ind;
#endif
  return true;
}

/*****************************************************************************
*****************************************************************************/
static void appInit(void)
{
  appMessage.type                 = 1;
  appMessage.nodeType             = APP_NODE_TYPE;
  appMessage.extAddr              = APP_ADDR;
  appMessage.addr                 = APP_ADDR;
  appMessage.version              = 0x01010100;
  appMessage.channelMask          = (1L << APP_CHANNEL);
  appMessage.panId                = APP_PANID;
  appMessage.channel              = APP_CHANNEL;
  appMessage.parent               = 0;
  appMessage.lqi                  = 0;
  appMessage.rssi                 = 0;

  appMessage.sensors.type         = 1;
  appMessage.sensors.size         = sizeof(int32_t) * 3;
  appMessage.sensors.battery      = 0;
  appMessage.sensors.temperature  = 0;
  appMessage.sensors.light        = 0;

#if APP_ENDDEVICE
  // lower power consumption
#endif
  ledInit();

#if APP_COORDINATOR
  appUart.bits         = HAL_UART_BITS_8;
  appUart.parity       = HAL_UART_PARITY_NONE;
  appUart.stop         = HAL_UART_STOP_BITS_1;
  appUart.baudrate     = HAL_UART_BAUDRATE_38400;
  appUart.txBuffer     = appUartTxBuffer;
  appUart.txBufferSize = sizeof(appUartTxBuffer);
  appUart.rxBuffer     = appUartRxBuffer;
  appUart.rxBufferSize = sizeof(appUartRxBuffer);
  appUart.rxCallback   = appUartRxCallback;
  appUart.txCallback   = NULL;
  HAL_UartInit(&appUart);
#endif

  NWK_SetAddr(APP_ADDR);
  NWK_SetPanId(APP_PANID);
  NWK_SetSecurityKey((uint8_t *)APP_SECURITY_KEY);
  PHY_SetChannel(APP_CHANNEL);
  PHY_SetRxState(true);

#if APP_ROUTER || APP_COORDINATOR
  appDataSendingTimer.interval = APP_SENDING_INTERVAL;
  appDataSendingTimer.mode = SYS_TIMER_INTERVAL_MODE;
  appDataSendingTimer.handler = appDataSendingTimerHandler;
#endif

  appState = APP_STATE_SEND_DATA;
  SYS_PortSet(APP_PORT);
}

/*****************************************************************************
*****************************************************************************/
static void appSendData(void)
{
  appMessage.parent = NWK_RouteNextHop(0);

  appMessage.sensors.battery     = rand_next();
  appMessage.sensors.temperature = rand_next();
  appMessage.sensors.light       = rand_next();

#if APP_ROUTER || APP_ENDDEVICE
  appState = APP_STATE_WAIT_DATA_CONF;

  appDataReq.dst = 0;
  appDataReq.port = APP_PORT;
  appDataReq.options = NWK_OPT_ACK_REQUEST | NWK_OPT_SECURITY_ENABLED;
  appDataReq.data = (uint8_t *)&appMessage;
  appDataReq.size = sizeof(appMessage);
  appDataReq.confirm = appDataConf;
  NWK_DataReq(&appDataReq);

  ledOn();
#else
  appSendMessageToUart((uint8_t *)&appMessage, sizeof(appMessage));
  SYS_TimerStart(&appDataSendingTimer);
  appState = APP_STATE_WAIT_SENDING_TIMER;
#endif
}

#if APP_ROUTER || APP_ENDDEVICE
/*****************************************************************************
*****************************************************************************/
static void appDataConf(NWK_DataReq_t *req)
{
  ledOff();
  appState = APP_STATE_DATA_SENT;
  SYS_PortSet(APP_PORT);
  (void)req;
}
#endif

#if APP_ROUTER || APP_COORDINATOR
/*****************************************************************************
*****************************************************************************/
static void appDataSendingTimerHandler(SYS_Timer_t *timer)
{
  appState = APP_STATE_SEND_DATA;
  SYS_PortSet(APP_PORT);

  (void)timer;
}
#endif

/*****************************************************************************
*****************************************************************************/
static void appTaskHandler(void)
{
  switch (appState)
  {
    case APP_STATE_INITIAL:
    {
      appInit();
    } break;

    case APP_STATE_SEND_DATA:
    {
      appSendData();
    } break;

    case APP_STATE_DATA_SENT:
    {
#if APP_ENDDEVICE
      appState = APP_STATE_PREPARE_TO_SLEEP;
      SYS_PortSet(APP_PORT);
#else
      SYS_TimerStart(&appDataSendingTimer);
      appState = APP_STATE_WAIT_SENDING_TIMER;
#endif
    } break;

#if APP_ENDDEVICE
    case APP_STATE_PREPARE_TO_SLEEP:
    {
      NWK_SleepReq();
      appState = APP_STATE_WAIT_SLEEP_CONF;
    } break;

    case APP_STATE_SLEEP:
    {
      PHY_SetRxState(false);
      HAL_Sleep(APP_SENDING_INTERVAL);
      appState = APP_STATE_WAKEUP;
      SYS_PortSet(APP_PORT);
    } break;

    case APP_STATE_WAKEUP:
    {
      NWK_WakeupReq();
      appState = APP_STATE_WAIT_WAKEUP_CONF;
    } break;
#endif

    default:
      break;
  }
}

#if APP_ENDDEVICE
/*****************************************************************************
*****************************************************************************/
void NWK_SleepConf(void)
{
  appState = APP_STATE_SLEEP;
  SYS_PortSet(APP_PORT);
}

/*****************************************************************************
*****************************************************************************/
void NWK_WakeupConf(void)
{
  PHY_SetRxState(true);
  appState = APP_STATE_SEND_DATA;
  SYS_PortSet(APP_PORT);
}
#endif

/*****************************************************************************
*****************************************************************************/
int main(void)
{
  NWK_Init();
  NWK_PortOpen(APP_PORT, appTaskHandler, appDataInd);

  while (1)
  {
    SYS_TaskRun();
  }

  return 0;
}

