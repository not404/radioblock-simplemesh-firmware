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

#include <stdlib.h>
#include <string.h>
#include "config.h"
#include "nwk.h"
#include "phy.h"
#include "sysTypes.h"
#include "halUart.h"
#include "serial.h"
#include "led.h"

/*****************************************************************************
*****************************************************************************/
#define AT_LEAST     0x80

/*****************************************************************************
*****************************************************************************/
AppStatus_t appCommandTestReqHandler(uint8_t *buf, uint8_t size);
AppStatus_t appCommandResetReqHandler(uint8_t *buf, uint8_t size);
AppStatus_t appCommandTestReqHandler(uint8_t *buf, uint8_t size);
AppStatus_t appCommandResetReqHandler(uint8_t *buf, uint8_t size);
AppStatus_t appCommandSettingsReqHandler(uint8_t *buf, uint8_t size);
AppStatus_t appCommandSetUartModeReqHandler(uint8_t *buf, uint8_t size);
AppStatus_t appCommandSleepReqHandler(uint8_t *buf, uint8_t size);
AppStatus_t appCommandDataReqHandler(uint8_t *buf, uint8_t size);
AppStatus_t appCommandSetAddrReqHandler(uint8_t *buf, uint8_t size);
AppStatus_t appCommandGetAddrReqHandler(uint8_t *buf, uint8_t size);
AppStatus_t appCommandSetPanIdReqHandler(uint8_t *buf, uint8_t size);
AppStatus_t appCommandGetPanIdReqHandler(uint8_t *buf, uint8_t size);
AppStatus_t appCommandSetChannelReqHandler(uint8_t *buf, uint8_t size);
AppStatus_t appCommandGetChannelReqHandler(uint8_t *buf, uint8_t size);
AppStatus_t appCommandSetRxStateReqHandler(uint8_t *buf, uint8_t size);
AppStatus_t appCommandGetRxStateReqHandler(uint8_t *buf, uint8_t size);
AppStatus_t appCommandSetTxPowerReqHandler(uint8_t *buf, uint8_t size);
AppStatus_t appCommandGetTxPowerReqHandler(uint8_t *buf, uint8_t size);
AppStatus_t appCommandSetSecurityKeyReqHandler(uint8_t *buf, uint8_t size);
AppStatus_t appCommandSetAckStateReqHandler(uint8_t *buf, uint8_t size);
AppStatus_t appCommandGetAckStateReqHandler(uint8_t *buf, uint8_t size);
AppStatus_t appCommandSetLedStateReqHandler(uint8_t *buf, uint8_t size);

#ifdef PER_APP
	AppStatus_t appCommandStartTestReqHandler(uint8_t *buf, uint8_t size);
	AppStatus_t appCommandTestCompleteHandler(uint8_t *buf, uint8_t size);
	AppStatus_t appCommandSendTestDataReqHandler(uint8_t *buf, uint8_t size);
#endif

static void appDataConf(NWK_DataReq_t *req);

/*****************************************************************************
*****************************************************************************/
typedef union PACK AppRespCommand_t
{
  AppCommandTestResp_t         test;
  AppCommandGetAddrResp_t      addr;
  AppCommandGetPanIdResp_t     panId;
  AppCommandGetChannelResp_t   channel;
  AppCommandGetRxStateResp_t   rxState;
  AppCommandGetTxPowerResp_t   txPower;
  AppCommandGetAckStateResp_t  ackState;
} AppRespCommand_t;

typedef struct AppCommandRecord_t
{
   AppCommandId_t        id;
   uint8_t               size;
   AppCommandHandler_t  *handler;
} AppCommandRecord_t;

/*****************************************************************************
*****************************************************************************/
static AppCommandRecord_t records[] =
{
  { APP_COMMAND_TEST_REQ,             sizeof(AppCommandTestReq_t),            appCommandTestReqHandler },
  { APP_COMMAND_RESET_REQ,            sizeof(AppCommandResetReq_t),           appCommandResetReqHandler },
  { APP_COMMAND_SETTINGS_REQ,         sizeof(AppCommandSettingsReq_t),        appCommandSettingsReqHandler },
  { APP_COMMAND_SET_UART_MODE_REQ,    sizeof(AppCommandSetUartModeReq_t),     appCommandSetUartModeReqHandler },
  { APP_COMMAND_SLEEP_REQ,            sizeof(AppCommandSleepReq_t),           appCommandSleepReqHandler },
  { APP_COMMAND_DATA_REQ,             AT_LEAST | sizeof(AppCommandDataReq_t), appCommandDataReqHandler },
  { APP_COMMAND_SET_ADDR_REQ,         sizeof(AppCommandSetAddrReq_t),         appCommandSetAddrReqHandler },
  { APP_COMMAND_GET_ADDR_REQ,         sizeof(AppCommandGetAddrReq_t),         appCommandGetAddrReqHandler },
  { APP_COMMAND_SET_PANID_REQ,        sizeof(AppCommandSetPanIdReq_t),        appCommandSetPanIdReqHandler },
  { APP_COMMAND_GET_PANID_REQ,        sizeof(AppCommandGetPanIdReq_t),        appCommandGetPanIdReqHandler },
  { APP_COMMAND_SET_CHANNEL_REQ,      sizeof(AppCommandSetChannelReq_t),      appCommandSetChannelReqHandler },
  { APP_COMMAND_GET_CHANNEL_REQ,      sizeof(AppCommandGetChannelReq_t),      appCommandGetChannelReqHandler },
  { APP_COMMAND_SET_RX_STATE_REQ,     sizeof(AppCommandSetRxStateReq_t),      appCommandSetRxStateReqHandler },
  { APP_COMMAND_GET_RX_STATE_REQ,     sizeof(AppCommandGetRxStateReq_t),      appCommandGetRxStateReqHandler },
  { APP_COMMAND_SET_TX_POWER_REQ,     sizeof(AppCommandSetTxPowerReq_t),      appCommandSetTxPowerReqHandler },
  { APP_COMMAND_GET_TX_POWER_REQ,     sizeof(AppCommandGetTxPowerReq_t),      appCommandGetTxPowerReqHandler },
  { APP_COMMAND_SET_SECURITY_KEY_REQ, sizeof(AppCommandSetSecurityKeyReq_t),  appCommandSetSecurityKeyReqHandler },
  { APP_COMMAND_SET_ACK_STATE_REQ,    sizeof(AppCommandSetAckStateReq_t),     appCommandSetAckStateReqHandler },
  { APP_COMMAND_GET_ACK_STATE_REQ,    sizeof(AppCommandGetAckStateReq_t),     appCommandGetAckStateReqHandler },
  { APP_COMMAND_SET_LED_STATE_REQ,    sizeof(AppCommandSetLedStateReq_t),     appCommandSetLedStateReqHandler },
#ifdef PER_APP
  { APP_COMMAND_START_TEST_REQ,    	  sizeof(AppCommandStartTest_t),          appCommandStartTestReqHandler },
  { APP_COMMAND_TEST_COMPLETE,        sizeof(AppCommandTestComplete_t),       appCommandTestCompleteHandler },
  { APP_COMMAND_SEND_DATA_REQ,        sizeof(AppCommandSendTestData_t),       appCommandSendTestDataReqHandler },
#endif
};

static uint32_t vBaudrates[32] =
{
  HAL_UART_BAUDRATE_AUTO,
  HAL_UART_BAUDRATE_50,
  HAL_UART_BAUDRATE_75,
  HAL_UART_BAUDRATE_110,
  HAL_UART_BAUDRATE_150,
  HAL_UART_BAUDRATE_300,
  HAL_UART_BAUDRATE_1200,
  HAL_UART_BAUDRATE_2400,
  HAL_UART_BAUDRATE_4800,
  HAL_UART_BAUDRATE_9600,
  HAL_UART_BAUDRATE_19200,
  HAL_UART_BAUDRATE_38400,
  HAL_UART_BAUDRATE_57600,
  HAL_UART_BAUDRATE_115200,
  HAL_UART_BAUDRATE_230400,
  HAL_UART_BAUDRATE_460800,
  HAL_UART_BAUDRATE(195, 12, 13), // 2000
  HAL_UART_BAUDRATE(100, 7, 8),   // 4000
  HAL_UART_BAUDRATE(50, 7, 8),    // 8000
  HAL_UART_BAUDRATE(39, 12, 13),  // 10000
  HAL_UART_BAUDRATE(20, 7, 8),    // 20000
  HAL_UART_BAUDRATE(13, 12, 13),  // 30000
  HAL_UART_BAUDRATE(10, 7, 8),    // 40000
  HAL_UART_BAUDRATE(8, 7, 8),     // 50000
  HAL_UART_BAUDRATE(7, 11, 14),   // 60000
  HAL_UART_BAUDRATE(6, 11, 14),   // 70000
  HAL_UART_BAUDRATE(5, 7, 8),     // 80000
  HAL_UART_BAUDRATE(5, 2, 3),     // 90000
  HAL_UART_BAUDRATE(4, 7, 8),     // 100000
  HAL_UART_BAUDRATE(2, 7, 8),     // 200000
  HAL_UART_BAUDRATE(2, 1, 4),     // 300000
  HAL_UART_BAUDRATE(1, 7, 8),     // 400000
};

static uint8_t vBits[4] =
{
  HAL_UART_BITS_5,
  HAL_UART_BITS_6,
  HAL_UART_BITS_7,
  HAL_UART_BITS_8,
};

static uint8_t vStopBits[2] =
{
  HAL_UART_STOP_BITS_1,
  HAL_UART_STOP_BITS_2,
};

static uint8_t vParity[5] =
{
  HAL_UART_PARITY_NONE,
  HAL_UART_PARITY_ODD,
  HAL_UART_PARITY_EVEN,
  HAL_UART_PARITY_FORCE_1,
  HAL_UART_PARITY_FORCE_0,
};

static AppRespCommand_t response;
static uint8_t responseSize = 0;
static NWK_DataReq_t appDataReq;
static bool appDataReqBusy = false;
static uint8_t appDataReqHandle;

/*****************************************************************************
*****************************************************************************/
static void appDataConf(NWK_DataReq_t *req)
{
  AppCommandDataConf_t conf;

  conf.id = APP_COMMAND_DATA_CONF;
  conf.status = req->status;
  conf.handle = appDataReqHandle;

  appUartSendCommand((uint8_t *)&conf, sizeof(conf));
  appDataReqBusy = false;

  (void)req;
}

/*****************************************************************************
*****************************************************************************/
uint8_t *appCommandResponse(void)
{
  if (responseSize)
    return (uint8_t *)&response;
  return NULL;
}

/*****************************************************************************
*****************************************************************************/
uint8_t appCommandResponseSize(void)
{
  return responseSize;
}

/*****************************************************************************
*****************************************************************************/
AppStatus_t appCommandTestReqHandler(uint8_t *buf, uint8_t size)
{
  response.test.id = APP_COMMAND_TEST_RESP;
  responseSize = sizeof(AppCommandTestResp_t);

  (void)buf;
  (void)size;
  return APP_STATUS_SUCESS;
}

/*****************************************************************************
*****************************************************************************/
AppStatus_t appCommandResetReqHandler(uint8_t *buf, uint8_t size)
{
  (void)buf;
  (void)size;
  return APP_STATUS_SUCESS;
}

/*****************************************************************************
*****************************************************************************/
AppStatus_t appCommandSettingsReqHandler(uint8_t *buf, uint8_t size)
{
  AppCommandSettingsReq_t *req = (AppCommandSettingsReq_t *)buf;

  if (APP_SETTINGS_SAVE == req->operation)
  {
    if (appIbPdsSave())
      return APP_STATUS_SUCESS;
    else
      return APP_STATUS_FLASH_ERROR;
  }
  else if (APP_SETTINGS_SET_DEFAULTS == req->operation)
  {
    appSetDefaults = true;
    return APP_STATUS_SUCESS;
  }

  (void)size;
  return APP_STATUS_MALFORMED_COMMAND;
}

/*****************************************************************************
*****************************************************************************/
AppStatus_t appCommandSetUartModeReqHandler(uint8_t *buf, uint8_t size)
{
  AppCommandSetUartModeReq_t *req = (AppCommandSetUartModeReq_t *)buf;
  uint8_t bits, parity, stop;
  uint32_t baudrate;

  if (req->bits < ARRAY_SIZE(vBits))
    bits = vBits[req->bits];
  else
    return APP_STATUS_MALFORMED_COMMAND;

  if (req->parity < ARRAY_SIZE(vParity))
    parity = vParity[req->parity];
  else
    return APP_STATUS_MALFORMED_COMMAND;

  if (req->stop < ARRAY_SIZE(vStopBits))
    stop = vStopBits[req->stop];
  else
    return APP_STATUS_MALFORMED_COMMAND;

  if (req->baudrate < ARRAY_SIZE(vBaudrates))
    baudrate = vBaudrates[req->baudrate];
  else
    return APP_STATUS_MALFORMED_COMMAND;
  
  appUpdateUart = true;
  appIb.bits = bits;
  appIb.parity = parity;
  appIb.stop = stop;
  appIb.baudrate = baudrate;

  (void)size;
  return APP_STATUS_SUCESS;
}

/*****************************************************************************
*****************************************************************************/
AppStatus_t appCommandSleepReqHandler(uint8_t *buf, uint8_t size)
{
  AppCommandSleepReq_t *req = (AppCommandSleepReq_t *)buf;

  appSleepReqInterval = req->interval;

  (void)size;
  return APP_STATUS_SUCESS;
}

/*****************************************************************************
*****************************************************************************/
AppStatus_t appCommandDataReqHandler(uint8_t *buf, uint8_t size)
{
  AppCommandDataReq_t *req = (AppCommandDataReq_t *)buf;
  uint8_t maxSize;

  appDataReqBusy = true;
  appDataReqHandle = req->handle;

  appDataReq.dst = req->dst;
  appDataReq.port = APP_PORT;
  appDataReq.options = req->options;
  appDataReq.data = (uint8_t *)req + sizeof(AppCommandDataReq_t);
  appDataReq.size = size - sizeof(AppCommandDataReq_t);
  appDataReq.confirm = appDataConf;

  if (appDataReq.options & NWK_OPT_SECURITY_ENABLED)
     maxSize = NWK_MAX_SECURED_PAYLOAD_SIZE;
  else
     maxSize = NWK_MAX_PAYLOAD_SIZE;

  if (appDataReq.size > maxSize)
    return APP_STATUS_INVALID_PAYLOAD_SIZE;

  NWK_DataReq(&appDataReq);

  return APP_STATUS_SUCESS;
}

/*****************************************************************************
*****************************************************************************/
AppStatus_t appCommandSetAddrReqHandler(uint8_t *buf, uint8_t size)
{
  AppCommandSetAddrReq_t *req = (AppCommandSetAddrReq_t *)buf;

  NWK_SetAddr(req->addr);
  appIb.addr = req->addr;

  (void)size;
  return APP_STATUS_SUCESS;
}

/*****************************************************************************
*****************************************************************************/
AppStatus_t appCommandGetAddrReqHandler(uint8_t *buf, uint8_t size)
{
  response.addr.id = APP_COMMAND_GET_ADDR_RESP;
  response.addr.addr = appIb.addr;
  responseSize = sizeof(AppCommandGetAddrResp_t);

  (void)buf;
  (void)size;
  return APP_STATUS_SUCESS;
}

/*****************************************************************************
*****************************************************************************/
AppStatus_t appCommandSetPanIdReqHandler(uint8_t *buf, uint8_t size)
{
  AppCommandSetPanIdReq_t *req = (AppCommandSetPanIdReq_t *)buf;

  NWK_SetPanId(req->panId);
  appIb.panId = req->panId;

  (void)size;
  return APP_STATUS_SUCESS;
}

/*****************************************************************************
*****************************************************************************/
AppStatus_t appCommandGetPanIdReqHandler(uint8_t *buf, uint8_t size)
{
  response.panId.id = APP_COMMAND_GET_PANID_RESP;
  response.panId.panId = appIb.panId;
  responseSize = sizeof(AppCommandGetPanIdResp_t);

  (void)buf;
  (void)size;
  return APP_STATUS_SUCESS;
}

/*****************************************************************************
*****************************************************************************/
AppStatus_t appCommandSetChannelReqHandler(uint8_t *buf, uint8_t size)
{
  AppCommandSetChannelReq_t *req = (AppCommandSetChannelReq_t *)buf;

  PHY_SetChannel(req->channel);
  appIb.channel = req->channel;

  (void)size;
  return APP_STATUS_SUCESS;
}

/*****************************************************************************
*****************************************************************************/
AppStatus_t appCommandGetChannelReqHandler(uint8_t *buf, uint8_t size)
{
  response.channel.id = APP_COMMAND_GET_CHANNEL_RESP;
  response.channel.channel = appIb.channel;
  responseSize = sizeof(AppCommandGetChannelResp_t);

  (void)buf;
  (void)size;
  return APP_STATUS_SUCESS;
}

/*****************************************************************************
*****************************************************************************/
AppStatus_t appCommandSetRxStateReqHandler(uint8_t *buf, uint8_t size)
{
  AppCommandSetRxStateReq_t *req = (AppCommandSetRxStateReq_t *)buf;

  PHY_SetRxState(req->rxState);
  appIb.rxState = req->rxState;

  (void)size;
  return APP_STATUS_SUCESS;
}

/*****************************************************************************
*****************************************************************************/
AppStatus_t appCommandGetRxStateReqHandler(uint8_t *buf, uint8_t size)
{
  response.rxState.id = APP_COMMAND_GET_RX_STATE_RESP;
  response.rxState.rxState = appIb.rxState;
  responseSize = sizeof(AppCommandGetRxStateResp_t);

  (void)buf;
  (void)size;
  return APP_STATUS_SUCESS;
}

/*****************************************************************************
*****************************************************************************/
AppStatus_t appCommandSetTxPowerReqHandler(uint8_t *buf, uint8_t size)
{
  AppCommandSetTxPowerReq_t *req = (AppCommandSetTxPowerReq_t *)buf;

  PHY_SetTxPower(req->txPower);
  appIb.txPower = req->txPower;

  (void)size;
  return APP_STATUS_SUCESS;
}

/*****************************************************************************
*****************************************************************************/
AppStatus_t appCommandGetTxPowerReqHandler(uint8_t *buf, uint8_t size)
{
  response.txPower.id = APP_COMMAND_GET_TX_POWER_RESP;
  response.txPower.txPower = appIb.txPower;
  responseSize = sizeof(AppCommandGetTxPowerResp_t);

  (void)buf;
  (void)size;
  return APP_STATUS_SUCESS;
}

/*****************************************************************************
*****************************************************************************/
AppStatus_t appCommandSetAckStateReqHandler(uint8_t *buf, uint8_t size)
{
  AppCommandSetAckStateReq_t *req = (AppCommandSetAckStateReq_t *)buf;

  appIb.ackState = req->state;

  (void)size;
  return APP_STATUS_SUCESS;
}

/*****************************************************************************
*****************************************************************************/
AppStatus_t appCommandGetAckStateReqHandler(uint8_t *buf, uint8_t size)
{
  response.ackState.id = APP_COMMAND_GET_TX_POWER_RESP;
  response.ackState.state = appIb.ackState;
  responseSize = sizeof(AppCommandGetAckStateResp_t);

  (void)buf;
  (void)size;
  return APP_STATUS_SUCESS;
}

/*****************************************************************************
*****************************************************************************/
AppStatus_t appCommandSetSecurityKeyReqHandler(uint8_t *buf, uint8_t size)
{
  AppCommandSetSecurityKeyReq_t *req = (AppCommandSetSecurityKeyReq_t *)buf;

  NWK_SetSecurityKey(req->key);
  memcpy(appIb.key, req->key, 16);

  (void)size;
  return APP_STATUS_SUCESS;
}

/*****************************************************************************
*****************************************************************************/
AppStatus_t appCommandSetLedStateReqHandler(uint8_t *buf, uint8_t size)
{
  AppCommandSetLedStateReq_t *req = (AppCommandSetLedStateReq_t *)buf;

  if (req->ledState & 0x02)
  {
    ledToggle();
    appIb.ledState ^= 1;
  }
  else if (req->ledState & 0x01)
  {
    ledOn();
    appIb.ledState = 1;
  }
  else
  {
    ledOff();
    appIb.ledState = 0;
  }

  (void)size;
  return APP_STATUS_SUCESS;
}

/*****************************************************************************
*****************************************************************************/
#ifdef PER_APP
	AppStatus_t appCommandStartTestReqHandler(uint8_t *buf, uint8_t size)
	{
		AppCommandStartTest_t *req = (AppCommandStartTest_t *)buf;

		// @todo	Add logic to start the PER test here.

		(void)size;
		return APP_STATUS_SUCESS;
	}

	AppStatus_t appCommandTestCompleteHandler(uint8_t *buf, uint8_t size)
	{
		AppCommandTestComplete_t *req = (AppCommandTestComplete_t *)buf;

		// @todo	Add logic to send the PER test complete message OTA here.

		(void)size;
		return APP_STATUS_SUCESS;
	}

	AppStatus_t appCommandSendTestDataReqHandler(uint8_t *buf, uint8_t size)
	{
		AppCommandSendTestData_t *req = (AppCommandSendTestData_t *)buf;

		// @todo	Add logic to send the PER test data OTA here.

		(void)size;
		return APP_STATUS_SUCESS;
	}
#endif

/*****************************************************************************
*****************************************************************************/
AppStatus_t appCommandHandler(uint8_t *buf, uint8_t size)
{
  AppCommandHeader_t *header = (AppCommandHeader_t *)buf;

  responseSize = 0;

  if (size < sizeof(AppCommandHeader_t))
    return APP_STATUS_MALFORMED_COMMAND;

  for (uint8_t i = 0; i < (sizeof(records) / sizeof(records[0])); i++)
  {
    if (records[i].id == header->id)
    {
      if (records[i].size & AT_LEAST)
      {
        if (size < (records[i].size & ~AT_LEAST))
          return APP_STATUS_MALFORMED_COMMAND;
      }
      else
      {
        if (size != records[i].size)
          return APP_STATUS_MALFORMED_COMMAND;
      }

      return records[i].handler(buf, size);
    }
  }

  return APP_STATUS_UNKNOWN_COMMAND;
}

