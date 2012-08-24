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
#ifdef PER_APP
	#include "sysTimer.h"
	#include "phy.h"
#endif

/*****************************************************************************
*****************************************************************************/
#ifdef PER_APP
	// Create a 50mS callback timer.
	SYS_Timer_t txn_timer;

	// Create a 35S callback timer.
	SYS_Timer_t rxn_timer;

	// Create a variable to let the per app know it can transmit the next
	// frame.
	uint8_t perAppDataBusy;
#endif
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

#ifdef PER_APP
  perAppDataBusy = false;
#endif

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

/*
 * PER IMPLEMENTATION NOTES!
 *
 * The TXN will transmit 20 byte pseudo-random frames at 10 mS
 * intervals. It will transmit 250 frames. After frame transmission
 * is complete it will return to RX mode.
 *
 * The RXN will set a timer for 5 seconds which is 2x the time
 * required for it to receive the 250 frames.
 *
 * When the timer expires it will send the "test complete" frame
 * to the PCN.
 *
 * This will prompt the PCN to send a "send data" frame to the RXN
 * and the RXN will send three 64 byte payload frames and then a 58
 * byte payload frame (250 bytes of test data) - This will be LQI
 * data. The 4 frames will be repeated to send the RSSI data.
 *
 * The PCN will then post process the LQI and RSSI data to determine
 * PER results.
 */

#ifdef PER_APP

typedef struct PACK
{
  uint8_t      id;
  uint16_t     dst;
  uint8_t      options;
  uint8_t      handle;
  uint8_t	   payload[8];
} PerAppCommandDataReq_t;

extern AppIb_t appIb; // Declared in ib.c

extern uint8_t appUartCmdBuffer[APP_UART_CMD_BUFFER_SIZE];
extern uint8_t appUartCmdSize;

uint8_t dumbass_flag = 0;

	// We send 250 frames at 50mS intervals so we should hit
	// the call to perSendFrame 250 times. Count the number
	// of calls and stop it when 250 is reached.
	// uint8_t per_count = 0; is declared in serial.h and initialized
	// in the appInit function in serial.c.
	void perSendFrame(SYS_Timer_t *timer)
	{

		// If the confirm has not been received bail out.
		// Set to true in the PHY.
		if(perAppDataBusy)
			return;
		// Now its busy sending a frame...
		//perAppDataBusy = true;

//		if(per_count < 11)
		if(per_count < 251)
		{
			per_count++;

			// Create and send a PER frame to the RXN (0x0002). It has to
			// Look like it came in over the UART:
			PerAppCommandDataReq_t dr;
			dr.id = APP_COMMAND_DATA_REQ;
#ifdef TEST_MODE
			dr.dst = 0x0000;
#else
			dr.dst = 0x2222;
#endif
			dr.options = 0;
			dr.handle = 42;
			// Create some pseudo-random payload data. Don't let us collide with real commands.
			for(uint8_t i=0; i<8; i++)
				dr.payload[i] = per_count & 0x0F;

			// Fake out the UART task handler so that this frame gets sent.
			appUartState = APP_UART_STATE_OK;
			appState = APP_STATE_COMMAND_RECEIVED;
			SYS_PortSet(APP_PORT);

			memcpy(appUartCmdBuffer, (uint8_t *)&dr, sizeof(PerAppCommandDataReq_t));
			appUartCmdSize = 14;

			// Timer only has to be started once.
			if(!dumbass_flag)
			{
				SYS_TimerStart(&txn_timer);
				dumbass_flag = 1;
			}
		}
		else
		{
			ledOff();
			per_count = 0;
			// Turn the UART back on.
			// ota_enabled = 0;
			phyTrxSetState(TRX_CMD_RX_ON);
			SYS_TimerStop(&txn_timer);
		}
	}

	void perReceiveFrame(SYS_Timer_t *timer)
	{
		ledOff();

		//Stop the timer (should only occur once anyway)...
		SYS_TimerStop(&rxn_timer);

		// Total frames received for inclusion
		uint8_t totalFrames = 0;

		for(int i = 0; i < 256; i++) {
			totalFrames += lqi_buf[i];
		}

		// Send PER test complete frame to PCN.
		// Create and send a PER frame to the PCN (0x0000). It has to
		// Look like it came in over the UART:
		PerAppCommandDataReq_t dr;
		dr.id = APP_COMMAND_DATA_REQ;
		dr.dst = 0x0000;
		dr.options = 0;
		dr.handle = 42;
		// Insert the OTA "Test Complete" ID.
		dr.payload[0] = APP_COMMAND_TEST_COMPLETE;
		dr.payload[1] = totalFrames;

		// Fake out the UART task handler so that this frame gets sent.
		appUartState = APP_UART_STATE_OK;
		appState = APP_STATE_COMMAND_RECEIVED;
		SYS_PortSet(APP_PORT);

		memcpy(appUartCmdBuffer, (uint8_t *)&dr, sizeof(PerAppCommandDataReq_t));
		appUartCmdSize = 8;

		// Turn the UART back on.
		// ota_enabled = 0;
	}

	AppStatus_t appCommandStartTestReqHandler(uint8_t *buf, uint8_t size)
	{
		// Don't need or care about the parameters - They are used for
		// command dispatcher consistency.
		// AppCommandStartTest_t *req = (AppCommandStartTest_t *)buf;

		// Turn off UART while PER test is running.
		ota_enabled = 1;

		// @todo	Add logic to start the PER test here.
		// TXN address is: 0x0001
		// RXN address is: 0x0002
		if(0x1111 == appIb.addr)
		{
			// To prevent re-initialization of the timer when the test is
			// restarted.
			dumbass_flag = 0;
			ledOn();
			// Initialize the 50mS interval timer
			txn_timer.interval = 50;
			txn_timer.mode = SYS_TIMER_PERIODIC_MODE;
			txn_timer.handler = perSendFrame;

			// Init the frame send flag.
			perAppDataBusy = false;

			// Send the frame
			perSendFrame(&txn_timer);

		}
#ifdef TEST_MODE
		else if(0x0000 == appIb.addr)
#else
		else if(0x2222 == appIb.addr)
#endif
		{
			ledOn();
			// Init the two buffers/histograms.
		    memset(rssi_buf, 0, 256);
		    memset(lqi_buf, 0, 256);

			// Create a 20 Second timer and callback function. When it expires
			// it should send the "test complete" frame to the PCN (0x0000).

			// Initialize the 20S timer.
			rxn_timer.interval = 20000;
			rxn_timer.mode = SYS_TIMER_PERIODIC_MODE;
			rxn_timer.handler = perReceiveFrame;
			SYS_TimerStart(&rxn_timer);

			// Ensure the radio is set to RX mode.
			phyTrxSetState(TRX_CMD_RX_ON);

		}

		//(void)size;
		return APP_STATUS_SUCESS;
	}

	AppStatus_t appCommandTestCompleteHandler(uint8_t *buf, uint8_t size)
	{
		// Don't need or care about the parameters - They are used for
		// command dispatcher consistency.
		//AppCommandTestComplete_t *req = (AppCommandTestComplete_t *)buf;

		// @todo	Add logic to receive the PER test complete message OTA here.
		// The PCN is the receipient node.

		(void)size;
		return APP_STATUS_SUCESS;
	}

	AppStatus_t appCommandSendTestDataReqHandler(uint8_t *buf, uint8_t size)
	{
		// Don't need or care about the parameters - They are used for
		// command dispatcher consistency.
		//AppCommandSendTestData_t *req = (AppCommandSendTestData_t *)buf;

		// @todo	Add logic to send the PER test data OTA here.
		// The RXN is the recipient node here.

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

