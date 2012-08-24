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

#ifndef _SERIAL_H_
#define _SERIAL_H_

#include "nwk.h"

/*****************************************************************************
*****************************************************************************/
// A global variable to tell whether the app is using the UART or OTA.
// ETG uint8_t ota_enabled;
#ifdef PER_APP
// Create some variables to hold results of PER testing.
// The number of frames transmitted is always 250 for histogram purposes.
uint8_t rssi_buf[256]; // These are negative numbers. Convert in pc app
uint8_t lqi_buf[256];

// PER frame counter.
uint8_t per_count;

typedef enum AppState_t
{
  APP_STATE_INITIAL,
  APP_STATE_WAIT_COMMAND,
  APP_STATE_COMMAND_RECEIVED,
  APP_STATE_PREPARE_TO_SLEEP,
  APP_STATE_WAIT_SLEEP_CONF,
  APP_STATE_SLEEP,
  APP_STATE_WAKEUP,
  APP_STATE_WAIT_WAKEUP_CONF,
  APP_STATE_READY,

  APP_STATE_RESET_REQ,
  APP_STATE_SLEEP_REQ,
  APP_STATE_DEFAULTS_REQ,
  APP_STATE_UART_REQ,
} AppState_t;

typedef enum AppUartState_t
{
  APP_UART_STATE_IDLE,
  APP_UART_STATE_READ_SIZE,
  APP_UART_STATE_READ_DATA,
  APP_UART_STATE_READ_CRC_1,
  APP_UART_STATE_READ_CRC_2,
  APP_UART_STATE_OK,
  APP_UART_STATE_ERROR,
  APP_UART_STATE_STOP,
} AppUartState_t;

AppUartState_t appUartState;
AppState_t appState;


#define APP_UART_CMD_BUFFER_SIZE   150
#endif

/*****************************************************************************
*****************************************************************************/
typedef enum AppStatus_t
{
  APP_STATUS_SUCESS               = 0x00,
  APP_STATUS_INVALID_SIZE         = 0x80,
  APP_STATUS_INVALID_CRC          = 0x81,
  APP_STATUS_TIMEOUT              = 0x82,
  APP_STATUS_UNKNOWN_COMMAND      = 0x83,
  APP_STATUS_MALFORMED_COMMAND    = 0x84,
  APP_STATUS_FLASH_ERROR          = 0x85,
  APP_STATUS_INVALID_PAYLOAD_SIZE = 0x86,
} AppStatus_t;

typedef enum AppCommandId_t
{
  APP_COMMAND_ACK                   = 0x00,

  APP_COMMAND_TEST_REQ              = 0x01,
  APP_COMMAND_TEST_RESP             = 0x02,

  APP_COMMAND_RESET_REQ             = 0x03,
  APP_COMMAND_SETTINGS_REQ          = 0x04,
  APP_COMMAND_SET_UART_MODE_REQ     = 0x05,
  APP_COMMAND_SLEEP_REQ             = 0x06,
  APP_COMMAND_WAKEUP_IND            = 0x07,

  APP_COMMAND_DATA_REQ              = 0x20,
  APP_COMMAND_DATA_CONF             = 0x21,
  APP_COMMAND_DATA_IND              = 0x22,

  APP_COMMAND_SET_ADDR_REQ          = 0x23,
  APP_COMMAND_GET_ADDR_REQ          = 0x24,
  APP_COMMAND_GET_ADDR_RESP         = 0x25,

  APP_COMMAND_SET_PANID_REQ         = 0x26,
  APP_COMMAND_GET_PANID_REQ         = 0x27,
  APP_COMMAND_GET_PANID_RESP        = 0x28,

  APP_COMMAND_SET_CHANNEL_REQ       = 0x29,
  APP_COMMAND_GET_CHANNEL_REQ       = 0x2a,
  APP_COMMAND_GET_CHANNEL_RESP      = 0x2b,

  APP_COMMAND_SET_RX_STATE_REQ      = 0x2c,
  APP_COMMAND_GET_RX_STATE_REQ      = 0x2d,
  APP_COMMAND_GET_RX_STATE_RESP     = 0x2e,

  APP_COMMAND_SET_TX_POWER_REQ      = 0x2f,
  APP_COMMAND_GET_TX_POWER_REQ      = 0x30,
  APP_COMMAND_GET_TX_POWER_RESP     = 0x31,

  APP_COMMAND_SET_SECURITY_KEY_REQ  = 0x32,

  APP_COMMAND_SET_ACK_STATE_REQ     = 0x35,
  APP_COMMAND_GET_ACK_STATE_REQ     = 0x36,
  APP_COMMAND_GET_ACK_STATE_RESP    = 0x37,

  APP_COMMAND_SET_LED_STATE_REQ     = 0x80,

#ifdef PER_APP
  APP_COMMAND_START_TEST_REQ		= 0xFD,
  APP_COMMAND_TEST_COMPLETE			= 0xFE,
  APP_COMMAND_SEND_DATA_REQ			= 0xFF,
#endif // PER_APP
} AppCommandId_t;

enum
{
  APP_SETTINGS_SAVE                 = 0x10,
  APP_SETTINGS_SET_DEFAULTS         = 0x15,
};

typedef AppStatus_t (AppCommandHandler_t)(uint8_t *, uint8_t);

typedef struct AppIb_t
{
  uint32_t     valid;

  uint16_t     addr;
  uint16_t     panId;
  uint8_t      channel;
  uint8_t      rxState;
  uint8_t      txPower;
  uint8_t      ackState;
  uint8_t      key[16];

  uint8_t      bits;
  uint8_t      parity;
  uint8_t      stop;
  uint32_t     baudrate;

  uint8_t      ledState;
} AppIb_t;

typedef struct PACK
{
  uint8_t      id;
} AppCommandHeader_t;

typedef struct PACK
{
  uint8_t      id;
  uint8_t      status;
} AppCommandAck_t;

typedef struct PACK
{
  uint8_t      id;
} AppCommandTestReq_t;

typedef struct PACK
{
  uint8_t      id;
} AppCommandTestResp_t;

typedef struct PACK
{
  uint8_t      id;
} AppCommandResetReq_t;

typedef struct PACK
{
  uint8_t      id;
  uint8_t      operation;
} AppCommandSettingsReq_t;

typedef struct PACK
{
  uint8_t      id;
  uint8_t      bits;
  uint8_t      parity;
  uint8_t      stop;
  uint8_t      baudrate;
} AppCommandSetUartModeReq_t;

typedef struct PACK
{
  uint8_t      id;
  uint32_t     interval;
} AppCommandSleepReq_t;

typedef struct PACK
{
  uint8_t      id;
} AppCommandWakeupInd_t;

typedef struct PACK
{
  uint8_t      id;
  uint16_t     dst;
  uint8_t      options;
  uint8_t      handle;
} AppCommandDataReq_t;

typedef struct PACK
{
  uint8_t      id;
  uint8_t      status;
  uint8_t      handle;
} AppCommandDataConf_t;

typedef struct PACK
{
  uint8_t      id;
  uint16_t     src;
  uint8_t      options;
  uint8_t      lqi;
  int8_t       rssi;
} AppCommandDataIndHeader_t;

typedef struct PACK
{
  uint8_t      id;
  uint16_t     src;
  uint8_t      options;
  uint8_t      lqi;
  int8_t       rssi;
  uint8_t      payload[NWK_MAX_PAYLOAD_SIZE];
} AppCommandDataInd_t;

typedef struct PACK
{
  uint8_t      id;
  uint16_t     addr;
} AppCommandSetAddrReq_t;

typedef struct PACK
{
  uint8_t      id;
} AppCommandGetAddrReq_t;

typedef struct PACK
{
  uint8_t      id;
  uint16_t     addr;
} AppCommandGetAddrResp_t;

typedef struct PACK
{
  uint8_t      id;
  uint16_t     panId;
} AppCommandSetPanIdReq_t;

typedef struct PACK
{
  uint8_t      id;
} AppCommandGetPanIdReq_t;

typedef struct PACK
{
  uint8_t      id;
  uint16_t     panId;
} AppCommandGetPanIdResp_t;

typedef struct PACK
{
  uint8_t      id;
  uint8_t      channel;
} AppCommandSetChannelReq_t;

typedef struct PACK
{
  uint8_t      id;
} AppCommandGetChannelReq_t;

typedef struct PACK
{
  uint8_t      id;
  uint8_t      channel;
} AppCommandGetChannelResp_t;

typedef struct PACK
{
  uint8_t      id;
  uint8_t      rxState;
} AppCommandSetRxStateReq_t;

typedef struct PACK
{
  uint8_t      id;
} AppCommandGetRxStateReq_t;

typedef struct PACK
{
  uint8_t      id;
  uint8_t      rxState;
} AppCommandGetRxStateResp_t;

typedef struct PACK
{
  uint8_t      id;
  uint8_t      txPower;
} AppCommandSetTxPowerReq_t;

typedef struct PACK
{
  uint8_t      id;
} AppCommandGetTxPowerReq_t;

typedef struct PACK
{
  uint8_t      id;
  uint8_t      txPower;
} AppCommandGetTxPowerResp_t;

typedef struct PACK
{
  uint8_t      id;
  uint8_t      key[16];
} AppCommandSetSecurityKeyReq_t;

typedef struct PACK
{
  uint8_t      id;
  uint8_t      state;
} AppCommandSetAckStateReq_t;

typedef struct PACK
{
  uint8_t      id;
} AppCommandGetAckStateReq_t;

typedef struct PACK
{
  uint8_t      id;
  uint8_t      state;
} AppCommandGetAckStateResp_t;

typedef struct PACK
{
  uint8_t      id;
  uint8_t      ledState;
} AppCommandSetLedStateReq_t;

#ifdef PER_APP
	typedef struct PACK
	{
	  uint8_t      id;
	} AppCommandStartTest_t;

	typedef struct PACK
	{
	  uint8_t      id;
	  uint8_t	   total;
	} AppCommandTestComplete_t;

	typedef struct PACK
	{
	  uint8_t      id;
	} AppCommandSendTestData_t;
#endif

/*****************************************************************************
*****************************************************************************/
extern AppIb_t appIb;
extern uint32_t appSleepReqInterval;
extern bool appSetDefaults;
extern bool appUpdateUart;

/*****************************************************************************
*****************************************************************************/
uint8_t *appCommandResponse(void);
uint8_t appCommandResponseSize(void);
AppStatus_t appCommandHandler(uint8_t *buf, uint8_t size);
void appUartSendCommand(uint8_t *buf, uint8_t size);

void appIbInit(bool defaults);
bool appIbPdsSave(void);

#endif // _SERIAL_H_

