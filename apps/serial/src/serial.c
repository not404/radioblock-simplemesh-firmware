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
#include "serial.h"

/*****************************************************************************
*****************************************************************************/
#define APP_UART_START_BYTE        0xab
#define APP_UART_TIMER_INTERVAL    100
#define APP_UART_TX_BUFFER_SIZE    350
#define APP_UART_RX_BUFFER_SIZE    150
#ifndef PER_APP
	#define APP_UART_CMD_BUFFER_SIZE   150
#endif

/*****************************************************************************
*****************************************************************************/
#ifndef PER_APP
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
#endif

typedef struct PACK AppMessage_t
{
  uint8_t     type;
} AppMessage_t;

/*****************************************************************************
*****************************************************************************/
static void appUartAck(AppStatus_t status);
static void appUartInit(void);

/*****************************************************************************
*****************************************************************************/
uint32_t appSleepReqInterval;
bool appSetDefaults = false;
bool appUpdateUart = false;

#ifdef PER_APP
	AppState_t appState = APP_STATE_INITIAL;
#else
	static AppState_t appState = APP_STATE_INITIAL;
#endif

static HAL_Uart_t appUart;
static uint8_t appUartTxBuffer[APP_UART_TX_BUFFER_SIZE];
static uint8_t appUartRxBuffer[APP_UART_RX_BUFFER_SIZE];

#ifdef PER_APP
	AppUartState_t appUartState = APP_UART_STATE_IDLE;
	uint8_t appUartCmdBuffer[APP_UART_CMD_BUFFER_SIZE];
	uint8_t appUartCmdSize;
#else
	static AppUartState_t appUartState = APP_UART_STATE_IDLE;
	static uint8_t appUartCmdBuffer[APP_UART_CMD_BUFFER_SIZE];
	static uint8_t appUartCmdSize;
#endif
static SYS_Timer_t appUartTimer;


static uint8_t appUartCmdPtr;
static uint16_t appUartCmdCrc;
static AppStatus_t appUartStatus;

/*****************************************************************************
*****************************************************************************/
static uint16_t appCrcCcitt(uint8_t *buf, uint8_t size)
{
  uint16_t crc = 0x1234;

  for (uint8_t i = 0; i < size; i++)
  {
    uint8_t data = buf[i];

    data ^= crc & 0xff;
    data ^= data << 4;
    crc = (((uint16_t)data << 8) | ((crc >> 8) & 0xff)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3);
  }

  return crc;
}

/*****************************************************************************
*****************************************************************************/
static void appUartStateMachine(uint8_t byte)
{
  switch (appUartState)
  {
    case APP_UART_STATE_IDLE:
    {
      if (APP_UART_START_BYTE == byte)
        appUartState = APP_UART_STATE_READ_SIZE;
    } break;

    case APP_UART_STATE_READ_SIZE:
    {
      appUartCmdSize = byte;
      appUartCmdPtr = 0;

      if (appUartCmdSize <= sizeof(appUartCmdBuffer))
      {
        appUartState = APP_UART_STATE_READ_DATA;
      }
      else
      {
        appUartState = APP_UART_STATE_ERROR;
        appUartStatus = APP_STATUS_INVALID_SIZE;
      }
    } break;

    case APP_UART_STATE_READ_DATA:
    {
      appUartCmdBuffer[appUartCmdPtr++] = byte;
      if (appUartCmdSize == appUartCmdPtr)
        appUartState = APP_UART_STATE_READ_CRC_1;
    } break;

    case APP_UART_STATE_READ_CRC_1:
    {
      appUartCmdCrc = byte;
      appUartState = APP_UART_STATE_READ_CRC_2;
    } break;

    case APP_UART_STATE_READ_CRC_2:
    {
     appUartCmdCrc |= ((uint16_t)byte << 8);
      if (appUartCmdCrc == appCrcCcitt(appUartCmdBuffer, appUartCmdSize))
      {
        appUartState = APP_UART_STATE_OK;
        appState = APP_STATE_COMMAND_RECEIVED;
        SYS_PortSet(APP_PORT);
      }
      else
      {
        appUartState = APP_UART_STATE_ERROR;
        appUartStatus = APP_STATUS_INVALID_CRC;
      }
    } break;

    case APP_UART_STATE_OK:
      break;

    case APP_UART_STATE_ERROR:
      break;

    default:
      break;
  }

  SYS_TimerStop(&appUartTimer);
  SYS_TimerStart(&appUartTimer);
}

/*****************************************************************************
*****************************************************************************/
static void appUartTimerHandler(SYS_Timer_t *timer)
{
  if (APP_UART_STATE_IDLE == appUartState ||
      APP_UART_STATE_OK == appUartState ||
      APP_UART_STATE_STOP == appUartState)
    return;

  if (APP_UART_STATE_ERROR == appUartState)
    appUartAck(appUartStatus);
  else
    appUartAck(APP_STATUS_TIMEOUT);

  appUartState = APP_UART_STATE_IDLE;

  (void)timer;
}

/*****************************************************************************
*****************************************************************************/
static void appUartRxCallback(uint16_t bytes)
{
// If a character is received over the UART then turn off the PER_APP code.
// "ota_enabled" is a flag for this.
#ifdef PER_APP
//	ota_enabled = 0;
#endif

  uint8_t byte;

  for (uint16_t i = 0; i < bytes; i++)
  {
    byte = HAL_UartReadByte();
    appUartStateMachine(byte);
  }
}

/*****************************************************************************
*****************************************************************************/
static void appUartTxCallback(void)
{
  if (APP_STATE_RESET_REQ == appState)
  {
    HAL_WarmReset();
  }
  else if (APP_STATE_SLEEP_REQ == appState)
  {
    appState = APP_STATE_PREPARE_TO_SLEEP;
    SYS_PortSet(APP_PORT);
  }
  else if (APP_STATE_DEFAULTS_REQ == appState)
  {
    appState = APP_STATE_INITIAL;
    SYS_PortSet(APP_PORT);
  }
  else if (APP_STATE_UART_REQ == appState)
  {
    appUartInit();
    appState = APP_STATE_WAIT_COMMAND;
    appUartState = APP_UART_STATE_IDLE;
    appUpdateUart = false;
  }
}

/*****************************************************************************
*****************************************************************************/
static void appUartAck(AppStatus_t status)
{
  AppCommandAck_t ack;

  ack.id = APP_COMMAND_ACK;
  ack.status = status;
  appUartSendCommand((uint8_t *)&ack, sizeof(ack));
}

/*****************************************************************************
*****************************************************************************/
void appUartSendCommand(uint8_t *buf, uint8_t size)
{
//	if(ota_enabled) // Don't use the UART in the PER_APP
	{
	  uint16_t crc;

	  HAL_UartWriteByte(APP_UART_START_BYTE);
	  HAL_UartWriteByte(size);

	  for (uint8_t i = 0; i < size; i++)
	    HAL_UartWriteByte(buf[i]);

	  crc = appCrcCcitt(buf, size);
	  HAL_UartWriteByte(crc & 0xff);
	  HAL_UartWriteByte((crc >> 8) & 0xff);
	}
}

/*****************************************************************************
*****************************************************************************/
static bool appDataInd(NWK_DataInd_t *ind)
{
  AppCommandDataInd_t cmd;
  uint8_t size;

  if (0 == appIb.ackState)
    return false;

  size = 1/*start*/ + 1/*size*/ + sizeof(AppCommandDataIndHeader_t) + ind->size + 2/*crc*/;

  if (size > HAL_UartGetFreeSize())
    return false;

#ifdef LED_APP
  // ETG
  // Interrogate the payload. If byte [0] = 'O', turn the LED off,
  // if byte [0] = 'F' turn the LED on.
  if (cmd.payload[0] == 'O')
  	ledOn();
  if (cmd.payload[0] == 'F')
  	ledOff();
#endif // LED_APP

#ifdef PER_APP
  // Only the RXN collects data.
#ifdef TEST_MODE
  if(0x0000 == appIb.addr);
#else
  if(0x2222 == appIb.addr);
#endif
  {
	  // This records the "histograms" of received LQI & RSSI for up to 250 frames.
	  // The histograms/arrays are passed to the PC node and post analyzed.
	  lqi_buf[cmd.lqi]++;
	  // The number was converted to a -dB value when received, it is converted
	  // here to enable our array mechanism. That is why the * -1 and then adding
	  // one. In post analysis on the PC, it will need to be turned into a negative
	  // number.
	  rssi_buf[(cmd.rssi*(-1))+1]++;
  }

  /*
		Now we must extract the payload portion of the received OTA data frame
  	which contains an encapsulated "command". The supported commands and
  	payload formats are:
  	1. Set Channel Request
		cmd.payload[0] = Command ID (0x29)
  		cmd.payload[1] = Address LSB
  		cmd.payload[2] = Address MSB
  	2. Set Receiver State Request
  		cmd.payload[0] = Command ID (0x2C)
  		cmd.payload[1] = Receiver State (0 = off, 1 = on)
  	New OTA commands:
  	3. Start Test
  		cmd.payload[0] = Command ID (0xFD)
  	4. Test Complete
  		cmd.payload[0] = Command ID (0xFE)
  	5. Send Data
  		cmd.payload[0] = Command ID (0xFF)
  	6. Test Indication

  	Therefore, the normal data indication is 'hijacked' below...
  	We actually need to make it appear as if the command came in over the
  	UART so we hijack pieces of the "appUartStateMachine" function as
  	well.

  	The command will be processed in the "appTaskHandler" function...
   */

  appUartState = APP_UART_STATE_OK;
  appState = APP_STATE_COMMAND_RECEIVED;
  SYS_PortSet(APP_PORT);

  if(ind->data[0] == APP_COMMAND_SET_CHANNEL_REQ)
  {
	  memcpy(appUartCmdBuffer, ind->data, sizeof(AppCommandSetChannelReq_t));
  	  appUartCmdSize = 2;
  }
  else if(ind->data[0] == APP_COMMAND_SET_RX_STATE_REQ)
  {
	  memcpy(appUartCmdBuffer, ind->data, sizeof(AppCommandSetRxStateReq_t));
  	  appUartCmdSize = 2;
  }
  else if(ind->data[0] == APP_COMMAND_START_TEST_REQ)
  {
	  memcpy(appUartCmdBuffer, ind->data, sizeof(AppCommandStartTest_t));
  	  appUartCmdSize = 1;
  }

  else if(ind->data[0] == APP_COMMAND_TEST_COMPLETE)
  {
	  memcpy(appUartCmdBuffer, ind->data, sizeof(AppCommandTestComplete_t));
	  appUartCmdSize = 1;
  }

  else if(ind->data[0] == APP_COMMAND_SEND_DATA_REQ)
  {
	  memcpy(appUartCmdBuffer, ind->data, sizeof(AppCommandSendTestData_t));
	  appUartCmdSize = 1;
  }
  else // If this is not an OTA command then do the normal thing.
  {
  	cmd.id = APP_COMMAND_DATA_IND;
	cmd.src = ind->src;
	cmd.options = (ind->options.ack << 0) | (ind->options.security << 1);
	cmd.lqi = ind->lqi;
	cmd.rssi = ind->rssi;
	memcpy(cmd.payload, ind->data, ind->size);

	appUartSendCommand((uint8_t *)&cmd, sizeof(AppCommandDataIndHeader_t) + ind->size);

	SYS_TimerStop(&appUartTimer);
	SYS_TimerStart(&appUartTimer);

	return true;
	}
  SYS_TimerStop(&appUartTimer);
  SYS_TimerStart(&appUartTimer);
  return false;

#else
  cmd.id = APP_COMMAND_DATA_IND;
  cmd.src = ind->src;
  cmd.options = (ind->options.ack << 0) | (ind->options.security << 1);
  cmd.lqi = ind->lqi;
  cmd.rssi = ind->rssi;
  memcpy(cmd.payload, ind->data, ind->size);

  appUartSendCommand((uint8_t *)&cmd, sizeof(AppCommandDataIndHeader_t) + ind->size);
#endif // PER_APP

  return true;
}

/*****************************************************************************
*****************************************************************************/
static void appUartInit(void)
{
  HAL_UartClose();

  appUart.bits         = appIb.bits;
  appUart.parity       = appIb.parity;
  appUart.stop         = appIb.stop;
  appUart.baudrate     = appIb.baudrate;
  appUart.txBuffer     = appUartTxBuffer;
  appUart.txBufferSize = sizeof(appUartTxBuffer);
  appUart.rxBuffer     = appUartRxBuffer;
  appUart.rxBufferSize = sizeof(appUartRxBuffer);
  appUart.rxCallback   = appUartRxCallback;
  appUart.txCallback   = appUartTxCallback;
  HAL_UartInit(&appUart);

  appUartState = APP_UART_STATE_IDLE;
}

/*****************************************************************************
*****************************************************************************/
static void appInit(void)
{
  appIbInit(appSetDefaults);
  appUartInit();

  ledInit();
  if (appIb.ledState)
    ledOn();
  else
    ledOff();

  NWK_SetAddr(appIb.addr);
  NWK_SetPanId(appIb.panId);
  NWK_SetSecurityKey(appIb.key);
  PHY_SetChannel(appIb.channel);
  PHY_SetRxState(appIb.rxState);
  PHY_SetTxPower(appIb.txPower);

  appUartTimer.interval = APP_UART_TIMER_INTERVAL;
  appUartTimer.mode = SYS_TIMER_PERIODIC_MODE;
  appUartTimer.handler = appUartTimerHandler;
  SYS_TimerRestart(&appUartTimer);

  appState = APP_STATE_WAIT_COMMAND;
  appSetDefaults = false;


#ifdef PER_APP
  ota_enabled = 1;
  memset(rssi_buf, 0, 256);
  memset(lqi_buf, 0, 256);
  per_count = 0;
#else
  ota_enabled = 0;
#endif

}

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

    case APP_STATE_WAIT_COMMAND:
      break;

    case APP_STATE_COMMAND_RECEIVED:
    {
      AppStatus_t status;

      status = appCommandHandler(appUartCmdBuffer, appUartCmdSize);
      appUartAck(status);

      appState = APP_STATE_WAIT_COMMAND;
      appUartState = APP_UART_STATE_IDLE;

      if (APP_STATUS_SUCESS == status)
      {
        if (APP_COMMAND_RESET_REQ == appUartCmdBuffer[0])
        {
          appState = APP_STATE_RESET_REQ;
          appUartState = APP_UART_STATE_STOP;
        }
        else if (APP_COMMAND_SLEEP_REQ == appUartCmdBuffer[0])
        {
          appState = APP_STATE_SLEEP_REQ;
          appUartState = APP_UART_STATE_STOP;
        }
        else if (appSetDefaults)
        {
          appState = APP_STATE_DEFAULTS_REQ;
          appUartState = APP_UART_STATE_STOP;
        }
        else if (appUpdateUart)
        {
          appState = APP_STATE_UART_REQ;
          appUartState = APP_UART_STATE_STOP;
        }
        else if (appCommandResponseSize())
        {
          uint8_t *resp = appCommandResponse();
          uint8_t size = appCommandResponseSize();

          appUartSendCommand(resp, size);
        }
      }
    } break;

    case APP_STATE_PREPARE_TO_SLEEP:
    {
      NWK_SleepReq();
      appState = APP_STATE_WAIT_SLEEP_CONF;
    } break;

    case APP_STATE_WAIT_SLEEP_CONF:
      break;

    case APP_STATE_SLEEP:
    {
      PHY_SetRxState(false);
      HAL_Sleep(appSleepReqInterval);
      appState = APP_STATE_WAKEUP;
      SYS_PortSet(APP_PORT);
    } break;

    case APP_STATE_WAKEUP:
    {
      NWK_WakeupReq();
      appState = APP_STATE_WAIT_WAKEUP_CONF;
    } break;

    case APP_STATE_WAIT_WAKEUP_CONF:
      break;

    case APP_STATE_READY:
    {
      AppCommandWakeupInd_t ind;

      ind.id = APP_COMMAND_WAKEUP_IND;
      appUartSendCommand((uint8_t *)&ind, sizeof(ind));

      appState = APP_STATE_WAIT_COMMAND;
      appUartState = APP_UART_STATE_IDLE;
    } break;

    default:
      break;
  }
}

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
  PHY_SetRxState(appIb.rxState);
  appState = APP_STATE_READY;
  SYS_PortSet(APP_PORT);
}

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

