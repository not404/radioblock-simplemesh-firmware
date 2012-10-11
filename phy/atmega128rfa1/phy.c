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

#ifdef PHY_ATMEGA128RFA1

#include <avr/io.h>
#include <avr/interrupt.h>
#include "phy.h"
#include "sysTaskManager.h"
#include "assert.h"

/*****************************************************************************
*****************************************************************************/
#define IRQ_STATUS_CLEAR_VALUE         0xff
#define RSSI_BASE_VAL                  (-90)
#define TRX_FRAME_BUFFER(index)        (*(volatile uint8_t *)(0x180 + (index)))

/*****************************************************************************
*****************************************************************************/
enum
{
  TRX_CMD_NOP           = 0,
  TRX_CMD_TX_START      = 2,
  TRX_CMD_FORCE_TRX_OFF = 3,
  TRX_CMD_FORCE_PLL_ON  = 4,
  TRX_CMD_RX_ON         = 6,
  TRX_CMD_TRX_OFF       = 8,
  TRX_CMD_PLL_ON        = 9,
  TRX_CMD_RX_AACK_ON    = 22,
  TRX_CMD_TX_ARET_ON    = 25,
};

enum
{
  TRX_STATUS_P_ON                         = 0,
  TRX_STATUS_BUSY_RX                      = 1,
  TRX_STATUS_BUSY_TX                      = 2,
  TRX_STATUS_RX_ON                        = 6,
  TRX_STATUS_TRX_OFF                      = 8,
  TRX_STATUS_PLL_ON                       = 9,
  TRX_STATUS_SLEEP                        = 15,
  TRX_STATUS_BUSY_RX_AACK                 = 17,
  TRX_STATUS_BUSY_TX_ARET                 = 18,
  TRX_STATUS_RX_AACK_ON                   = 22,
  TRX_STATUS_TX_ARET_ON                   = 25,
  TRX_STATUS_RX_ON_NOCLK                  = 28,
  TRX_STATUS_RX_AACK_ON_NOCLK             = 29,
  TRX_STATUS_BUSY_RX_AACK_NOCLK           = 30,
  TRX_STATUS_STATE_TRANSITION_IN_PROGRESS = 31,
};

typedef enum PHY_State_t
{
  PHY_STATE_INITIAL,
  PHY_STATE_IDLE,
  PHY_STATE_TX_WAIT_END,
  PHY_STATE_TX_CONFIRM,
  PHY_STATE_RX_IND,
} PHY_State_t;

/*****************************************************************************
*****************************************************************************/
static inline void phyTrxSetState(uint8_t state);
void PHY_DataConf(uint8_t status);
void PHY_DataInd(PHY_DataInd_t *ind);

/*****************************************************************************
*****************************************************************************/
static volatile PHY_State_t phyState = PHY_STATE_INITIAL;
static volatile uint8_t     phyTxStatus;
static volatile int8_t      phyRxRssi;
static volatile uint8_t     phyRxSize;
static uint8_t              phyRxBuffer[128];

/*****************************************************************************
*****************************************************************************/
void PHY_Init(void)
{
  TRXPR_struct.trxrst = 1;

  phyTrxSetState(TRX_CMD_TRX_OFF);

  CSMA_SEED_1_struct.aack_set_pd = 1;
  CSMA_SEED_1_struct.aack_dis_ack = 0;

  IRQ_STATUS = IRQ_STATUS_CLEAR_VALUE;
  IRQ_MASK_struct.rx_end_en = 1;
  IRQ_MASK_struct.tx_end_en = 1;

  TRX_CTRL_2_struct.rx_safe_mode = 1;

  CSMA_SEED_0 = 0x12; // TODO: should be random

  phyState = PHY_STATE_IDLE;
  phyTrxSetState(TRX_CMD_PLL_ON);
}

/*****************************************************************************
*****************************************************************************/
void PHY_SetRxState(bool rx)
{
  if (rx)
    phyTrxSetState(TRX_CMD_RX_AACK_ON);
  else
    phyTrxSetState(TRX_CMD_PLL_ON);
}

/*****************************************************************************
*****************************************************************************/
void PHY_SetChannel(uint8_t channel)
{
  PHY_CC_CCA_struct.channel = channel;
}

/*****************************************************************************
*****************************************************************************/
void PHY_SetPanId(uint16_t panId)
{
  uint8_t *d = (uint8_t *)&panId;

  PAN_ID_0 = d[0];
  PAN_ID_1 = d[1];
}

/*****************************************************************************
*****************************************************************************/
void PHY_SetShortAddr(uint16_t addr)
{
  uint8_t *d = (uint8_t *)&addr;

  SHORT_ADDR_0 = d[0];
  SHORT_ADDR_1 = d[1];
}

/*****************************************************************************
*****************************************************************************/
bool PHY_IsBusy(void)
{
  return PHY_STATE_IDLE != phyState;
}

/*****************************************************************************
*****************************************************************************/
void PHY_DataReq(uint8_t *data, uint8_t size)
{
  assert(PHY_STATE_IDLE == phyState);

  phyTrxSetState(TRX_CMD_TX_ARET_ON);

  TRX_FRAME_BUFFER(0) = size + 2/*crc*/;
  for (uint8_t i = 0; i < size; i++)
    TRX_FRAME_BUFFER(i+1) = data[i];

  TRX_STATE = CMD_TX_START;

  phyState = PHY_STATE_TX_WAIT_END;
}

/*****************************************************************************
*****************************************************************************/
ISR(TRX24_TX_END_vect)
{
  if (TRX_STATUS_TX_ARET_ON == TRX_STATUS_struct.trx_status)
  {
    assert(PHY_STATE_TX_WAIT_END == phyState); // assert_inline

    //TRX_STATE = TRX_CMD_PLL_ON; // Don't wait for this to complete
    phyTrxSetState(TRX_CMD_PLL_ON);

    phyState = PHY_STATE_TX_CONFIRM;
    phyTxStatus = TRX_STATE_struct.trac_status;
    SYS_TaskSetInline(PHY_TASK);
  }
  else
  {
    // Auto ACK transmission completed
  }
}

/*****************************************************************************
*****************************************************************************/
ISR(TRX24_RX_END_vect)
{
  assert(PHY_STATE_IDLE == phyState); // assert_inline

  TRX_STATE = TRX_CMD_PLL_ON; // Don't wait for this to complete
  phyRxRssi = (int8_t)PHY_ED_LEVEL;
  phyRxSize = TST_RX_LENGTH;
  phyState = PHY_STATE_RX_IND;
  SYS_TaskSetInline(PHY_TASK);
}

/*****************************************************************************
*****************************************************************************/
static inline void phyTrxSetState(uint8_t state)
{
  TRX_STATE = CMD_FORCE_TRX_OFF;
  TRX_STATE = state;
  while (state != TRX_STATUS_struct.trx_status);
}

/*****************************************************************************
*****************************************************************************/
void phyTaskHandler(void)
{
  switch (phyState)
  {
    case PHY_STATE_INITIAL:
      break;

    case PHY_STATE_IDLE:
      break;

    case PHY_STATE_TX_WAIT_END:
      break;

    case PHY_STATE_TX_CONFIRM:
    {
      PHY_DataConf(phyTxStatus);

      while (TRX_CMD_PLL_ON != TRX_STATUS_struct.trx_status);
      phyTrxSetState(TRX_CMD_RX_AACK_ON);
      phyState = PHY_STATE_IDLE;
    } break;

    case PHY_STATE_RX_IND:
    {
      PHY_DataInd_t ind;

      for (uint8_t i = 0; i < phyRxSize + 1/*lqi*/; i++)
        phyRxBuffer[i] = TRX_FRAME_BUFFER(i);

      ind.data = phyRxBuffer;
      ind.size = phyRxSize - 2/*crc*/;
      ind.lqi  = phyRxBuffer[phyRxSize];
      ind.rssi = phyRxRssi + RSSI_BASE_VAL;
      PHY_DataInd(&ind);

      while (TRX_CMD_PLL_ON != TRX_STATUS_struct.trx_status);
      phyTrxSetState(TRX_CMD_RX_AACK_ON);
      phyState = PHY_STATE_IDLE;
    } break;

    default:
      break;
  }
}

#endif // PHY_ATMEGA128RFA1

