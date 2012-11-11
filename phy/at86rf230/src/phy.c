/*
 * Copyright (c) 2011 - 2012, SimpleMesh AUTHORS
 * Eric Gnoske,
 * Colin O'Flynn,
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

#ifdef PHY_AT86RF230

#include <stdbool.h>
#include "phy.h"
#include "sysTaskManager.h"

/*****************************************************************************
*****************************************************************************/
// To support LED app
#include "led.h"

/*****************************************************************************
*****************************************************************************/
#define RSSI_BASE_VAL                  (-90)
#define CCA_REG_CHANNEL_MASK           (~0x1f)
#define TX_PWR_REG_TX_PWR_MASK         (~0x0f)

/*****************************************************************************
*****************************************************************************/
static void phyWriteRegister(uint8_t reg, uint8_t value);
static uint8_t phyReadRegister(uint8_t reg);
static void phyTrxSetState(uint8_t state);

/*****************************************************************************
*****************************************************************************/

volatile PHY_State_t phyState = PHY_STATE_INITIAL;
volatile uint8_t     phyTxStatus;
volatile int8_t      phyRxRssi;
static uint8_t       phyRxBuffer[128];

/*****************************************************************************
*****************************************************************************/
void PHY_Init(void)
{
  HAL_PhyReset();

  phyWriteRegister(TRX_STATE_REG, TRX_CMD_TRX_OFF);
  while (TRX_STATUS_TRX_OFF != (phyReadRegister(TRX_STATUS_REG) & TRX_STATUS_TRX_STATUS_MASK));

  phyWriteRegister(CSMA_SEED_0_REG, 0x12); // TODO: should be random
  phyWriteRegister(PHY_TX_PWR_REG, (1 << 7)/*TX_AUTO_CRC_ON*/ | 0x00/* +3 dBm */);

  phyWriteRegister(IRQ_MASK_REG, 0x00);
  phyReadRegister(IRQ_STATUS_REG);
  phyWriteRegister(IRQ_MASK_REG, TRX_END_MASK);

  phyIb.requests = PHY_IB_NONE;
  phyIb.rx = false;
  phyIb.txPower = 0;
  phyState = PHY_STATE_IDLE;
}

/*****************************************************************************
*****************************************************************************/
void PHY_DataReq(uint8_t *data, uint8_t size)
{
  phyTrxSetState(TRX_CMD_TX_ARET_ON);

  HAL_PhySpiSelect();
  HAL_PhySpiWriteByte(RF_CMD_FRAME_W);
  HAL_PhySpiWriteByte(size + 2/*crc*/);
  for (uint8_t i = 0; i < size; i++)
    HAL_PhySpiWriteByte(data[i]);
  HAL_PhySpiDeselect();

  phyWriteRegister(TRX_STATE_REG, TRX_CMD_TX_START);

  phyState = PHY_STATE_TX_WAIT_END;


}

/*****************************************************************************
*****************************************************************************/
void PHY_SetRxState(bool rx)
{
  phyIb.requests |= PHY_IB_RX_STATE;
  phyIb.rx = rx;
  SYS_TaskSet(PHY_TASK);
}

/*****************************************************************************
*****************************************************************************/
void PHY_SetTxPower(uint8_t txPower)
{
  phyIb.requests |= PHY_IB_TX_POWER;
  phyIb.txPower = txPower;
  SYS_TaskSet(PHY_TASK);
}

/*****************************************************************************
*****************************************************************************/
void PHY_SetChannel(uint8_t channel)
{
  phyIb.requests |= PHY_IB_CHANNEL;
  phyIb.channel = channel;
  SYS_TaskSet(PHY_TASK);
}

/*****************************************************************************
*****************************************************************************/
void PHY_SetPanId(uint16_t panId)
{
  phyIb.requests |= PHY_IB_PANID;
  phyIb.panId = panId;
  SYS_TaskSet(PHY_TASK);
}

/*****************************************************************************
*****************************************************************************/
void PHY_SetAddr(uint16_t addr)
{
  phyIb.requests |= PHY_IB_ADDR;
  phyIb.addr = addr;
  SYS_TaskSet(PHY_TASK);
}

/*****************************************************************************
*****************************************************************************/
bool PHY_Busy(void)
{
  return PHY_STATE_IDLE != phyState || PHY_IB_NONE != phyIb.requests;
}

/*****************************************************************************
*****************************************************************************/
void PHY_SleepReq(void)
{
  phyIb.requests |= PHY_IB_SLEEP;
  SYS_TaskSet(PHY_TASK);
}

/*****************************************************************************
*****************************************************************************/
void PHY_WakeupReq(void)
{
//  assert(PHY_STATE_SLEEP == phyState);

  phyIb.requests |= PHY_IB_WAKEUP;
  HAL_PhySlpTrClear();
  phyState = PHY_STATE_IDLE;
  SYS_TaskSet(PHY_TASK);
}

/*****************************************************************************
*****************************************************************************/
static void phyReturnToRxState(void)
{
  if (phyIb.rx)
    phyTrxSetState(TRX_CMD_RX_AACK_ON);
  else
    phyTrxSetState(TRX_CMD_TRX_OFF);
}

/*****************************************************************************
*****************************************************************************/
static void phyProcessRequests(void)
{
  if (PHY_IB_NONE == phyIb.requests)
  {
    phyReturnToRxState();
    return;
  }

  phyTrxSetState(TRX_CMD_TRX_OFF);

  if (phyIb.requests & PHY_IB_ADDR)
  {
    uint8_t *d = (uint8_t *)&phyIb.addr;
    phyWriteRegister(SHORT_ADDR_0_REG, d[0]);
    phyWriteRegister(SHORT_ADDR_1_REG, d[1]);
  }

  if (phyIb.requests & PHY_IB_PANID)
  {
    uint8_t *d = (uint8_t *)&phyIb.panId;
    phyWriteRegister(PAN_ID_0_REG, d[0]);
    phyWriteRegister(PAN_ID_1_REG, d[1]);
  }

  if (phyIb.requests & PHY_IB_CHANNEL)
  {
    uint8_t reg = phyReadRegister(PHY_CC_CCA_REG) & CCA_REG_CHANNEL_MASK;
    phyWriteRegister(PHY_CC_CCA_REG, reg | phyIb.channel);
  }

  if (phyIb.requests & PHY_IB_TX_POWER)
  {
    uint8_t reg = phyReadRegister(PHY_TX_PWR_REG) & TX_PWR_REG_TX_PWR_MASK;
    phyWriteRegister(PHY_TX_PWR_REG, reg | phyIb.txPower);
  }

  if (phyIb.requests & PHY_IB_SLEEP)
  {
    phyTrxSetState(TRX_CMD_TRX_OFF);
    HAL_PhySlpTrSet();
    phyState = PHY_STATE_SLEEP;
    PHY_SleepConf();
  }

  if (phyIb.requests & PHY_IB_WAKEUP)
  {
    PHY_WakeupConf();
  }

  phyIb.requests = PHY_IB_NONE;

  if (PHY_STATE_IDLE == phyState)
    phyReturnToRxState();
}

/*****************************************************************************
*****************************************************************************/
static void phyWriteRegister(uint8_t reg, uint8_t value)
{
  phyWriteRegisterInline(reg, value);
}

/*****************************************************************************
*****************************************************************************/
static uint8_t phyReadRegister(uint8_t reg)
{
  return phyReadRegisterInline(reg);
}

/*****************************************************************************
*****************************************************************************/
static void phyTrxSetState(uint8_t state)
{
  phyWriteRegister(TRX_STATE_REG, TRX_CMD_FORCE_TRX_OFF);
  phyWriteRegister(TRX_STATE_REG, state);
  while (state != (phyReadRegister(TRX_STATUS_REG) & TRX_STATUS_TRX_STATUS_MASK));
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
    {
      phyProcessRequests();
    } break;

    case PHY_STATE_SLEEP:
      break;

    case PHY_STATE_TX_WAIT_END:
      break;

    case PHY_STATE_TX_CONFIRM:
    {
      PHY_DataConf(phyTxStatus);

      while (TRX_CMD_PLL_ON != (phyReadRegister(TRX_STATUS_REG) & TRX_STATUS_TRX_STATUS_MASK));
      phyState = PHY_STATE_IDLE;
      SYS_TaskSet(PHY_TASK);
    } break;

    case PHY_STATE_RX_IND:
    {
      PHY_DataInd_t ind;
      uint8_t size;

      //ETG  A simple indicator that frames are being received...
      ledToggle();

      HAL_PhySpiSelect();
      HAL_PhySpiWriteByte(RF_CMD_FRAME_R);
      size = HAL_PhySpiWriteByte(0);
      for (uint8_t i = 0; i < size + 1/*lqi*/; i++)
        phyRxBuffer[i] = HAL_PhySpiWriteByte(0);
      HAL_PhySpiDeselect();
      ind.data = phyRxBuffer;
      ind.size = size - 2/*crc*/;
      ind.lqi  = phyRxBuffer[size];
      ind.rssi = phyRxRssi + RSSI_BASE_VAL;
      PHY_DataInd(&ind);

      while (TRX_CMD_PLL_ON != (phyReadRegister(TRX_STATUS_REG) & TRX_STATUS_TRX_STATUS_MASK));
      phyState = PHY_STATE_IDLE;
      SYS_TaskSet(PHY_TASK);
    } break;

    default:
      break;
  }
}

#endif // PHY_AT86RF230

