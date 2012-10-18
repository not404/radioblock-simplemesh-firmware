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

#ifndef _PHY_H_
#define _PHY_H_

#include <stdint.h>
#include <stdbool.h>
#include "halPhy.h"
#include "sysTaskManager.h"

/*****************************************************************************
*****************************************************************************/
enum
{
  TRAC_STATUS_SUCCESS                = 0,
  TRAC_STATUS_SUCCESS_DATA_PENDING   = 1,
  TRAC_STATUS_SUCCESS_WAIT_FOR_ACK   = 2,
  TRAC_STATUS_CHANNEL_ACCESS_FAILURE = 3,
  TRAC_STATUS_NO_ACK                 = 5,
  TRAC_STATUS_INVALID                = 7,
};

enum
{
  TRX_STATUS_REG   = 0x01,
  TRX_STATE_REG    = 0x02,
  TRX_CTRL_0_REG   = 0x03,
  PHY_TX_PWR_REG   = 0x05,
  PHY_RSSI_REG     = 0x06,
  PHY_ED_LEVEL_REG = 0x07,
  PHY_CC_CCA_REG   = 0x08,
  CCA_THRESH_REG   = 0x09,
  TRX_CTRL_2_REG   = 0x0c,
  IRQ_MASK_REG     = 0x0e,
  IRQ_STATUS_REG   = 0x0f,
  VREG_CTRL_REG    = 0x10,
  BATMON_REG       = 0x11,
  XOSC_CTRL_REG    = 0x12,
  XAH_CTRL_1_REG   = 0x17, // ETG - To turn on promiscuous mode for sniffing.
  PLL_CF_REG       = 0x1a,
  PLL_DCU_REG      = 0x1b,
  PART_NUM_REG     = 0x1c,
  VERSION_NUM_REG  = 0x1d,
  MAN_ID_0_REG     = 0x1e,
  MAN_ID_1_REG     = 0x1f,
  SHORT_ADDR_0_REG = 0x20,
  SHORT_ADDR_1_REG = 0x21,
  PAN_ID_0_REG     = 0x22,
  PAN_ID_1_REG     = 0x23,
  IEEE_ADDR_0_REG  = 0x24,
  IEEE_ADDR_1_REG  = 0x25,
  IEEE_ADDR_2_REG  = 0x26,
  IEEE_ADDR_3_REG  = 0x27,
  IEEE_ADDR_4_REG  = 0x28,
  IEEE_ADDR_5_REG  = 0x29,
  IEEE_ADDR_6_REG  = 0x2a,
  IEEE_ADDR_7_REG  = 0x2b,
  XAH_CTRL_0_REG   = 0x2c,
  CSMA_SEED_0_REG  = 0x2d,
  CSMA_SEED_1_REG  = 0x2e,
};

enum
{
  RF_CMD_REG_W     = ((1<<7) | (1<<6)),
  RF_CMD_REG_R     = ((1<<7) | (0<<6)),
  RF_CMD_FRAME_W   = ((0<<7) | (1<<6) | (1<<5)),
  RF_CMD_FRAME_R   = ((0<<7) | (0<<6) | (1<<5)),
  RF_CMD_SRAM_W    = ((0<<7) | (1<<6) | (0<<5)),
  RF_CMD_SRAM_R    = ((0<<7) | (0<<6) | (0<<5)),
};

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
  TRX_STATUS_TRX_STATUS_MASK              = 0x1f,
};

enum
{
  PLL_LOCK_MASK   = (1 << 0),
  PLL_UNLOCK_MASK = (1 << 1),
  RX_START_MASK   = (1 << 2),
  TRX_END_MASK    = (1 << 3),
  TRX_UR_MASK     = (1 << 6),
  BAT_LOW_MASK    = (1 << 7),
};

typedef enum PHY_State_t
{
  PHY_STATE_INITIAL,
  PHY_STATE_IDLE,
  PHY_STATE_SLEEP,
  PHY_STATE_TX_WAIT_END,
  PHY_STATE_TX_CONFIRM,
  PHY_STATE_RX_IND,
} PHY_State_t;

typedef enum PHY_TxPower_t
{
  PHY_TX_POWER_3_0_DBM      = 0x00,
  PHY_TX_POWER_2_8_DBM      = 0x01,
  PHY_TX_POWER_2_3_DBM      = 0x02,
  PHY_TX_POWER_1_8_DBM      = 0x03,
  PHY_TX_POWER_1_3_DBM      = 0x04,
  PHY_TX_POWER_0_7_DBM      = 0x05,
  PHY_TX_POWER_0_DBM        = 0x06,
  PHY_TX_POWER_M_1_DBM      = 0x07,
  PHY_TX_POWER_M_2_DBM      = 0x08,
  PHY_TX_POWER_M_3_DBM      = 0x09,
  PHY_TX_POWER_M_4_DBM      = 0x0a,
  PHY_TX_POWER_M_5_DBM      = 0x0b,
  PHY_TX_POWER_M_7_DBM      = 0x0c,
  PHY_TX_POWER_M_9_DBM      = 0x0d,
  PHY_TX_POWER_M_12_DBM     = 0x0e,
  PHY_TX_POWER_M_17_DBM     = 0x0f,
} PHY_TxPower_t;

typedef struct PHY_DataInd_t
{
  uint8_t    *data;
  uint8_t    size;
  uint8_t    lqi;
  int8_t     rssi;
} PHY_DataInd_t;

#if SNIFFER
	// Global flag to indicate frame received to sniffer app.
	uint8_t sniffFlag;

	enum
	{
	  PHY_IB_NONE      = 0,
	  PHY_IB_ADDR      = (1 << 0),
	  PHY_IB_PANID     = (1 << 1),
	  PHY_IB_CHANNEL   = (1 << 2),
	  PHY_IB_RX_STATE  = (1 << 3),
	  PHY_IB_TX_POWER  = (1 << 4),
	  PHY_IB_SLEEP     = (1 << 5),
	  PHY_IB_WAKEUP    = (1 << 6),
	};

	typedef struct PhyIb_t
	{
	  uint8_t     requests;

	  uint16_t    addr;
	  uint16_t    panId;
	  uint8_t     channel;
	  bool        rx;
	  uint8_t     txPower;
	} PhyIb_t;
#endif

/*****************************************************************************
*****************************************************************************/
extern volatile PHY_State_t phyState;
extern volatile uint8_t     phyTxStatus;
extern volatile int8_t      phyRxRssi;

/*****************************************************************************
*****************************************************************************/
// ETG
#if SNIFFER
	void phyTrxSetState(uint8_t state);
	void sendSnifferResults(void);
#endif

void PHY_Init(void);
void PHY_SetRxState(bool rx);
void PHY_SetTxPower(uint8_t txPower);
void PHY_SetChannel(uint8_t channel);
void PHY_SetPanId(uint16_t panId);
void PHY_SetAddr(uint16_t addr);
bool PHY_Busy(void);
void PHY_DataReq(uint8_t *data, uint8_t size);
void PHY_DataConf(uint8_t status);
void PHY_DataInd(PHY_DataInd_t *ind);
void PHY_SleepReq(void);
void PHY_WakeupReq(void);
void PHY_SleepConf(void);
void PHY_WakeupConf(void);

/*****************************************************************************
*****************************************************************************/
INLINE void phyWriteRegisterInline(uint8_t reg, uint8_t value)
{
  HAL_PhySpiSelect();
  HAL_PhySpiWriteByteInline(RF_CMD_REG_W | reg);
  HAL_PhySpiWriteByteInline(value);
  HAL_PhySpiDeselect();
}

/*****************************************************************************
*****************************************************************************/
INLINE uint8_t phyReadRegisterInline(uint8_t reg)
{
  uint8_t value;

  HAL_PhySpiSelect();
  HAL_PhySpiWriteByteInline(RF_CMD_REG_R | reg);
  value = HAL_PhySpiWriteByteInline(0);
  HAL_PhySpiDeselect();

  return value;
}

/*****************************************************************************
*****************************************************************************/

INLINE void phyInterruptHandler(void)
{
  uint8_t irq;

 // DEBUG
  sniffFlag = phyReadRegisterInline(TRX_STATE_REG);

  irq = phyReadRegisterInline(IRQ_STATUS_REG);
  if (0 == (irq & TRX_END_MASK))
    return;

  sniffFlag = phyReadRegisterInline(TRX_STATE_REG);

#if SNIFFER
//    phyWriteRegisterInline(TRX_STATE_REG, TRX_CMD_RX_AACK_ON);
//    phyRxRssi = (int8_t)phyReadRegisterInline(PHY_ED_LEVEL_REG);
    sniffFlag = 1;
    return;
#endif

  if (PHY_STATE_TX_WAIT_END == phyState)
  {
    phyWriteRegisterInline(TRX_STATE_REG, TRX_CMD_PLL_ON);
    phyTxStatus = (phyReadRegisterInline(TRX_STATE_REG) >> 5) & 0x07;
    phyState = PHY_STATE_TX_CONFIRM;
    SYS_TaskSetInline(PHY_TASK);
  }

  else if (PHY_STATE_IDLE == phyState)
  {
    phyWriteRegisterInline(TRX_STATE_REG, TRX_CMD_PLL_ON);
    phyRxRssi = (int8_t)phyReadRegisterInline(PHY_ED_LEVEL_REG);
    phyState = PHY_STATE_RX_IND;
    SYS_TaskSetInline(PHY_TASK);


  }
}

#endif // _PHY_H_

