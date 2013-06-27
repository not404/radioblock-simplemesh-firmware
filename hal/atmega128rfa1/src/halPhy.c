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
 *   3) Neither the name of the FIP AUTHORS nor the names of its contributors
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

#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
//#include "hal.h"
#include "phy.h"
#include "halPhy.h"
#include "sysTaskManager.h"

#include <avr/io.h>

/*****************************************************************************
*****************************************************************************/
inline static void halDelay(uint8_t us)
{
  for (uint8_t i = 0; i < us; i++) // TODO: do better than this
    asm("nop");
}

/*****************************************************************************
*****************************************************************************/
void HAL_PhyReset(void)
{
  halPhyRstClr();
  halDelay(10);
  halPhyRstSet();
}

/*****************************************************************************
*****************************************************************************/
void HAL_PhyInit(void)
{
  halPhySlpTrClr();

  halPhyRstSet();
  
  IRQ_MASK = ((1 << 3) | (1 << 6)); // setup TRX24_RX_END IRQ and TRX24_TX_END
}

/*****************************************************************************
*****************************************************************************/
ISR(TRX24_RX_END_vect)
{
  // TRX_STATE = TRX_CMD_PLL_ON; // Don't wait for this to complete
  *(uint8_t*)(TRX_REGISTER_BASEADDR + TRX_STATE_REG) = TRX_CMD_PLL_ON;
  phyRxRssi =  *(int8_t*)(TRX_REGISTER_BASEADDR + PHY_RSSI);
  phyState = PHY_STATE_RX_IND;
  SYS_TaskSetInline(PHY_TASK);
}

/*****************************************************************************
*****************************************************************************/
ISR(TRX24_TX_END_vect)
{
  if (PHY_STATE_TX_WAIT_END == phyState)
  {
    //TRX_STATE = TRX_CMD_PLL_ON; // Don't wait for this to complete
    *(uint8_t*)(TRX_REGISTER_BASEADDR + TRX_STATE_REG) = TRX_CMD_PLL_ON;

    phyState = PHY_STATE_TX_CONFIRM;
    phyTxStatus = (*(int8_t*)(TRX_REGISTER_BASEADDR + TRX_STATE_REG) >> 5) & 0x07;
    SYS_TaskSetInline(PHY_TASK);
  }
  else
  {
    // Auto ACK transmission completed
  }
}



