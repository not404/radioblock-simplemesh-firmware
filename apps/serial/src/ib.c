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
#include "sysTypes.h"
#include "halUart.h"
#include "serial.h"

/*****************************************************************************
*****************************************************************************/
#define VALID_SETTINGS_MARKER  0x706d6953L

#define IAP_ADDRESS            0x1fff1ff1L

#define IAP_PREPARE_SECTOR     50
#define IAP_COPY_RAM_TO_FLASH  51
#define IAP_ERASE_SECTORS      52

#define FLASH_PAGE_SIZE        256
#define PDS_FLASH_ADDR         0x7000

/*****************************************************************************
*****************************************************************************/
void (*iap)(uint32_t *, uint32_t *) = (void *)IAP_ADDRESS;
AppIb_t appIb;


/*****************************************************************************
*****************************************************************************/
void appIbSetDefault(void)
{
  appIb.valid    = VALID_SETTINGS_MARKER;

  appIb.addr     = APP_DEFAULT_ADDR;
  appIb.panId    = APP_DEFAULT_PANID;
  appIb.channel  = APP_DEFAULT_CHANNEL;
  appIb.rxState  = APP_DEFAULT_RX_STATE;
  appIb.txPower  = APP_DEFAULT_TX_POWER;
  appIb.ackState = APP_DEFAULT_ACK_STATE;
  memcpy(appIb.key, (uint8_t *)APP_DEFAULT_SECURITY_KEY, 16);

  appIb.bits     = APP_DEFAULT_UART_BITS;
  appIb.parity   = APP_DEFAULT_UART_PARITY;
  appIb.stop     = APP_DEFAULT_UART_STOP;
  appIb.baudrate = APP_DEFAULT_UART_BAUDRATE;

  appIb.ledState = APP_DEFAULT_LED_STATE;
}

/*****************************************************************************
*****************************************************************************/
bool appIbPdsSave(void)
{
  uint32_t command[5];
  uint32_t result[4];
  uint32_t buffer[FLASH_PAGE_SIZE / sizeof(uint32_t)];

  memset((uint8_t *)buffer, 0xff, FLASH_PAGE_SIZE);
  memcpy((uint8_t *)buffer, (uint8_t *)&appIb, sizeof(appIb));

  command[0] = IAP_PREPARE_SECTOR;
  command[1] = PDS_FLASH_ADDR >> 12;
  command[2] = PDS_FLASH_ADDR >> 12;
  ATOMIC_SECTION_ENTER
    iap(command, result);
  ATOMIC_SECTION_LEAVE
  if (result[0])
    return false;

  command[0] = IAP_ERASE_SECTORS;
  command[1] = PDS_FLASH_ADDR >> 12;
  command[2] = PDS_FLASH_ADDR >> 12;
  ATOMIC_SECTION_ENTER
    iap(command, result);
  ATOMIC_SECTION_LEAVE
  if (result[0])
    return false;

  command[0] = IAP_PREPARE_SECTOR;
  command[1] = PDS_FLASH_ADDR >> 12;
  command[2] = PDS_FLASH_ADDR >> 12;
  ATOMIC_SECTION_ENTER
    iap(command, result);
  ATOMIC_SECTION_LEAVE
  if (result[0])
    return false;

  command[0] = IAP_COPY_RAM_TO_FLASH;
  command[1] = PDS_FLASH_ADDR;
  command[2] = (uint32_t)buffer;
  command[3] = FLASH_PAGE_SIZE;
  command[4] = F_CPU / 1000;
  ATOMIC_SECTION_ENTER
    iap(command, result);
  ATOMIC_SECTION_LEAVE
  if (result[0])
    return false;

  return true;
}

/*****************************************************************************
*****************************************************************************/
void appIbInit(bool defaults)
{
  memcpy((uint8_t *)&appIb, (uint8_t *)PDS_FLASH_ADDR, sizeof(appIb));
  if (defaults || VALID_SETTINGS_MARKER != appIb.valid)
    appIbSetDefault();
}

