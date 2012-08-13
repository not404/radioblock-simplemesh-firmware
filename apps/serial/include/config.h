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

#ifndef _CONFIG_H_
#define _CONFIG_H_

/*****************************************************************************
*****************************************************************************/
#define APP_PORT                       1

#define APP_DEFAULT_ADDR               2
#define APP_DEFAULT_PANID              0
#define APP_DEFAULT_CHANNEL            0x0f
#define APP_DEFAULT_RX_STATE           true
#define APP_DEFAULT_TX_POWER           +3
#define APP_DEFAULT_ACK_STATE          1
#define APP_DEFAULT_SECURITY_KEY       "SimpleMesh_12345"

#define APP_DEFAULT_UART_BITS          HAL_UART_BITS_8
#define APP_DEFAULT_UART_PARITY        HAL_UART_PARITY_NONE
#define APP_DEFAULT_UART_STOP          HAL_UART_STOP_BITS_1
#define APP_DEFAULT_UART_BAUDRATE      HAL_UART_BAUDRATE_115200

#define APP_DEFAULT_LED_STATE          0


#define NWK_DUPLICATE_REJECTION_TABLE_SIZE      20
#define NWK_DUPLICATE_REJECTION_TIMER_INTERVAL  1000   // 1 second
#define NWK_DUPLICATE_REJECTION_TTL             10     // 10 seconds
#define NWK_ROUTE_TABLE_SIZE                    10
#define NWK_ROUTE_DEFAULT_SCORE                 3
#define NWK_ACK_WAIT_TIME                       1000   // 1 second
#define NWK_MAX_PORTS_AMOUNT                    16

#ifdef PER_APP
	#define SYS_MEM_POOL_SIZE						500
#else
	#define SYS_MEM_POOL_SIZE                       2000
#endif // PER_APP

#define HAL_UART_ENABLE

#endif // _CONFIG_H_

