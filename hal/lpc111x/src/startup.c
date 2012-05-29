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

#include "sysTypes.h"

/*****************************************************************************
*****************************************************************************/
#define ALIAS(f) __attribute__ ((weak, alias (#f)))
#define IRQ_HANDLER __attribute__ ((section(".irq_handlers")))

/*****************************************************************************
*****************************************************************************/
void HAL_ResetHandler(void);
WEAK void HAL_NmiHandler(void);
WEAK void HAL_HardFaultHandler(void);
WEAK void HAL_SvCallHandler(void);
WEAK void HAL_PendSvHandler(void);
WEAK void HAL_SysTickHandler(void);
WEAK void HAL_DummyIrqHandler(void);

void HAL_WAKEUP0_0_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);
void HAL_WAKEUP0_1_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);
void HAL_WAKEUP0_2_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);
void HAL_WAKEUP0_3_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);
void HAL_WAKEUP0_4_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);
void HAL_WAKEUP0_5_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);
void HAL_WAKEUP0_6_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);
void HAL_WAKEUP0_7_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);
void HAL_WAKEUP0_8_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);
void HAL_WAKEUP0_9_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);
void HAL_WAKEUP0_10_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);
void HAL_WAKEUP0_11_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);
void HAL_WAKEUP1_0_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);
void HAL_CAN_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);
void HAL_SSP1_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);
void HAL_I2C_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);
void HAL_TIMER16_0_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);
void HAL_TIMER16_1_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);
void HAL_TIMER32_0_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);
void HAL_TIMER32_1_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);
void HAL_SSP0_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);
void HAL_UART_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);
void HAL_ADC_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);
void HAL_WDT_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);
void HAL_BOD_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);
void HAL_PIOINT3_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);
void HAL_PIOINT2_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);
void HAL_PIOINT1_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);
void HAL_PIOINT0_IrqHandler(void) ALIAS(HAL_DummyIrqHandler);

extern int main(void);

extern void _stack_top(void);
extern unsigned int _etext;
extern unsigned int _data;
extern unsigned int _edata;
extern unsigned int _bss;
extern unsigned int _ebss;

/*****************************************************************************
*****************************************************************************/
__attribute__ ((section(".vectors")))
void (* const vectors[])(void) =
{
  &_stack_top,                   // Initial Stack Pointer
  HAL_ResetHandler,              // Reset
  HAL_NmiHandler,                // NMI
  HAL_HardFaultHandler,          // Hard Fault
  0,                             // Reserved
  0,                             // Reserved
  0,                             // Reserved
  0,                             // Reserved
  0,                             // Reserved
  0,                             // Reserved
  0,                             // Reserved
  HAL_SvCallHandler,             // SVCall
  0,                             // Reserved
  0,                             // Reserved
  HAL_PendSvHandler,             // PendSV
  HAL_SysTickHandler,            // SysTick

  HAL_WAKEUP0_0_IrqHandler,      // PIO0_0 Wakeup
  HAL_WAKEUP0_1_IrqHandler,      // PIO0_1 Wakeup
  HAL_WAKEUP0_2_IrqHandler,      // PIO0_2 Wakeup
  HAL_WAKEUP0_3_IrqHandler,      // PIO0_3 Wakeup
  HAL_WAKEUP0_4_IrqHandler,      // PIO0_4 Wakeup
  HAL_WAKEUP0_5_IrqHandler,      // PIO0_5 Wakeup
  HAL_WAKEUP0_6_IrqHandler,      // PIO0_6 Wakeup
  HAL_WAKEUP0_7_IrqHandler,      // PIO0_7 Wakeup
  HAL_WAKEUP0_8_IrqHandler,      // PIO0_8 Wakeup
  HAL_WAKEUP0_9_IrqHandler,      // PIO0_9 Wakeup
  HAL_WAKEUP0_10_IrqHandler,     // PIO0_10 Wakeup
  HAL_WAKEUP0_11_IrqHandler,     // PIO0_11 Wakeup
  HAL_WAKEUP1_0_IrqHandler,      // PIO1_0 Wakeup
  HAL_CAN_IrqHandler,            // C_CAN Interrupt
  HAL_SSP1_IrqHandler,           // SPI/SSP1 Interrupt
  HAL_I2C_IrqHandler,            // I2C0
  HAL_TIMER16_0_IrqHandler,      // CT16B0 (16-bit Timer 0)
  HAL_TIMER16_1_IrqHandler,      // CT16B1 (16-bit Timer 1)
  HAL_TIMER32_0_IrqHandler,      // CT32B0 (32-bit Timer 0)
  HAL_TIMER32_1_IrqHandler,      // CT32B1 (32-bit Timer 1)
  HAL_SSP0_IrqHandler,           // SPI/SSP0 Interrupt
  HAL_UART_IrqHandler,           // UART0
  0,                             // Reserved
  0,                             // Reserved
  HAL_ADC_IrqHandler,            // ADC (A/D Converter)
  HAL_WDT_IrqHandler,            // WDT (Watchdog Timer)
  HAL_BOD_IrqHandler,            // BOD (Brownout Detect)
  0,                             // Reserved
  HAL_PIOINT3_IrqHandler,        // PIO INT3
  HAL_PIOINT2_IrqHandler,        // PIO INT2
  HAL_PIOINT1_IrqHandler,        // PIO INT1
  HAL_PIOINT0_IrqHandler,        // PIO INT0
};

/*****************************************************************************
*****************************************************************************/
IRQ_HANDLER void HAL_ResetHandler(void)
{
  unsigned int *src, *dst;

  src = &_etext;
  dst = &_data;
  while (dst < &_edata)
    *dst++ = *src++;

  dst = &_bss;
  while (dst < &_ebss)
    *dst++ = 0;

  main();
  while (1);
}

/*****************************************************************************
*****************************************************************************/
IRQ_HANDLER void HAL_NmiHandler(void)
{
  while (1);
}

/*****************************************************************************
*****************************************************************************/
IRQ_HANDLER void HAL_HardFaultHandler(void)
{
  while(1);
}

/*****************************************************************************
*****************************************************************************/
IRQ_HANDLER void HAL_SvCallHandler(void)
{
  while(1);
}

/*****************************************************************************
*****************************************************************************/
IRQ_HANDLER void HAL_PendSvHandler(void)
{
  while(1);
}

/*****************************************************************************
*****************************************************************************/
IRQ_HANDLER void HAL_SysTickHandler(void)
{
  while(1);
}

/*****************************************************************************
*****************************************************************************/
IRQ_HANDLER void HAL_DummyIrqHandler(void)
{
  while(1);
}

