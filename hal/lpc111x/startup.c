#define WEAK __attribute__ ((weak))
#define ALIAS(f) __attribute__ ((weak, alias (#f)))
#define IRQ_HANDLER __attribute__ ((section(".irq_handlers")))

void ResetISR(void);
WEAK void NMI_Handler(void);
WEAK void HardFault_Handler(void);
WEAK void SVCall_Handler(void);
WEAK void PendSV_Handler(void);
WEAK void SysTick_Handler(void);
WEAK void IntDefaultHandler(void);

void CAN_IRQHandler(void) ALIAS(IntDefaultHandler);
void SSP1_IRQHandler(void) ALIAS(IntDefaultHandler);
void I2C_IRQHandler(void) ALIAS(IntDefaultHandler);
void TIMER16_0_IRQHandler(void) ALIAS(IntDefaultHandler);
void TIMER16_1_IRQHandler(void) ALIAS(IntDefaultHandler);
void TIMER32_0_IRQHandler(void) ALIAS(IntDefaultHandler);
void TIMER32_1_IRQHandler(void) ALIAS(IntDefaultHandler);
void SSP0_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART_IRQHandler(void) ALIAS(IntDefaultHandler);
void ADC_IRQHandler(void) ALIAS(IntDefaultHandler);
void WDT_IRQHandler(void) ALIAS(IntDefaultHandler);
void BOD_IRQHandler(void) ALIAS(IntDefaultHandler);
void PIOINT3_IRQHandler(void) ALIAS(IntDefaultHandler);
void PIOINT2_IRQHandler(void) ALIAS(IntDefaultHandler);
void PIOINT1_IRQHandler(void) ALIAS(IntDefaultHandler);
void PIOINT0_IRQHandler(void) ALIAS(IntDefaultHandler);
void WAKEUP_IRQHandler(void) ALIAS(IntDefaultHandler);

extern int main(void);

extern void _vStackTop(void);

__attribute__ ((section(".vectors")))
void (* const vectors[])(void) =
{
  &_vStackTop,		    				// The initial stack pointer
  ResetISR,                               // The reset handler
  NMI_Handler,                            // The NMI handler
  HardFault_Handler,                      // The hard fault handler
  0,                      				// Reserved
  0,                      				// Reserved
  0,                      				// Reserved
  0,                                      // Reserved
  0,                                      // Reserved
  0,                                      // Reserved
  0,                                      // Reserved
  SVCall_Handler,                      	// SVCall handler
  0,                      				// Reserved
  0,                                      // Reserved
  PendSV_Handler,                      	// The PendSV handler
  SysTick_Handler,                      	// The SysTick handler

  WAKEUP_IRQHandler,                      // PIO0_0  Wakeup
  WAKEUP_IRQHandler,                      // PIO0_1  Wakeup
  WAKEUP_IRQHandler,                      // PIO0_2  Wakeup
  WAKEUP_IRQHandler,                      // PIO0_3  Wakeup
  WAKEUP_IRQHandler,                      // PIO0_4  Wakeup
  WAKEUP_IRQHandler,                      // PIO0_5  Wakeup
  WAKEUP_IRQHandler,                      // PIO0_6  Wakeup
  WAKEUP_IRQHandler,                      // PIO0_7  Wakeup
  WAKEUP_IRQHandler,                      // PIO0_8  Wakeup
  WAKEUP_IRQHandler,                      // PIO0_9  Wakeup
  WAKEUP_IRQHandler,                      // PIO0_10 Wakeup
  WAKEUP_IRQHandler,                      // PIO0_11 Wakeup
  WAKEUP_IRQHandler,                      // PIO1_0  Wakeup
  CAN_IRQHandler,			// C_CAN Interrupt
  SSP1_IRQHandler, 			// SPI/SSP1 Interrupt
  I2C_IRQHandler,                      	// I2C0
  TIMER16_0_IRQHandler,                   // CT16B0 (16-bit Timer 0)
  TIMER16_1_IRQHandler,                   // CT16B1 (16-bit Timer 1)
  TIMER32_0_IRQHandler,                   // CT32B0 (32-bit Timer 0)
  TIMER32_1_IRQHandler,                   // CT32B1 (32-bit Timer 1)
  SSP0_IRQHandler,                      	// SPI/SSP0 Interrupt
  UART_IRQHandler,                      	// UART0
  0, 				                     	// Reserved
  0,                      				// Reserved
  ADC_IRQHandler,                      	// ADC   (A/D Converter)
  WDT_IRQHandler,                      	// WDT   (Watchdog Timer)
  BOD_IRQHandler,                      	// BOD   (Brownout Detect)
  0,                      				// Reserved
  PIOINT3_IRQHandler,                     // PIO INT3
  PIOINT2_IRQHandler,                     // PIO INT2
  PIOINT1_IRQHandler,                     // PIO INT1
  PIOINT0_IRQHandler,                     // PIO INT0
};




extern unsigned int _etext;
extern unsigned int _data;
extern unsigned int _edata;
extern unsigned int _bss;
extern unsigned int _ebss;


IRQ_HANDLER void data_init(unsigned int romstart, unsigned int start, unsigned int len)
{
	unsigned int *pulDest = (unsigned int*) start;
	unsigned int *pulSrc = (unsigned int*) romstart;
	unsigned int loop;
	for (loop = 0; loop < len; loop = loop + 4)
		*pulDest++ = *pulSrc++;
}

IRQ_HANDLER void bss_init(unsigned int start, unsigned int len)
{
	unsigned int *pulDest = (unsigned int*) start;
	unsigned int loop;
	for (loop = 0; loop < len; loop = loop + 4)
		*pulDest++ = 0;
}

IRQ_HANDLER void ResetISR(void)
{
  unsigned int * LoadAddr, *ExeAddr, *EndAddr, SectionLen;

  LoadAddr = &_etext;
  ExeAddr = &_data;
  EndAddr = &_edata;
  SectionLen = (void*)EndAddr - (void*)ExeAddr;
  data_init((unsigned int)LoadAddr, (unsigned int)ExeAddr, SectionLen);

  ExeAddr = &_bss;
  EndAddr = &_ebss;
  SectionLen = (void*)EndAddr - (void*)ExeAddr;
  bss_init((unsigned int)ExeAddr, SectionLen);

  main();
  while (1);


#if 0

// These are defined and created by the linker, locating them in memory
extern unsigned char _etext;
extern unsigned char _data;
extern unsigned char _edata;
extern unsigned char _bss;
extern unsigned char _ebss;

// Prototype the required startup functions
extern void main(void);

// The entry point of the application, prepare segments,
// initialize the cpu and execute main()
void boot_entry(void)
{
  register unsigned char *src, *dst;

  // Get physical data address and copy it to sram
  src = &_etext;
  dst = &_data;
  while(dst < &_edata) {
    *dst++ = *src++;
  }

  // Clear the bss segment
  dst = &_bss;
  while(dst < &_ebss) {
    *dst++ = 0;
  }

  // Execute the code at the program entry point
  main();

  // Do nothing when returned from main, just keep looping
  while(1);
}
#endif

}

#include <LPC11xx.h>
#include <core_cm0.h>

IRQ_HANDLER void NMI_Handler(void)
{
  while (1)
  {
    for (int i = 0; i < 30000; i++) asm("nop");
    LPC_GPIO0->MASKED_ACCESS[(1 << 7)] = (0 << 7);
    for (int i = 0; i < 30000; i++) asm("nop");
    LPC_GPIO0->MASKED_ACCESS[(1 << 7)] = (1 << 7);
  }
  while (1);
}

IRQ_HANDLER void HardFault_Handler(void)
{
  while (1)
  {
    for (int i = 0; i < 30000; i++) asm("nop");
    LPC_GPIO0->MASKED_ACCESS[(1 << 7)] = (0 << 7);
    for (int i = 0; i < 30000; i++) asm("nop");
    LPC_GPIO0->MASKED_ACCESS[(1 << 7)] = (1 << 7);
  }
  while(1);
}

IRQ_HANDLER void SVCall_Handler(void)
{
  while (1)
  {
    for (int i = 0; i < 30000; i++) asm("nop");
    LPC_GPIO0->MASKED_ACCESS[(1 << 7)] = (0 << 7);
    for (int i = 0; i < 30000; i++) asm("nop");
    LPC_GPIO0->MASKED_ACCESS[(1 << 7)] = (1 << 7);
  }
  while(1);
}

IRQ_HANDLER void PendSV_Handler(void)
{
  while (1)
  {
    for (int i = 0; i < 30000; i++) asm("nop");
    LPC_GPIO0->MASKED_ACCESS[(1 << 7)] = (0 << 7);
    for (int i = 0; i < 30000; i++) asm("nop");
    LPC_GPIO0->MASKED_ACCESS[(1 << 7)] = (1 << 7);
  }
  while(1);
}

IRQ_HANDLER void SysTick_Handler(void)
{
  while (1)
  {
    for (int i = 0; i < 30000; i++) asm("nop");
    LPC_GPIO0->MASKED_ACCESS[(1 << 7)] = (0 << 7);
    for (int i = 0; i < 30000; i++) asm("nop");
    LPC_GPIO0->MASKED_ACCESS[(1 << 7)] = (1 << 7);
  }
  while(1);
}

IRQ_HANDLER void IntDefaultHandler(void)
{
  while (1)
  {
    for (int i = 0; i < 30000; i++) asm("nop");
    LPC_GPIO0->MASKED_ACCESS[(1 << 7)] = (0 << 7);
    for (int i = 0; i < 30000; i++) asm("nop");
    LPC_GPIO0->MASKED_ACCESS[(1 << 7)] = (1 << 7);
  }
  while(1);
}

