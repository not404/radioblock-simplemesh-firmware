/**/
MEMORY
{
  flash (rx) : ORIGIN = 0x0, LENGTH = 0x8000 /* 32k */
  ram (rwx)  : ORIGIN = 0x10000000, LENGTH = 0x2000 /* 8k */
}

__top_flash = 0x0 + 0x8000;
__top_ram = 0x10000000 + 0x2000;

ENTRY(ResetISR)

SECTIONS
{
  .text : ALIGN(4)
  {
    FILL(0xff)
    KEEP(*(.vectors))
    . = ALIGN(4);
    *(.irq_handlers*)
    *(.text*)
    *(.rodata)
    *(.rodata.*)
    . = ALIGN(4);
  } > flash

  _etext = .;

  .uninit_RESERVED : ALIGN(4)
  {
    KEEP(*(.bss.$RESERVED*))
  } > ram

  .data : ALIGN(4)
  {
    FILL(0xff)
    _data = .;
    *(vtable)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > ram AT > flash

  .bss : ALIGN(4)
  {
    _bss = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    PROVIDE(end = .);
  } > ram

  PROVIDE(_pvHeapStart = .);
  PROVIDE(_vStackTop = __top_ram - 0);
}

