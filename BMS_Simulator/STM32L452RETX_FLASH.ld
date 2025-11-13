/* STM32L452RETx linker script - minimal freestanding GCC version
 * FLASH 512 KB @ 0x08000000
 * SRAM1 128 KB @ 0x20000000
 * SRAM2  32 KB @ 0x10000000
 */

ENTRY(Reset_Handler)

/* Memory regions */
MEMORY
{
  FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 512K
  RAM   (xrw)     : ORIGIN = 0x20000000, LENGTH = 128K
  RAM2  (xrw)     : ORIGIN = 0x10000000, LENGTH = 32K
}

/* Provide symbols the startup code / HAL expect */
_estack = ORIGIN(RAM) + LENGTH(RAM);   /* initial MSP stack pointer */
_Min_Heap_Size = 0x200;               /* adjust later if you want malloc */
_Min_Stack_Size = 0x400;              /* 1 KB stack safety guard */

/* Sections */
SECTIONS
{
  /* Vector table and code */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))   /* startup_stm32l4xx.s places the vector table here */
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text*)              /* code */
    *(.rodata*)            /* read-only data (const) */
    *(.glue_7) *(.glue_7t)
    *(.eh_frame*)

    . = ALIGN(4);
    KEEP(*(.init))
    KEEP(*(.fini))
  } >FLASH

  /* ARM exception unwinding stuff (optional, but avoids link warnings) */
  .ARM.extab :
  {
    *(.ARM.extab*)
  } >FLASH

  .ARM.exidx :
  {
    *(.ARM.exidx*)
  } >FLASH

  /* These symbols help the C runtime copy / zero sections */
  _etext = .;        /* end of code+rodata in FLASH */

  /* Initialized data goes in RAM but its load image is in FLASH */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;      /* start of .data in RAM */
    *(.data*)
    . = ALIGN(4);
    _edata = .;      /* end of .data in RAM */
  } >RAM AT >FLASH

  /* Uninitialized data (zeroed at startup) */
  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _sbss = .;       /* start of .bss in RAM */
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;       /* end of .bss in RAM */
  } >RAM

  /* Optional: put ._user_heap_stack in RAM so linker can check size */
  ._user_heap_stack (NOLOAD):
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /* Discard stuff we don't want in final image (C++ EH tables, etc.) */
  /DISCARD/ :
  {
    *(.note*)
    *(.comment*)
  }
}

/* Symbols HAL / startup code usually pulls in */
PROVIDE(__stack = _estack);

/* For newlib syscalls if needed */
PROVIDE(_Heap_Begin = end);
PROVIDE(_Heap_Limit = ORIGIN(RAM) + LENGTH(RAM));
