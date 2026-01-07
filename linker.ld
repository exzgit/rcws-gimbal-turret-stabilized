/* -------------------------------------------------------------------------- */
/*  Memory layout                                                             */
/* -------------------------------------------------------------------------- */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
  RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
}

/* -------------------------------------------------------------------------- */
/*  Entry point                                                               */
/* -------------------------------------------------------------------------- */
ENTRY(Reset_Handler)

/* -------------------------------------------------------------------------- */
/*  Sections                                                                  */
/* -------------------------------------------------------------------------- */
SECTIONS
{
  /* Vector table and program code in FLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } > FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text*)
    *(.rodata*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } > FLASH

  /* Initialized data copied to RAM */
  _sidata = LOADADDR(.data);

  .data : AT(_sidata)
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > RAM

  /* Uninitialized data */
  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > RAM

  /* Heap and stack (defined in startup file) */
  .heap (NOLOAD) :
  {
    . = ALIGN(8);
    _heap_start = .;
    *(.heap*)
    . = ALIGN(8);
    _heap_end = .;
  } > RAM

  .stack (NOLOAD) :
  {
    . = ALIGN(8);
    *(.stack*)
    . = ALIGN(8);
  } > RAM

  /* Discard useless junk */
  /DISCARD/ :
  {
    *(.note*)
    *(.comment*)
  }
}
