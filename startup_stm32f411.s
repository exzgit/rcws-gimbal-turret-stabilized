.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb

.global _estack
.global Reset_Handler

_estack = 0x2002000   /* 128KB SRAM */

.section .isr_vector, "a", %progbits
.word _estack
.word Reset_Handler

.section .text.Reset_Handler
Reset_Handler:
  bl main
  b .
