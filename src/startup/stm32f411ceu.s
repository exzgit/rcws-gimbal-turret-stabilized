/*
 * @file     : stm32f411ceu.s
 * @author   : exzdevs
 * @desc     : ...
 *
 */

.syntax     unified
.cpu        cortex-m4
.fpu        fpv4-sp-d16
.thumb

/* external symbols */
.extern     main

/* stack and heap configuration */
.equ        STACK_SIZE, 0x00000400
.equ        HEAP_SIZE,  0x00000200

.section    .stack, "aw", %progbits
.align      3
_stack_limit:
  .space    STACK_SIZE
_stack_top:

.section    .heap, "aw", %progbits
.align      3
_heap_start:
  .space    HEAP_SIZE
_heap_end:

/*
 * @title   : Vector Table
 * @desc    : ...
 *
 */

.section    .isr_vector, "a", %progbits
.align      2
.global     g_pfnVectors

g_pfnVectors:
  .word     _stack_top                        /* initial stack pointer */
  .word     Reset_Handler
  .word     NMI_Handler
  .word     HardFault_Handler
  .word     MemManage_Handler
  .word     BusFault_Handler
  .word     UsageFault_Handler
  .word     0
  .word     0
  .word     0
  .word     0
  .word     SVC_Handler
  .word     DebugMon_Handler
  .word     0
  .word     PendSV_Handler
  .word     SysTick_Handler

  /* STM32F411 interrupts */
  .word     WWDG_IRQHandler
  .word     PVD_IRQHandler
  .word     TAMP_STAMP_IRQHandler
  .word     RTC_WKUP_IRQHandler
  .word     FLASH_IRQHandler
  .word     RCC_IRQHandler
  .word     EXTI0_IRQHandler
  .word     EXTI1_IRQHandler
  .word     EXTI2_IRQHandler
  .word     EXTI3_IRQHandler
  .word     EXTI4_IRQHandler
  .word     DMA1_Stream0_IRQHandler
  .word     DMA1_Stream1_IRQHandler
  .word     DMA1_Stream2_IRQHandler
  .word     DMA1_Stream3_IRQHandler
  .word     DMA1_Stream4_IRQHandler
  .word     DMA1_Stream5_IRQHandler
  .word     DMA1_Stream6_IRQHandler
  .word     ADC_IRQHandler
  .word     0
  .word     0
  .word     0
  .word     0
  .word     EXTI9_5_IRQHandler
  .word     TIM1_BRK_TIM9_IRQHandler
  .word     TIM1_UP_TIM10_IRQHandler
  .word     TIM1_TRG_COM_TIM11_IRQHandler
  .word     TIM1_CC_IRQHandler
  .word     TIM2_IRQHandler
  .word     TIM3_IRQHandler
  .word     TIM4_IRQHandler
  .word     I2C1_EV_IRQHandler
  .word     I2C1_ER_IRQHandler
  .word     I2C2_EV_IRQHandler
  .word     I2C2_ER_IRQHandler
  .word     SPI1_IRQHandler
  .word     SPI2_IRQHandler
  .word     USART1_IRQHandler
  .word     USART2_IRQHandler
  .word     0
  .word     EXTI15_10_IRQHandler
  .word     RTC_Alarm_IRQHandler
  .word     OTG_FS_WKUP_IRQHandler
  .word     0
  .word     0
  .word     0
  .word     0
  .word     DMA1_Stream7_IRQHandler
  .word     0
  .word     SDIO_IRQHandler
  .word     TIM5_IRQHandler
  .word     SPI3_IRQHandler
  .word     0
  .word     0
  .word     0
  .word     0 
  .word     DMA2_Stream0_IRQHandler
  .word     DMA2_Stream1_IRQHandler
  .word     DMA2_Stream2_IRQHandler
  .word     DMA2_Stream3_IRQHandler
  .word     DMA2_Stream4_IRQHandler
  .word     0
  .word     0
  .word     0
  .word     0
  .word     0
  .word     OTG_FS_IRQHandler
  .word     DMA2_Stream5_IRQHandler
  .word     DMA2_Stream6_IRQHandler
  .word     DMA2_Stream7_IRQHandler
  .word     USART6_IRQHandler
  .word     I2C3_EV_IRQHandler
  .word     I2C3_ER_IRQHandler
  .word     FPU_IRQHandler
  .word     SPI4_IRQHandler


/*
 * @title   : Reset Handler
 * @desc    : ...
 *
 */

.section    .text.Reset_Handler
.align      2
.global     Reset_Handler
.type       Reset_Handler, %function

Reset_Handler:
  /* Copy .data from FLASH to RAM */
  ldr r0, =_sidata
  ldr r1, =_sdata
  ldr r2, =_edata
CopyDataLoop:
  cmp r1, r2
  bcc CopyData
  b   ZeroBssInit
CopyData:
  ldr r3, [r0], #4
  str r3, [r1], #4
  b   CopyDataLoop

ZeroBssInit:
  /* Zero .bss */
  ldr r0, =_sbss
  ldr r1, =_ebss
  movs r2, #0
ZeroBssLoop:
  cmp r0, r1
  bcc ZeroBss
  b   SystemInitCall
ZeroBss:
  str r2, [r0], #4
  b   ZeroBssLoop

SystemInitCall:
  bl main

Hang:
  b Hang

.size       Reset_Handler, .-Reset_Handler

/*
 * @title   : Default Handler
 * @desc    : ...
 *
 */

.section    .text.Default_Handler
.align      2
Default_Handler:
  b .

/* weak aliases */
.weak       NMI_Handler
.weak       HardFault_Handler
.weak       MemManage_Handler
.weak       BusFault_Handler
.weak       UsageFault_Handler
.weak       SVC_Handler
.weak       DebugMon_Handler
.weak       PendSV_Handler
.weak       SysTick_Handler

.weak       WWDG_IRQHandler
.weak       PVD_IRQHandler
.weak       TAMP_STAMP_IRQHandler
.weak       RTC_WKUP_IRQHandler
.weak       FLASH_IRQHandler
.weak       RCC_IRQHandler
.weak       EXTI0_IRQHandler
.weak       EXTI1_IRQHandler
.weak       EXTI2_IRQHandler
.weak       EXTI3_IRQHandler
.weak       EXTI4_IRQHandler
.weak       DMA1_Stream0_IRQHandler
.weak       DMA1_Stream1_IRQHandler
.weak       DMA1_Stream2_IRQHandler
.weak       DMA1_Stream3_IRQHandler
.weak       DMA1_Stream4_IRQHandler
.weak       DMA1_Stream5_IRQHandler
.weak       DMA1_Stream6_IRQHandler
.weak       ADC_IRQHandler
.weak       EXTI9_5_IRQHandler
.weak       TIM1_BRK_TIM9_IRQHandler
.weak       TIM1_UP_TIM10_IRQHandler
.weak       TIM1_TRG_COM_TIM11_IRQHandler
.weak       TIM1_CC_IRQHandler
.weak       TIM2_IRQHandler
.weak       TIM3_IRQHandler
.weak       TIM4_IRQHandler
.weak       I2C1_EV_IRQHandler
.weak       I2C1_ER_IRQHandler
.weak       I2C2_EV_IRQHandler
.weak       I2C2_ER_IRQHandler
.weak       SPI1_IRQHandler
.weak       SPI2_IRQHandler
.weak       USART1_IRQHandler
.weak       USART2_IRQHandler
.weak       EXTI15_10_IRQHandler
.weak       RTC_Alarm_IRQHandler
.weak       OTG_FS_WKUP_IRQHandler
.weak       DMA1_Stream7_IRQHandler
.weak       SDIO_IRQHandler
.weak       TIM5_IRQHandler
.weak       SPI3_IRQHandler
.weak       DMA2_Stream0_IRQHandler
.weak       DMA2_Stream1_IRQHandler
.weak       DMA2_Stream2_IRQHandler
.weak       DMA2_Stream3_IRQHandler
.weak       DMA2_Stream4_IRQHandler
.weak       OTG_FS_IRQHandler
.weak       DMA2_Stream5_IRQHandler
.weak       DMA2_Stream6_IRQHandler
.weak       DMA2_Stream7_IRQHandler
.weak       USART6_IRQHandler
.weak       I2C3_EV_IRQHandler
.weak       I2C3_ER_IRQHandler
.weak       FPU_IRQHandler
.weak       SPI4_IRQHandler

NMI_Handler               = Default_Handler
HardFault_Handler         = Default_Handler
MemManage_Handler         = Default_Handler
BusFault_Handler          = Default_Handler
UsageFault_Handler        = Default_Handler
SVC_Handler               = Default_Handler
DebugMon_Handler          = Default_Handler
PendSV_Handler            = Default_Handler
SysTick_Handler           = Default_Handler

WWDG_IRQHandler           = Default_Handler
PVD_IRQHandler            = Default_Handler
TAMP_STAMP_IRQHandler     = Default_Handler
RTC_WKUP_IRQHandler       = Default_Handler
FLASH_IRQHandler          = Default_Handler
RCC_IRQHandler            = Default_Handler
EXTI0_IRQHandler          = Default_Handler
EXTI1_IRQHandler          = Default_Handler
EXTI2_IRQHandler          = Default_Handler
EXTI3_IRQHandler          = Default_Handler
EXTI4_IRQHandler          = Default_Handler
DMA1_Stream0_IRQHandler   = Default_Handler
DMA1_Stream1_IRQHandler   = Default_Handler
DMA1_Stream2_IRQHandler   = Default_Handler
DMA1_Stream3_IRQHandler   = Default_Handler
DMA1_Stream4_IRQHandler   = Default_Handler
DMA1_Stream5_IRQHandler   = Default_Handler
DMA1_Stream6_IRQHandler   = Default_Handler
ADC_IRQHandler            = Default_Handler
EXTI9_5_IRQHandler        = Default_Handler
TIM1_BRK_TIM9_IRQHandler  = Default_Handler
TIM1_UP_TIM10_IRQHandler  = Default_Handler
TIM1_TRG_COM_TIM11_IRQHandler = Default_Handler
TIM1_CC_IRQHandler        = Default_Handler
TIM2_IRQHandler           = Default_Handler
TIM3_IRQHandler           = Default_Handler
TIM4_IRQHandler           = Default_Handler
I2C1_EV_IRQHandler        = Default_Handler
I2C1_ER_IRQHandler        = Default_Handler
I2C2_EV_IRQHandler        = Default_Handler
I2C2_ER_IRQHandler        = Default_Handler
SPI1_IRQHandler           = Default_Handler
SPI2_IRQHandler           = Default_Handler
USART1_IRQHandler         = Default_Handler
USART2_IRQHandler         = Default_Handler
EXTI15_10_IRQHandler      = Default_Handler
RTC_Alarm_IRQHandler      = Default_Handler
OTG_FS_WKUP_IRQHandler    = Default_Handler
DMA1_Stream7_IRQHandler   = Default_Handler
SDIO_IRQHandler           = Default_Handler
TIM5_IRQHandler           = Default_Handler
SPI3_IRQHandler           = Default_Handler
DMA2_Stream0_IRQHandler   = Default_Handler
DMA2_Stream1_IRQHandler   = Default_Handler
DMA2_Stream2_IRQHandler   = Default_Handler
DMA2_Stream3_IRQHandler   = Default_Handler
DMA2_Stream4_IRQHandler   = Default_Handler
OTG_FS_IRQHandler         = Default_Handler
DMA2_Stream5_IRQHandler   = Default_Handler
DMA2_Stream6_IRQHandler   = Default_Handler
DMA2_Stream7_IRQHandler   = Default_Handler
USART6_IRQHandler         = Default_Handler
I2C3_EV_IRQHandler        = Default_Handler
I2C3_ER_IRQHandler        = Default_Handler
FPU_IRQHandler            = Default_Handler
SPI4_IRQHandler           = Default_Handler
