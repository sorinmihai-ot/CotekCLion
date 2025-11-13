    .syntax unified
    .cpu cortex-m4
    .fpu softvfp
    .thumb

    /* ----------------------------------------------------------------------
     *  Extern symbols provided by the linker script or by C code
     * ---------------------------------------------------------------------- */
    .extern  SystemInit
    .extern  __libc_init_array
    .extern  main

    .global  g_pfnVectors
    .global  Default_Handler

    /* These come from the linker script. */
    .extern _estack
    .extern _etext
    .extern _sdata
    .extern _edata
    .extern _sbss
    .extern _ebss

    /* ----------------------------------------------------------------------
     *  Reset Handler
     * ---------------------------------------------------------------------- */
    .section .text.Reset_Handler,"ax",%progbits
    .align  2
    .weak   Reset_Handler
    .type   Reset_Handler, %function
    .thumb_func
Reset_Handler:
    /* 1. Init stack pointer */
    ldr   r0, =_estack
    mov   sp, r0

    /* 2. Copy initialized data from FLASH (_etext) to SRAM (_sdata.._edata) */
    ldr   r1, =_sdata    /* r1 = dst (RAM)   */
    ldr   r2, =_edata    /* r2 = end of data */
    ldr   r3, =_etext    /* r3 = src (FLASH) */
1:
    cmp   r1, r2
    bcc   2f
    b     3f
2:
    ldr   r0, [r3], #4
    str   r0, [r1], #4
    b     1b

3:
    /* 3. Zero-fill the .bss section (_sbss.._ebss) */
    ldr   r1, =_sbss
    ldr   r2, =_ebss
    movs  r0, #0
4:
    cmp   r1, r2
    bcc   5f
    b     6f
5:
    str   r0, [r1], #4
    b     4b

6:
    /* 4. Low-level clock / HAL init */
    bl    SystemInit

    /* 5. Run static constructors (C++ / newlib init) */
    bl    __libc_init_array

    /* 6. Call main() */
    bl    main

    /* 7. If main ever returns, just loop */
Infinite_Loop:
    b     Infinite_Loop

    .size  Reset_Handler, .-Reset_Handler

    /* ----------------------------------------------------------------------
     *  Default interrupt handler
     * ---------------------------------------------------------------------- */
    .section .text.Default_Handler,"ax",%progbits
    .align  2
    .weak   Default_Handler
    .type   Default_Handler, %function
    .thumb_func
Default_Handler:
    b     Default_Handler
    .size Default_Handler, .-Default_Handler

    /* ----------------------------------------------------------------------
     *  Vector table
     * ---------------------------------------------------------------------- */
    .section  .isr_vector,"a",%progbits
    .align    2
    .type     g_pfnVectors, %object
    .size     g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
    .word _estack                /* Initial Stack Pointer */
    .word Reset_Handler          /* Reset Handler */
    .word NMI_Handler
    .word HardFault_Handler
    .word MemManage_Handler
    .word BusFault_Handler
    .word UsageFault_Handler
    .word 0
    .word 0
    .word 0
    .word 0
    .word SVC_Handler
    .word DebugMon_Handler
    .word 0
    .word PendSV_Handler
    .word SysTick_Handler

    /* STM32L452 interrupt vectors (match your old table ordering) */
    .word WWDG_IRQHandler
    .word PVD_PVM_IRQHandler
    .word TAMP_STAMP_IRQHandler
    .word RTC_WKUP_IRQHandler
    .word FLASH_IRQHandler
    .word RCC_IRQHandler
    .word EXTI0_IRQHandler
    .word EXTI1_IRQHandler
    .word EXTI2_IRQHandler
    .word EXTI3_IRQHandler
    .word EXTI4_IRQHandler
    .word DMA1_Channel1_IRQHandler
    .word DMA1_Channel2_IRQHandler
    .word DMA1_Channel3_IRQHandler
    .word DMA1_Channel4_IRQHandler
    .word DMA1_Channel5_IRQHandler
    .word DMA1_Channel6_IRQHandler
    .word DMA1_Channel7_IRQHandler
    .word ADC1_IRQHandler
    .word CAN1_TX_IRQHandler
    .word CAN1_RX0_IRQHandler
    .word CAN1_RX1_IRQHandler
    .word CAN1_SCE_IRQHandler
    .word EXTI9_5_IRQHandler
    .word TIM1_BRK_TIM15_IRQHandler
    .word TIM1_UP_TIM16_IRQHandler
    .word TIM1_TRG_COM_IRQHandler
    .word TIM1_CC_IRQHandler
    .word TIM2_IRQHandler
    .word TIM3_IRQHandler
    .word 0                      /* Reserved in your original table */
    .word I2C1_EV_IRQHandler
    .word I2C1_ER_IRQHandler
    .word I2C2_EV_IRQHandler
    .word I2C2_ER_IRQHandler
    .word SPI1_IRQHandler
    .word SPI2_IRQHandler
    .word USART1_IRQHandler
    .word USART2_IRQHandler
    .word USART3_IRQHandler
    .word EXTI15_10_IRQHandler
    .word RTC_Alarm_IRQHandler
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word SDMMC1_IRQHandler
    .word 0
    .word SPI3_IRQHandler
    .word UART4_IRQHandler
    .word 0
    .word TIM6_DAC_IRQHandler
    .word 0
    .word DMA2_Channel1_IRQHandler
    .word DMA2_Channel2_IRQHandler
    .word DMA2_Channel3_IRQHandler
    .word DMA2_Channel4_IRQHandler
    .word DMA2_Channel5_IRQHandler
    .word DFSDM1_FLT0_IRQHandler
    .word DFSDM1_FLT1_IRQHandler
    .word 0
    .word COMP_IRQHandler
    .word LPTIM1_IRQHandler
    .word LPTIM2_IRQHandler
    .word USB_IRQHandler
    .word DMA2_Channel6_IRQHandler
    .word DMA2_Channel7_IRQHandler
    .word LPUART1_IRQHandler
    .word QUADSPI_IRQHandler
    .word I2C3_EV_IRQHandler
    .word I2C3_ER_IRQHandler
    .word SAI1_IRQHandler
    .word 0
    .word 0
    .word TSC_IRQHandler
    .word 0
    .word 0
    .word RNG_IRQHandler
    .word FPU_IRQHandler
    .word CRS_IRQHandler
    .word I2C4_EV_IRQHandler
    .word I2C4_ER_IRQHandler

    /* ----------------------------------------------------------------------
     *  Weak aliases for all exception/IRQ handlers
     * ---------------------------------------------------------------------- */
    .thumb_func
    .weak NMI_Handler
    .set  NMI_Handler,Default_Handler

    .weak HardFault_Handler
    .set  HardFault_Handler,Default_Handler

    .weak MemManage_Handler
    .set  MemManage_Handler,Default_Handler

    .weak BusFault_Handler
    .set  BusFault_Handler,Default_Handler

    .weak UsageFault_Handler
    .set  UsageFault_Handler,Default_Handler

    .weak SVC_Handler
    .set  SVC_Handler,Default_Handler

    .weak DebugMon_Handler
    .set  DebugMon_Handler,Default_Handler

    .weak PendSV_Handler
    .set  PendSV_Handler,Default_Handler

    .weak SysTick_Handler
    .set  SysTick_Handler,Default_Handler

    .weak WWDG_IRQHandler
    .set  WWDG_IRQHandler,Default_Handler

    .weak PVD_PVM_IRQHandler
    .set  PVD_PVM_IRQHandler,Default_Handler

    .weak TAMP_STAMP_IRQHandler
    .set  TAMP_STAMP_IRQHandler,Default_Handler

    .weak RTC_WKUP_IRQHandler
    .set  RTC_WKUP_IRQHandler,Default_Handler

    .weak FLASH_IRQHandler
    .set  FLASH_IRQHandler,Default_Handler

    .weak RCC_IRQHandler
    .set  RCC_IRQHandler,Default_Handler

    .weak EXTI0_IRQHandler
    .set  EXTI0_IRQHandler,Default_Handler

    .weak EXTI1_IRQHandler
    .set  EXTI1_IRQHandler,Default_Handler

    .weak EXTI2_IRQHandler
    .set  EXTI2_IRQHandler,Default_Handler

    .weak EXTI3_IRQHandler
    .set  EXTI3_IRQHandler,Default_Handler

    .weak EXTI4_IRQHandler
    .set  EXTI4_IRQHandler,Default_Handler

    .weak DMA1_Channel1_IRQHandler
    .set  DMA1_Channel1_IRQHandler,Default_Handler

    .weak DMA1_Channel2_IRQHandler
    .set  DMA1_Channel2_IRQHandler,Default_Handler

    .weak DMA1_Channel3_IRQHandler
    .set  DMA1_Channel3_IRQHandler,Default_Handler

    .weak DMA1_Channel4_IRQHandler
    .set  DMA1_Channel4_IRQHandler,Default_Handler

    .weak DMA1_Channel5_IRQHandler
    .set  DMA1_Channel5_IRQHandler,Default_Handler

    .weak DMA1_Channel6_IRQHandler
    .set  DMA1_Channel6_IRQHandler,Default_Handler

    .weak DMA1_Channel7_IRQHandler
    .set  DMA1_Channel7_IRQHandler,Default_Handler

    .weak ADC1_IRQHandler
    .set  ADC1_IRQHandler,Default_Handler

    .weak CAN1_TX_IRQHandler
    .set  CAN1_TX_IRQHandler,Default_Handler

    .weak CAN1_RX0_IRQHandler
    .set  CAN1_RX0_IRQHandler,Default_Handler

    .weak CAN1_RX1_IRQHandler
    .set  CAN1_RX1_IRQHandler,Default_Handler

    .weak CAN1_SCE_IRQHandler
    .set  CAN1_SCE_IRQHandler,Default_Handler

    .weak EXTI9_5_IRQHandler
    .set  EXTI9_5_IRQHandler,Default_Handler

    .weak TIM1_BRK_TIM15_IRQHandler
    .set  TIM1_BRK_TIM15_IRQHandler,Default_Handler

    .weak TIM1_UP_TIM16_IRQHandler
    .set  TIM1_UP_TIM16_IRQHandler,Default_Handler

    .weak TIM1_TRG_COM_IRQHandler
    .set  TIM1_TRG_COM_IRQHandler,Default_Handler

    .weak TIM1_CC_IRQHandler
    .set  TIM1_CC_IRQHandler,Default_Handler

    .weak TIM2_IRQHandler
    .set  TIM2_IRQHandler,Default_Handler

    .weak TIM3_IRQHandler
    .set  TIM3_IRQHandler,Default_Handler

    .weak I2C1_EV_IRQHandler
    .set  I2C1_EV_IRQHandler,Default_Handler

    .weak I2C1_ER_IRQHandler
    .set  I2C1_ER_IRQHandler,Default_Handler

    .weak I2C2_EV_IRQHandler
    .set  I2C2_EV_IRQHandler,Default_Handler

    .weak I2C2_ER_IRQHandler
    .set  I2C2_ER_IRQHandler,Default_Handler

    .weak SPI1_IRQHandler
    .set  SPI1_IRQHandler,Default_Handler

    .weak SPI2_IRQHandler
    .set  SPI2_IRQHandler,Default_Handler

    .weak USART1_IRQHandler
    .set  USART1_IRQHandler,Default_Handler

    .weak USART2_IRQHandler
    .set  USART2_IRQHandler,Default_Handler

    .weak USART3_IRQHandler
    .set  USART3_IRQHandler,Default_Handler

    .weak EXTI15_10_IRQHandler
    .set  EXTI15_10_IRQHandler,Default_Handler

    .weak RTC_Alarm_IRQHandler
    .set  RTC_Alarm_IRQHandler,Default_Handler

    .weak SDMMC1_IRQHandler
    .set  SDMMC1_IRQHandler,Default_Handler

    .weak SPI3_IRQHandler
    .set  SPI3_IRQHandler,Default_Handler

    .weak UART4_IRQHandler
    .set  UART4_IRQHandler,Default_Handler

    .weak TIM6_DAC_IRQHandler
    .set  TIM6_DAC_IRQHandler,Default_Handler

    .weak DMA2_Channel1_IRQHandler
    .set  DMA2_Channel1_IRQHandler,Default_Handler

    .weak DMA2_Channel2_IRQHandler
    .set  DMA2_Channel2_IRQHandler,Default_Handler

    .weak DMA2_Channel3_IRQHandler
    .set  DMA2_Channel3_IRQHandler,Default_Handler

    .weak DMA2_Channel4_IRQHandler
    .set  DMA2_Channel4_IRQHandler,Default_Handler

    .weak DMA2_Channel5_IRQHandler
    .set  DMA2_Channel5_IRQHandler,Default_Handler

    .weak DFSDM1_FLT0_IRQHandler
    .set  DFSDM1_FLT0_IRQHandler,Default_Handler

    .weak DFSDM1_FLT1_IRQHandler
    .set  DFSDM1_FLT1_IRQHandler,Default_Handler

    .weak COMP_IRQHandler
    .set  COMP_IRQHandler,Default_Handler

    .weak LPTIM1_IRQHandler
    .set  LPTIM1_IRQHandler,Default_Handler

    .weak LPTIM2_IRQHandler
    .set  LPTIM2_IRQHandler,Default_Handler

    .weak USB_IRQHandler
    .set  USB_IRQHandler,Default_Handler

    .weak DMA2_Channel6_IRQHandler
    .set  DMA2_Channel6_IRQHandler,Default_Handler

    .weak DMA2_Channel7_IRQHandler
    .set  DMA2_Channel7_IRQHandler,Default_Handler

    .weak LPUART1_IRQHandler
    .set  LPUART1_IRQHandler,Default_Handler

    .weak QUADSPI_IRQHandler
    .set  QUADSPI_IRQHandler,Default_Handler

    .weak I2C3_EV_IRQHandler
    .set  I2C3_EV_IRQHandler,Default_Handler

    .weak I2C3_ER_IRQHandler
    .set  I2C3_ER_IRQHandler,Default_Handler

    .weak SAI1_IRQHandler
    .set  SAI1_IRQHandler,Default_Handler

    .weak TSC_IRQHandler
    .set  TSC_IRQHandler,Default_Handler

    .weak RNG_IRQHandler
    .set  RNG_IRQHandler,Default_Handler

    .weak FPU_IRQHandler
    .set  FPU_IRQHandler,Default_Handler

    .weak CRS_IRQHandler
    .set  CRS_IRQHandler,Default_Handler

    .weak I2C4_EV_IRQHandler
    .set  I2C4_EV_IRQHandler,Default_Handler

    .weak I2C4_ER_IRQHandler
    .set  I2C4_ER_IRQHandler,Default_Handler
