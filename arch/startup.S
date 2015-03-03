/* File: startup_ARMCM3.S
 * Purpose: startup file for Cortex-M3 devices. Should use with 
 *   GCC for ARM Embedded Processors
 * Version: V1.4
 * Date: 09 July 2012
 * 
 * Copyright (c) 2011, 2012, ARM Limited
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the ARM Limited nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ARM LIMITED BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
    .syntax unified
    .arch armv7-m

    .section .stack
    .align 3
#ifdef __STACK_SIZE
    .equ    Stack_Size, __STACK_SIZE
#else
    .equ    Stack_Size, 0xc00
#endif
    .globl    __StackTop
    .globl    __StackLimit
__StackLimit:
    .space    Stack_Size
    .size __StackLimit, . - __StackLimit
__StackTop:
    .size __StackTop, . - __StackTop

    .section .heap
    .align 3
#ifdef __HEAP_SIZE
    .equ    Heap_Size, __HEAP_SIZE
#else
    .equ    Heap_Size, 0
#endif
    .globl    __HeapBase
    .globl    __HeapLimit
__HeapBase:
    .if    Heap_Size
    .space    Heap_Size
    .endif
    .size __HeapBase, . - __HeapBase
__HeapLimit:
    .size __HeapLimit, . - __HeapLimit
    
    .section .isr_vector
    .align 2
    .globl __isr_vector
__isr_vector:
    .long    __StackTop            /* Top of Stack */
    .long    Reset_Handler         /* Reset Handler */
    .long    NMI_Handler           /* NMI Handler */
    .long    HardFault_Handler     /* Hard Fault Handler */
    .long    MemManage_Handler     /* MPU Fault Handler */
    .long    BusFault_Handler      /* Bus Fault Handler */
    .long    UsageFault_Handler    /* Usage Fault Handler */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    SVC_Handler           /* SVCall Handler */
    .long    DebugMon_Handler      /* Debug Monitor Handler */
    .long    0                     /* Reserved */
    .long    PendSV_Handler        /* PendSV Handler */
    .long    SysTick_Handler       /* SysTick Handler */

    /* External interrupts */
    .long    Default_Handler            /* 0: WWDG */
    .long    Default_Handler            /* 1: PVD */
    .long    Default_Handler            /* 2: TAMPER */
    .long    Default_Handler            /* 3: RTC */
    .long    Default_Handler            /* 4: FLASH */
    .long    Default_Handler            /* 5: RCC */
    .long    Default_Handler            /* 6: EXTI0 */
    .long    Default_Handler /*.long    ADS_Handler                 7: EXTI1 */
    .long    EXTI_Handler              	/* 8: EXTI2 */
    .long    EXTI_Handler              	/* 9: EXTI3 */
    .long    EXTI_Handler              	/* 10: EXTI4 */
    .long    Default_Handler           	/* 11: DMA1_Channel1 */
    .long    Default_Handler           	/* 12: DMA1_Channel2 */
    .long    Default_Handler           	/* 13: DMA1_Channel3 */
    .long    Default_Handler           	/* 14: DMA1_Channel4 */
    .long    Default_Handler           	/* 15: DMA1_Channel5 */
    .long    Default_Handler           	/* 16: DMA1_Channel6 */
    .long    UART2_TX_Handler          	/* 17: DMA1_Channel7 */
    .long    ADC_Handler	           	/* 18: ADC1_2 */
    .long    Default_Handler           	/* 19: CAN1_TX */
    .long    Default_Handler           	/* 20: CAN1_RX0 */
    .long    Default_Handler           	/* 21: CAN1_RX1 */
    .long    Default_Handler           	/* 22: CAN1_SCE */
    .long    EXTI_Handler              	/* 23: EXTI9_5 */
    .long    Default_Handler           	/* 24: TIM1_BRK */
    .long    Default_Handler           	/* 25: TIM1_UP */
    .long    Default_Handler           	/* 26: TIM1_TRG_COM */
    .long    Default_Handler           	/* 27: TIM1_CC */
    .long    Default_Handler           	/* 28: TIM2 */
    .long    Default_Handler           	/* 29: TIM3 */
    .long    Default_Handler           	/* 30: TIM4 */
    .long    Default_Handler           	/* 31: I2C1_EV */
    .long    Default_Handler           	/* 32: I2C1_ER */
    .long    Default_Handler           	/* 33: I2C2_EV */
    .long    Default_Handler           	/* 34: I2C2_ER */
    .long    Default_Handler           	/* 35: SPI1 */
    .long    Default_Handler           	/* 36: SPI2 */
    .long    UART1_Handler             	/* 37: USART1 */
    .long    UART2_Handler             	/* 38: USART2 */
    .long    Default_Handler           	/* 39: USART3 */
    .long    EXTI_Handler              	/* 40: EXTI15_10 */
    .long    Default_Handler           	/* 41: RTC_Alarm */
    .long    Default_Handler           	/* 42: OTG_FS_WKUP */
    .long    0                        	/* 43: reserved */
    .long    0                        	/* 44: reserved */
    .long    0                        	/* 45: reserved */
    .long    0                        	/* 46: reserved */
    .long    0                        	/* 47: reserved */
    .long    0                        	/* 48: reserved */
    .long    0                        	/* 49: reserved */
    .long    Default_Handler           	/* 50: TIM5 */
    .long    Default_Handler           	/* 51: SPI3 */
    .long    Default_Handler           	/* 52: UART4 */
    .long    Default_Handler           	/* 53: UART5 */
    .long    modbus_end_of_telegram    	/* 54: TIM6 */
    .long    Default_Handler           	/* 55: TIM7 */
    .long    Default_Handler           	/* 56: DMA2_Channel1 */
    .long    Default_Handler           	/* 57: DMA2_Channel2 */
    .long    Default_Handler           	/* 58: DMA2_Channel3 */
    .long    Default_Handler           	/* 59: DMA2_Channel4 */
    .long    Default_Handler           	/* 60: DMA2_Channel5 */
    .long    Default_Handler           	/* 61: ETH */
    .long    Default_Handler           	/* 62: ETH_WKUP */
    .long    Default_Handler           	/* 63: CAN2_TX */
    .long    Default_Handler           	/* 64: CAN2_RX0 */
    .long    Default_Handler           	/* 65: CAN2_RX1 */
    .long    Default_Handler           	/* 66: CAN2_SCE */
    .long    Default_Handler           	/* 67: OTG_FS */    

    .size    __isr_vector, . - __isr_vector
 
     .text
     .thumb
    .thumb_func
    .align 2
    .globl    Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:
/*     Loop to copy data from read only memory to RAM. The ranges
 *      of copy from/to are specified by following symbols evaluated in 
 *      linker script.
 *      __etext: End of code section, i.e., begin of data sections to copy from.
 *      __data_start__/__data_end__: RAM address range that data should be
 *      copied to. Both must be aligned to 4 bytes boundary.  */

    ldr    r1, =__etext
    ldr    r2, =__data_start__
    ldr    r3, =__data_end__

#if 1
/* Here are two copies of loop implemenations. First one favors code size
 * and the second one favors performance. Default uses the first one. 
 * Change to "#if 0" to use the second one */
.LC0:
    cmp     r2, r3
    ittt    lt
    ldrlt   r0, [r1], #4
    strlt   r0, [r2], #4
    blt    .LC0
#else
    subs    r3, r2
    ble    .LC1    
.LC0:
    subs    r3, #4
    ldr    r0, [r1, r3]
    str    r0, [r2, r3]
    bgt    .LC0
.LC1:
#endif

#ifdef __STARTUP_CLEAR_BSS
/*     This part of work usually is done in C library startup code. Otherwise,
 *     define this macro to enable it in this startup.
 *
 *     Loop to zero out BSS section, which uses following symbols
 *     in linker script:
 *      __bss_start__: start of BSS section. Must align to 4
 *      __bss_end__: end of BSS section. Must align to 4
 */
    ldr r1, =__bss_start__
    ldr r2, =__bss_end__

    movs    r0, 0
.LC2:
    cmp     r1, r2
    itt    lt
    strlt   r0, [r1], #4
    blt    .LC2
#endif /* __STARTUP_CLEAR_BSS */

#ifndef __NO_SYSTEM_INIT
    bl    SystemInit
#endif

#ifndef __START
#define __START _start
#endif
    bl    __START
    .pool
    .size Reset_Handler, . - Reset_Handler
    
/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
    .macro    def_default_handler    handler_name
    .align 1
    .thumb_func
    .weak    \handler_name
    .type    \handler_name, %function
\handler_name :
    b    .
    .size    \handler_name, . - \handler_name
    .endm
    
    def_default_handler    NMI_Handler
/*    def_default_handler    HardFault_Handler */
    def_default_handler    MemManage_Handler
    def_default_handler    BusFault_Handler
    def_default_handler    UsageFault_Handler
    def_default_handler    SVC_Handler
    def_default_handler    DebugMon_Handler
    def_default_handler    PendSV_Handler
    def_default_handler    SysTick_Handler
    def_default_handler    Default_Handler

    .weak    DEF_IRQHandler
    .set    DEF_IRQHandler, Default_Handler

    .end
