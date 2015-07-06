/******************************************************************************* 
 * Tracealyzer v2.5.1 Recorder Library
 * Percepio AB, www.percepio.com
 *
 * trcHardwarePort.h
 *
 * Contains together with trcHardwarePort.c all hardware portability issues of 
 * the trace recorder library.
 *
 * Terms of Use
 * This software is copyright Percepio AB. The recorder library is free for
 * use together with Percepio products. You may distribute the recorder library
 * in its original form, including modifications in trcPort.c and trcPort.h
 * given that these modification are clearly marked as your own modifications
 * and documented in the initial comment section of these source files. 
 * This software is the intellectual property of Percepio AB and may not be 
 * sold or in other ways commercially redistributed without explicit written 
 * permission by Percepio AB.
 *
 * Disclaimer 
 * The trace tool and recorder library is being delivered to you AS IS and 
 * Percepio AB makes no warranty as to its use or performance. Percepio AB does 
 * not and cannot warrant the performance or results you may obtain by using the 
 * software or documentation. Percepio AB make no warranties, express or 
 * implied, as to noninfringement of third party rights, merchantability, or 
 * fitness for any particular purpose. In no event will Percepio AB, its 
 * technology partners, or distributors be liable to you for any consequential, 
 * incidental or special damages, including any lost profits or lost savings, 
 * even if a representative of Percepio AB has been advised of the possibility 
 * of such damages, or for any claim by any third party. Some jurisdictions do 
 * not allow the exclusion or limitation of incidental, consequential or special 
 * damages, or the exclusion of implied warranties or limitations on how long an 
 * implied warranty may last, so the above limitations may not apply to you.
 *
 * Copyright Percepio AB, 2013.
 * www.percepio.com
 ******************************************************************************/

#ifndef TRCPORT_H
#define TRCPORT_H

#include "trcKernelPort.h"

/* If Win32 port */
#ifdef WIN32

   #undef _WIN32_WINNT
   #define _WIN32_WINNT 0x0600

   /* Standard includes. */
   #include <stdio.h>
   #include <windows.h>
   #include <direct.h>

/*******************************************************************************
 * The Win32 port by default saves the trace to file and then kills the
 * program when the recorder is stopped, to facilitate quick, simple tests
 * of the recorder.
 ******************************************************************************/
   #define WIN32_PORT_SAVE_WHEN_STOPPED 1
   #define WIN32_PORT_EXIT_WHEN_STOPPED 1

#endif

#define DIRECTION_INCREMENTING 1
#define DIRECTION_DECREMENTING 2

/******************************************************************************
 * Supported ports
 * 
 * PORT_HWIndependent
 * A hardware independent fallback option for event timestamping. Provides low 
 * resolution timestamps based on the OS tick.
 * This may be used on the Win32 port, but may also be used on embedded hardware 
 * platforms. All time durations will be truncated to the OS tick frequency, 
 * typically 1 KHz. This means that a task or ISR that executes in less than 
 * 1 ms get an execution time of zero.
 *
 * PORT_APPLICATION_DEFINED
 * Allows for defining the port macros in other source code files.
 *
 * PORT_Win32
 * "Accurate" timestamping based on the Windows performance counter for Win32 
 * builds. Note that this gives the host machine time, not the kernel time.
 *
 * Hardware specific ports
 * To get accurate timestamping, a hardware timer is necessary. Below are the 
 * available ports. Some of these are "unofficial", meaning that 
 * they have not yet been verified by Percepio but have been contributed by 
 * external developers. They should work, otherwise let us know by emailing 
 * support@percepio.com. Some work on any OS platform, while other are specific 
 * to a certain operating system.
 *****************************************************************************/

/****** Port Name ******************** Code ** Official ** OS Platform *******/
#define PORT_APPLICATION_DEFINED       -2   /* -           -                 */
#define PORT_NOT_SET                   -1   /* -           -                 */
#define PORT_HWIndependent             0    /* Yes         Any               */
#define PORT_Win32                     1    /* Yes         Windows           */
#define PORT_Atmel_AT91SAM7            2    /* No          Any               */
#define PORT_Atmel_UC3A0               3    /* No          Any               */
#define PORT_ARM_CortexM               4    /* Yes         Any               */
#define PORT_Renesas_RX600             5    /* Yes         Any               */
#define PORT_Microchip_dsPIC_AND_PIC24 6    /* Yes         Any               */
#define PORT_TEXAS_INSTRUMENTS_TMS570  7    /* No          Any               */
#define PORT_TEXAS_INSTRUMENTS_MSP430  8    /* No          Any               */
#define PORT_MICROCHIP_PIC32           9    /* No          Any               */
#define PORT_XILINX_PPC405             10   /* No          FreeRTOS          */
#define PORT_XILINX_PPC440             11   /* No          FreeRTOS          */
#define PORT_XILINX_MICROBLAZE         12   /* No          Any               */
#define PORT_NXP_LPC210X               13   /* No          Any               */

/*** Select the port name here! **********************************************/
#define SELECTED_PORT PORT_ARM_CortexM
/*****************************************************************************/

#if (SELECTED_PORT == PORT_NOT_SET) 
#error "You need to define SELECTED_PORT here!"
#endif

/*******************************************************************************
 * IRQ_PRIORITY_ORDER
 *
 * Macro which should be defined as an integer of 0 or 1.
 *
 * This should be 0 if lower IRQ priority values implies higher priority 
 * levels, such as on ARM Cortex M. If the opposite scheme is used, i.e., 
 * if higher IRQ priority values means higher priority, this should be 1.
 *
 * This setting is not critical. It is used only to sort and colorize the 
 * interrupts in priority order, in case you record interrupts using
 * the vTraceStoreISRBegin and vTraceStoreISREnd routines.
 *
 ******************************************************************************
 *
 * HWTC Macros 
 *
 * These four HWTC macros provides a hardware isolation layer representing a 
 * generic hardware timer/counter used for driving the operating system tick, 
 * such as the SysTick feature of ARM Cortex M3/M4, or the PIT of the Atmel 
 * AT91SAM7X.
 *
 * HWTC_COUNT: The current value of the counter. This is expected to be reset 
 * a each tick interrupt. Thus, when the tick handler starts, the counter has 
 * already wrapped.
 *
 * HWTC_COUNT_DIRECTION: Should be one of:
 * - DIRECTION_INCREMENTING - for hardware timer/counters of incrementing type
 *   such as the PIT on Atmel AT91SAM7X.
 *   When the counter value reach HWTC_PERIOD, it is reset to zero and the
 *   interrupt is signaled.
 * - DIRECTION_DECREMENTING - for hardware timer/counters of decrementing type
 *   such as the SysTick on ARM Cortex M3/M4 chips.
 *   When the counter value reach 0, it is reset to HWTC_PERIOD and the
 *   interrupt is signaled.
 *
 * HWTC_PERIOD: The number of increments or decrements of HWTC_COUNT between
 * two OS tick interrupts. This should preferably be mapped to the reload
 * register of the hardware timer, to make it more portable between chips in the 
 * same family. The macro should in most cases be (reload register + 1).
 * For FreeRTOS, this can in most cases be defined as 
 * #define HWTC_PERIOD (configCPU_CLOCK_HZ / configTICK_RATE_HZ)
 *
 * HWTC_DIVISOR: If the timer frequency is very high, like on the Cortex M chips
 * (where the SysTick runs at the core clock frequency), the "differential 
 * timestamping" used in the recorder will more frequently insert extra XTS 
 * events to store the timestamps, which increases the event buffer usage. 
 * In such cases, to reduce the number of XTS events and thereby get longer 
 * traces, you use HWTC_DIVISOR to scale down the timestamps and frequency.
 * Assuming a OS tick rate of 1 KHz, it is suggested to keep the effective timer
 * frequency below 65 MHz to avoid an excessive amount of XTS events. Thus, a
 * Cortex M chip running at 72 MHZ should use a HWTC_DIVISOR of 2, while a 
 * faster chip require a higher HWTC_DIVISOR value. 
 *
 * The HWTC macros and vTracePortGetTimeStamp is the main porting issue
 * or the trace recorder library. Typically you should not need to change
 * the code of vTracePortGetTimeStamp if using the HWTC macros.
 *
 ******************************************************************************/

#if (SELECTED_PORT == PORT_Win32)
    
    #define HWTC_COUNT_DIRECTION DIRECTION_INCREMENTING
    #define HWTC_COUNT (ulGetRunTimeCounterValue())
    #define HWTC_PERIOD 0
    #define HWTC_DIVISOR 1
    #define IRQ_PRIORITY_ORDER 1  // Please update according to your hardware...

#elif (SELECTED_PORT == PORT_HWIndependent)
    
    #define HWTC_COUNT_DIRECTION DIRECTION_INCREMENTING
    #define HWTC_COUNT 0
    #define HWTC_PERIOD 1
    #define HWTC_DIVISOR 1
    #define IRQ_PRIORITY_ORDER 1  // Please update according to your hardware...

#elif (SELECTED_PORT == PORT_ARM_CortexM)

    /* For all chips using ARM Cortex M cores */

    #define HWTC_COUNT_DIRECTION DIRECTION_DECREMENTING
    #define HWTC_COUNT (*((uint32_t*)0xE000E018))
    #define HWTC_PERIOD ((*(uint32_t*)0xE000E014) + 1)
    #define HWTC_DIVISOR 4    
    #define IRQ_PRIORITY_ORDER 0  // lower IRQ priority values are more significant

#elif (SELECTED_PORT == PORT_Renesas_RX600)    

    #include "iodefine.h"

    #define HWTC_COUNT_DIRECTION DIRECTION_INCREMENTING
    #define HWTC_COUNT (CMT0.CMCNT)
    #define HWTC_PERIOD (CMT0.CMCOR + 1)
    #define HWTC_DIVISOR 1
    #define IRQ_PRIORITY_ORDER 1  // higher IRQ priority values are more significant

#elif (SELECTED_PORT == PORT_Microchip_dsPIC_AND_PIC24)

    /* For Microchip PIC24 and dsPIC (16 bit) */

    /* Note: The trace library was originally designed for 32-bit MCUs, and is slower
       than intended on 16-bit MCUs. Storing an event on a PIC24 takes about 70 �s. 
       In comparison, 32-bit MCUs are often 10-20 times faster. If recording overhead 
       becomes a problem on PIC24, use the filters to exclude less interesting tasks 
       or system calls. */

    #define HWTC_COUNT_DIRECTION DIRECTION_INCREMENTING
    #define HWTC_COUNT (TMR1)
    #define HWTC_PERIOD (PR1+1)
    #define HWTC_DIVISOR 1
    #define IRQ_PRIORITY_ORDER 0  // lower IRQ priority values are more significant

#elif (SELECTED_PORT == PORT_Atmel_AT91SAM7)

    /* UNOFFICIAL PORT - NOT YET VERIFIED BY PERCEPIO */
    	
    #define HWTC_COUNT_DIRECTION DIRECTION_INCREMENTING
    #define HWTC_COUNT ((uint32_t)(AT91C_BASE_PITC->PITC_PIIR & 0xFFFFF))
    #define HWTC_PERIOD ((uint32_t)(AT91C_BASE_PITC->PITC_PIMR + 1))
    #define HWTC_DIVISOR 1
    #define IRQ_PRIORITY_ORDER 1  // higher IRQ priority values are more significant

#elif (SELECTED_PORT == PORT_Atmel_UC3A0)

    /* UNOFFICIAL PORT - NOT YET VERIFIED BY PERCEPIO */
    /* For Atmel AVR32 (AT32UC3A).*/
	
    #define HWTC_COUNT_DIRECTION DIRECTION_INCREMENTING
    #define HWTC_COUNT ((uint32_t)sysreg_read(AVR32_COUNT))
    #define HWTC_PERIOD ((uint32_t)(sysreg_read(AVR32_COMPARE) + 1))
    #define HWTC_DIVISOR 1
    #define IRQ_PRIORITY_ORDER 1  // higher IRQ priority values are more significant

#elif (SELECTED_PORT == PORT_NXP_LPC210X)

    /* UNOFFICIAL PORT - NOT YET VERIFIED BY PERCEPIO */   
    /* Tested with LPC2106, but should work with most LPC21XX chips. */
    	
    #define HWTC_COUNT_DIRECTION DIRECTION_INCREMENTING
    #define HWTC_COUNT  *((uint32_t *)0xE0004008 )
    #define HWTC_PERIOD *((uint32_t *)0xE0004018 )
    #define HWTC_DIVISOR 1    
    #define IRQ_PRIORITY_ORDER 0  // lower IRQ priority values are more significant

#elif (SELECTED_PORT == PORT_TEXAS_INSTRUMENTS_TMS570)
 
    /* UNOFFICIAL PORT - NOT YET VERIFIED BY PERCEPIO */

    #define RTIFRC0 *((uint32_t *)0xFFFFFC10)
    #define RTICOMP0 *((uint32_t *)0xFFFFFC50)
    #define RTIUDCP0 *((uint32_t *)0xFFFFFC54)
    #define HWTC_COUNT_DIRECTION DIRECTION_INCREMENTING
    #define HWTC_COUNT (RTIFRC0 - (RTICOMP0 - RTIUDCP0))
    #define HWTC_PERIOD (RTIUDCP0)
    #define HWTC_DIVISOR 1

    #define IRQ_PRIORITY_ORDER 0  // lower IRQ priority values are more significant

#elif (SELECTED_PORT == PORT_TEXAS_INSTRUMENTS_MSP430)

    /* UNOFFICIAL PORT - NOT YET VERIFIED BY PERCEPIO */

    #define HWTC_COUNT_DIRECTION DIRECTION_INCREMENTING
    #define HWTC_COUNT (TA0R)
    #define HWTC_PERIOD (((uint16_t)TACCR0)+1)
    #define HWTC_DIVISOR 1
    #define IRQ_PRIORITY_ORDER 1  // higher IRQ priority values are more significant

#elif (SELECTED_PORT == PORT_MICROCHIP_PIC32)

    /* UNOFFICIAL PORT - NOT YET VERIFIED BY PERCEPIO */

    #define HWTC_COUNT_DIRECTION DIRECTION_INCREMENTING
    #define HWTC_COUNT (ReadTimer1())     /* Should be available in BSP */
    #define HWTC_PERIOD (ReadPeriod1()+1) /* Should be available in BSP */
    #define HWTC_DIVISOR 1
    #define IRQ_PRIORITY_ORDER 0  // lower IRQ priority values are more significant

#elif (SELECTED_PORT == PORT_XILINX_PPC405)
    
    /* UNOFFICIAL PORT - NOT YET VERIFIED BY PERCEPIO */

    #define HWTC_COUNT_DIRECTION DIRECTION_DECREMENTING
    #define HWTC_COUNT  mfspr(0x3db)	
	#if (defined configCPU_CLOCK_HZ && defined configTICK_RATE_HZ) // Check if FreeRTOS
	    /* For FreeRTOS only - found no generic OS independent solution for the PPC405 architecture. */
	    #define HWTC_PERIOD ( configCPU_CLOCK_HZ / configTICK_RATE_HZ ) // Same as in port.c for PPC405
	#else
	    /* Not defined for other operating systems yet */
	    #error HWTC_PERIOD must be defined to give the number of hardware timer ticks per OS tick.
	#endif	
    #define HWTC_DIVISOR 1
    #define IRQ_PRIORITY_ORDER 0  // lower IRQ priority values are more significant

#elif (SELECTED_PORT == PORT_XILINX_PPC440)
    
	/* UNOFFICIAL PORT - NOT YET VERIFIED BY PERCEPIO */
    /* This should work with most PowerPC chips */    
    
	#define HWTC_COUNT_DIRECTION DIRECTION_DECREMENTING
    #define HWTC_COUNT  mfspr(0x016)
	#if (defined configCPU_CLOCK_HZ && defined configTICK_RATE_HZ) // Check if FreeRTOS
		/* For FreeRTOS only - found no generic OS independent solution for the PPC440 architecture. */
		#define HWTC_PERIOD ( configCPU_CLOCK_HZ / configTICK_RATE_HZ ) // Same as in port.c for PPC440
	#else
		/* Not defined for other operating systems yet */
		#error HWTC_PERIOD must be defined to give the number of hardware timer ticks per OS tick.
	#endif
    #define HWTC_DIVISOR 1    
    #define IRQ_PRIORITY_ORDER 0  // lower IRQ priority values are more significant
    
#elif (SELECTED_PORT == PORT_XILINX_MICROBLAZE)

    /* UNOFFICIAL PORT - NOT YET VERIFIED BY PERCEPIO */

    /* This should work with most Microblaze configurations.
       It uses the AXI Timer 0 - the tick interrupt source.
       If an AXI Timer 0 peripheral is available on your hardware platform, no modifications are required. */
	
    #include "xtmrctr_l.h"

    #define HWTC_COUNT_DIRECTION DIRECTION_DECREMENTING
    #define HWTC_COUNT XTmrCtr_GetTimerCounterReg( XPAR_TMRCTR_0_BASEADDR, 0 )
  	#define HWTC_PERIOD (XTmrCtr_mGetLoadReg( XPAR_TMRCTR_0_BASEADDR,  0) + 1)
    #define HWTC_DIVISOR 16
    #define IRQ_PRIORITY_ORDER 0  // lower IRQ priority values are more significant

#elif (SELECTED_PORT == PORT_APPLICATION_DEFINED)

	#if !( defined (HWTC_COUNT_DIRECTION) && defined (HWTC_COUNT) && defined (HWTC_PERIOD) && defined (HWTC_DIVISOR) && defined (IRQ_PRIORITY_ORDER) )
		#error SELECTED_PORT is PORT_APPLICATION_DEFINED but not all of the necessary constants have been defined.
	#endif

#elif (SELECTED_PORT != PORT_NOT_SET)

    #error "SELECTED_PORT had unsupported value!"
    #define SELECTED_PORT PORT_NOT_SET

#endif

#if (SELECTED_PORT != PORT_NOT_SET)
    
    #ifndef HWTC_COUNT_DIRECTION
    #error "HWTC_COUNT_DIRECTION is not set!"
    #endif 
    
    #ifndef HWTC_COUNT
    #error "HWTC_COUNT is not set!"    
    #endif 
    
    #ifndef HWTC_PERIOD
    #error "HWTC_PERIOD is not set!"
    #endif 
    
    #ifndef HWTC_DIVISOR
    #error "HWTC_DIVISOR is not set!"    
    #endif 
    
    #ifndef IRQ_PRIORITY_ORDER
    #error "IRQ_PRIORITY_ORDER is not set!"
    #elif (IRQ_PRIORITY_ORDER != 0) && (IRQ_PRIORITY_ORDER != 1)
    #error "IRQ_PRIORITY_ORDER has bad value!"
    #endif 
    
    #if (HWTC_DIVISOR < 1)
    #error "HWTC_DIVISOR must be a non-zero positive value!"
    #endif 
	
#endif

/*******************************************************************************
 * vTraceConsoleMessage
 *
 * A wrapper for your system-specific console "printf" console output function.
 * This needs to be correctly defined to see status reports from the trace 
 * status monitor task (this is defined in trcUser.c).
 ******************************************************************************/         
#define vTraceConsoleMessage(fmt, args...)   {char buf[100]; sprintf(buf, fmt, ## args);  gdi_send_msg_on_monitor(buf);}

/*******************************************************************************
 * vTracePortGetTimeStamp
 *
 * Returns the current time based on the HWTC macros which provide a hardware
 * isolation layer towards the hardware timer/counter.
 *
 * The HWTC macros and vTracePortGetTimeStamp is the main porting issue
 * or the trace recorder library. Typically you should not need to change
 * the code of vTracePortGetTimeStamp if using the HWTC macros.
 *
 ******************************************************************************/
void vTracePortGetTimeStamp(uint32_t *puiTimestamp);

/*******************************************************************************
 * vTracePortEnd
 * 
 * This function is called when the recorder is stopped due to full buffer.
 * Mainly intended to show a message in the console.
 * This is used by the Win32 port to store the trace to a file. The file path is
 * set using vTracePortSetFileName.
 ******************************************************************************/
void vTracePortEnd(void);

#endif
