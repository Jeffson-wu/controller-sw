/**
  ******************************************************************************
  * @file    debug.c
  * @author  Jari Rene Jensen
  * @version V1.0.0
  * @date    19 - May - 2015
  * @brief   CDebug functionality
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 Xtel </center></h2>
  *
  ******************************************************************************
  */ 
  
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "serial.h"
#include "debug.h"
/* ---------------------------------------------------------------------------*/
/* Global Debug variables ----------------------------------------------------*/
int dbgActiveISRid = 0;
int dbgLastActiveISRid = 0;
char dbgbuf[DEBUG_BUFFER_SIZE]; /*buffer for debug printf*/

extern void stopPeltier(void);
extern void PWM_Stop(void);
extern void gdi_send_msg_response(char * response);

/* Private feature defines ---------------------------------------------------*/
/* Private debug defines -----------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/
/* Private prototypes                                                         */
/* ---------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------*/
/* functions                                                                  */
/* ---------------------------------------------------------------------------*/

// make weak versions of functions like stopPeltier()

/* ---------------------------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/
inline void dbgTraceStoreISRBegin(int id)
{
  dbgActiveISRid = id;
}

/* ---------------------------------------------------------------------------*/
inline void dbgTraceStoreISREnd()
{
  dbgLastActiveISRid = dbgActiveISRid;
  dbgActiveISRid = 0;
}

/* ---------------------------------------------------------------------------*/
char *dbgPrintIsr(char *str)
{
  sprintf(str, "Active ISR: %d LastActive ISR: %d\n", dbgActiveISRid, dbgLastActiveISRid);
  return str;
}

/* ---------------------------------------------------------------------------*/
void send_msg_on_monitor(char * response)
{
  if(USART3_intitalized)
  {
    char i = 0;
    int len = strlen(response)+3;
    char message[strlen(response)+3];
    strcpy(message, "\0");
    strcat(message, response);
    strcat(message, "\r\n");
    while(i<len)
    {
      while(USART_GetFlagStatus(USART3, USART_FLAG_TXE)==RESET);
      USART_SendData(USART3,*(message+i));
      i++;
    }
  }
}

/* ---------------------------------------------------------------------------*/
void printHeap(void) {
  extern size_t xFreeBytesRemaining;
  sprintf(dbgbuf, "Heap free bytes: %d", xFreeBytesRemaining);
  send_msg_on_monitor(dbgbuf);
}

/* ---------------------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(unsigned char* file, unsigned int line)
{ extern size_t xFreeBytesRemaining;
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  char afbuf[100];
  u8 data;
  GPIO_SetBits(GPIOB,GPIO_Pin_11);    /* Turn on error LED */
  GPIO_ResetBits(GPIOC,GPIO_Pin_9);   /* Turn off hartbeat LED */  
  GPIO_ResetBits(GPIOB,GPIO_Pin_0);   /* Turn off RX LED */
  GPIO_ResetBits(GPIOB,GPIO_Pin_1);   /* Turn off TX LED */
  PWM_Stop();
  stopPeltier();
  sprintf(afbuf, "assert_failed: %s %d", file, line);
  send_msg_on_monitor(afbuf);
  sprintf(afbuf, "Heap free bytes: %d", xFreeBytesRemaining);
  send_msg_on_monitor(afbuf);
  
  sprintf(afbuf, "Task: %s", pcTaskGetTaskName( xTaskGetCurrentTaskHandle()) );
  send_msg_on_monitor(afbuf);
  send_msg_on_monitor(dbgPrintIsr(afbuf));

  //__asm("BKPT #0\n") ; // Break into the debugger

  /* Infinite loop */
  /* cmd = "bu" -> "OK" to let Linux know that the M3 crashed */
  while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE)==RESET);
  data = USART_ReceiveData(USART1);
  if('B' == (data & 0x0DF) )
  { 
    while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE)==RESET);
    data = USART_ReceiveData(USART1);
    if('U' == (data & 0x0DF) )
    { 
      while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE)==RESET);
      data = USART_ReceiveData(USART1);
      if('\r' == data)
      { 
        gdi_send_msg_response("OK");
      }
    }
  }
}
#endif

void HardFault_Handler(void)
{
 __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler2_address_const                            \n"
        " bx r2                                                     \n"
        " handler2_address_const: .word prvGetRegistersFromStack    \n"
    );
}

/* ---------------------------------------------------------------------------*/
void prvGetRegistersFromStack( uint32_t *pulFaultStackAddress )
{
  static uint32_t hardFaultSP __attribute__ ((section (".regPointer")));
  u8 data;
  int i;
  char hfbuf[100];
  /* These are volatile to try and prevent the compiler/linker optimising them
  away as the variables never actually get used.  If the debugger won't show the
  values of the variables, make them global by moving their declaration outside
  of this function. */
  volatile uint32_t r[16];
  /*                sp is held in pulFaultStackAddress */
  volatile uint32_t lr; /* Link register. r14 */
  volatile uint32_t pc; /* Program counter. r15 */
  volatile uint32_t psr;/* Program status register. */
  volatile uint32_t _CFSR;
  volatile uint32_t _HFSR;
  volatile uint32_t _DFSR;
  volatile uint32_t _AFSR;
  volatile uint32_t _MMAR;
  volatile uint32_t _BFAR;

  register unsigned int _r4  __asm("r4");
  register unsigned int _r5  __asm("r5");
  register unsigned int _r6  __asm("r6");
  register unsigned int _r7  __asm("r7");
  register unsigned int _r8  __asm("r8");
  register unsigned int _r9  __asm("r9");
  register unsigned int _r10 __asm("r10");
  register unsigned int _r11 __asm("r11");

  register unsigned int _r13 __asm("r13");

  hardFaultSP = _r13; // Save current SP to find variables below in a RAM dump.
  r[0]  = pulFaultStackAddress[ 0 ];
  r[1]  = pulFaultStackAddress[ 1 ];
  r[2]  = pulFaultStackAddress[ 2 ];
  r[3]  = pulFaultStackAddress[ 3 ];
  r[4]  = _r4;
  r[5]  = _r5;
  r[6]  = _r6;
  r[7]  = _r7;
  r[8]  = _r8;
  r[9]  = _r9;
  r[10] = _r10;
  r[11] = _r11;
  r[12] = pulFaultStackAddress[ 4 ];
  //r[13] = _r13;
  lr = pulFaultStackAddress[ 5 ];
  pc = pulFaultStackAddress[ 6 ];
  psr = pulFaultStackAddress[ 7 ];

  // Configurable Fault Status Register
  // Consists of MMSR, BFSR and UFSR
  _CFSR = (*((volatile unsigned long *)(0xE000ED28))) ;   
  
  // Hard Fault Status Register
  _HFSR = (*((volatile unsigned long *)(0xE000ED2C))) ;
  
  // Debug Fault Status Register
  _DFSR = (*((volatile unsigned long *)(0xE000ED30))) ;
  
  // Auxiliary Fault Status Register
  _AFSR = (*((volatile unsigned long *)(0xE000ED3C))) ;
  
  // Read the Fault Address Registers. These may not contain valid values.
  // Check BFARVALID/MMARVALID to see if they are valid values
  // MemManage Fault Address Register
  _MMAR = (*((volatile unsigned long *)(0xE000ED34))) ;
  // Bus Fault Address Register
  _BFAR = (*((volatile unsigned long *)(0xE000ED38))) ;

  GPIO_SetBits(GPIOB,GPIO_Pin_11);  /* Turn on error LED */
  GPIO_ResetBits(GPIOC,GPIO_Pin_9); /* Turn off hartbeat LED */
  GPIO_SetBits(GPIOB,GPIO_Pin_0);   /* Turn on RX LED */
  GPIO_SetBits(GPIOB,GPIO_Pin_1);   /* Turn on TX LED */
  PWM_Stop();
  send_msg_on_monitor("\r\n!! HardFault !!"); //Print PC and SP for quick ref.
  //Bus Fault Status Register
  if(_CFSR & 0x00000100) { send_msg_on_monitor("IBUSRR"); }
  if(_CFSR & 0x00000200) { send_msg_on_monitor("PRECISERR"); }
  if(_CFSR & 0x00000400) { send_msg_on_monitor("IMPRECISERR"); }
  if(_CFSR & 0x00000800) { send_msg_on_monitor("UNSTKERR"); }
  if(_CFSR & 0x00001000) { send_msg_on_monitor("STKERR"); }
  if(_CFSR & 0x00008000) { 
      sprintf(hfbuf, "BFARVALID  - BFAR: %08X", (unsigned int)_BFAR);
      send_msg_on_monitor(hfbuf);
    }
  //Usage Fault Status Register
  if(_CFSR & 0x00010000) { send_msg_on_monitor("UNDEFINSTR"); }
  if(_CFSR & 0x00020000) { send_msg_on_monitor("INVSTATE"); }
  if(_CFSR & 0x00040000) { send_msg_on_monitor("INVPC"); }
  if(_CFSR & 0x00080000) { send_msg_on_monitor("NOCP"); }
  if(_CFSR & 0x00200000) { send_msg_on_monitor("DIVBYZERO"); }
  if(_CFSR & 0x00100000) { send_msg_on_monitor("UNALIGNED"); }
  //Memory Manage Fault Status Register
  if(_CFSR & 0x00000080) { send_msg_on_monitor("MMARVALID"); }
  if(_CFSR & 0x00000010) { send_msg_on_monitor("MSTKERR"); }
  if(_CFSR & 0x00000008) { send_msg_on_monitor("MUNSTKERR"); }
  if(_CFSR & 0x00000002) { send_msg_on_monitor("DACCVIOL"); }
  if(_CFSR & 0x00000001) { send_msg_on_monitor("IACCVIOL"); }
  sprintf(hfbuf, "Task: %s", pcTaskGetTaskName( xTaskGetCurrentTaskHandle()) );
  send_msg_on_monitor(hfbuf);
  
  for(i = 0; i < 13; i++ )
  {
    sprintf(hfbuf, "r%d: 0x%08X", i, (unsigned int)r[i]);
    send_msg_on_monitor(hfbuf);
  }
  sprintf(hfbuf, "sp: 0x%08X", (unsigned int)hardFaultSP);
  send_msg_on_monitor(hfbuf);
  sprintf(hfbuf, "lr: 0x%08X", (unsigned int)lr);
  send_msg_on_monitor(hfbuf);
  sprintf(hfbuf, "pc: 0x%08X", (unsigned int)pc);
  send_msg_on_monitor(hfbuf);
  sprintf(hfbuf, "psr: 0x%08X", (unsigned int)psr);
  send_msg_on_monitor(hfbuf);
 
  send_msg_on_monitor(dbgPrintIsr(hfbuf));
  
  //__asm("BKPT #0\n") ; // Break into the debugger

  /* cmd = "bu" -> "OK" to let Linux know that the M3 crashed */
  while(1)
  {
    while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE)==RESET);
    data = USART_ReceiveData(USART1);
    if('B' == (data & 0x0DF) )
    { 
      while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE)==RESET);
      data = USART_ReceiveData(USART1);
      if('U' == (data & 0x0DF) )
      { 
        while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE)==RESET);
        data = USART_ReceiveData(USART1);
        if('\r' == data)
        { 
          gdi_send_msg_response("OK");
        }
      }
    }
  }
  lr=lr;
  pc=pc;
  psr=psr;
  hardFaultSP=hardFaultSP;
  
  _CFSR=_CFSR;
  _HFSR=_HFSR;
  _DFSR=_DFSR;
  _AFSR=_AFSR;
  _MMAR=_MMAR;
  _BFAR=_BFAR;
}


