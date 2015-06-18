/**
  ******************************************************************************
  * @file    debug.h 
  * @author  Jari Rene Jensen
  * @version V1.0.0
  * @date    19 - May - 2015
  * @brief   Debug features
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 Xtel </center></h2>
  *
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEBUG_H
#define __DEBUG_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define TRACE_ISR_ID_UART1 1
#define TRACE_ISR_ID_UART2 2
#define TRACE_ISR_ID_UART3 3
#define TRACE_ISR_ID_UART2_TX 4
#define TRACE_ISR_ID_EXTI 5
#define TRACE_ISR_ID_ADC 5
#define TRACE_ISR_ID_MB_EOT 6

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

inline void dbgTraceStoreISRBegin(int id);
inline void dbgTraceStoreISREnd();
char *dbgPrintIsr(char *buf);




#endif /* __DEBUG_H */
