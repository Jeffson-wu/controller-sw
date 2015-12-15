/**
  ******************************************************************************
  * @file    logtask.h 
  * @author  Jari Rene Jensen
  * @version V1.0.0
  * @date    10-Feb - 2014
  * @brief   Header for temperature logging task
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 Xtel </center></h2>
  *
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LOGTASK_H
#define __LOGTASK_H

/* Includes ------------------------------------------------------------------*/
#include <timers.h>
/* Exported types ------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables ------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */
void vReadTubeTemp(xTimerHandle pxTimer );
void LogTask( void * pvParameters );
void sendLog();
void emptyLog(int tubeId);
int addLog(char *poutText,int tubeId, int size );

#endif /* __LOGTASK_H */

/************************ (C) COPYRIGHT Xtel *****END OF FILE****/

