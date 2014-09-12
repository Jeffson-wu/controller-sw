/**
  ******************************************************************************
  * @file    swupdate.h 
  * @author  Jari Rene Jensen
  * @version V1.0.0
  * @date    10-Feb - 2014
  * @brief   Header for SW update task
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 Xtel </center></h2>
  *
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SWUPDATETASK_H
#define __SWUPDATETASK_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables ------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */
extern int SWU_start_task(void);
extern void startSWUpdate(void);
extern void SWUpdateTask( void * pvParameters );

#endif /* __SWUPDATETASK_H */

/************************ (C) COPYRIGHT Xtel *****END OF FILE****/
