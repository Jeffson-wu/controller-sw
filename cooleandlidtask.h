/**
  ******************************************************************************
  * @file    cooleandlidtask.h 
  * @author  Jari Rene Jensen
  * @version V1.0.0
  * @date    30-Oct -2013
  * @brief   Header for Cooler and lid heater task
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 Xtel </center></h2>
  *
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COOLERANDLIDTASK_H
#define __COOLERANDLIDTASK_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum {
  LID_ADDR = 17,
  PELTIER_ADDR,
  //PELTIER_ADDR2,
  FAN_ADDR,
  nofADDR
} coolandlid_addrs_t;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void CoolAndLidTask( void * pvParameters );
int getClLog(char *poutText );
bool coolLidReadRegs(u8 slave, u16 addr, u16 datasize, u16 *buffer);
bool coolLidWriteRegs(u8 slave, u16 addr, u16 *data, u16 datasize);

#endif /* __COOLERANDLIDTASK_H */

/************************ (C) COPYRIGHT Xtel *****END OF FILE****/

