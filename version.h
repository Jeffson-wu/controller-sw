/**
  ******************************************************************************
  * @file    version.h 
  * @author  Jari Rene Jensen
  * @version V1.0.0
  * @date    9-Apr-2015
  * @brief   Header for SW Revision, Build date and Git Commit Id
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 Xtel </center></h2>
  *
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VERSION_H
#define __VERSION_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

// SW version
#define SW_VERSION "2.000.000"

// Strings in file version.c generated from the build process - Do NOT commit version.c to Git!!
extern char buildRevStr[]    __attribute__ ((section (".buildId_data")));
extern char buildDateStr[]   __attribute__ ((section (".buildId_data")));
extern char gitCommitIdStr[] __attribute__ ((section (".buildId_data")));

#endif /* __VERSION_H */

/************************ (C) COPYRIGHT Xtel *****END OF FILE****/


