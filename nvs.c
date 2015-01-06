/*
 * nvs.c
 *
 *  Created on: January 2015
 *      Author: Jari Rene Jensen <rj@xtel.dk>
 *
 * Implementation file for non volatile storage
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "nvs.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define FLASH_START_ADDR    0x08000000
#define FLASH_PAGE_SIZE     0x0800        // 2kB pages 
#define FLASH_MAGIC_NUMBER  0xF0F0
#define NVS_ADDRESS         0x0803F800    // Last Sector of 256B FLASH

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/
FLASH_Status flashWritePage(uint32_t page, uint16_t length, uint16_t data[])
{
  int i;
  FLASH_Status status;

  if(FLASH_PAGE_SIZE < length){ return FLASH_ERROR_PG; }

  __disable_irq(); // disable interrupt for this operation
  FLASH_Unlock();

  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR); 

  /* Erase the FLASH page */
  status = FLASH_ErasePage(page);
  if (FLASH_COMPLETE != status) {__enable_irq(); return status; }

  /* Write the FLASH page */
  for(i = 0; i < length/2; i++)
  {
    status = FLASH_ProgramHalfWord(page + (i*2), data[i]);
    if(FLASH_COMPLETE != status) { break; }
  }
  
  FLASH_Lock(); 
  __enable_irq();

  /* Verify */
  if(FLASH_COMPLETE == status)
  {
    for(i = 0; i < length/2; i++)
    {
      if(*(__IO uint16_t *)page == data[i]) 
      {
          page += 2;
      }
      else
      {
        status = FLASH_ERROR_PG;
        return status;
      }
    }
  }
  return status;
}

/* ---------------------------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/
// Add dummy field with known bit pattern to see that there are valid data in NVS
int NVSwrite(uint16_t length, void *data)
{
  FLASH_Status status;

  if(FLASH_PAGE_SIZE >= length)
  {
    if( 1 == (length & 1) ) { length += 1; }    // Must write an even number of bytes
      /* Re-write the FLASH NVS page */
      status = flashWritePage(NVS_ADDRESS, length, data);
     if(FLASH_COMPLETE == status) 
     { 
       return 0;
     }
  }  
  return -1;
}

/* ---------------------------------------------------------------------------*/
int NVSread(uint16_t length, void *data)
{
  int ret = -1;
  int i;
  
  if(FLASH_PAGE_SIZE  >= length)
  {
    for(i = 0; i < length/2; i++)
    {
      if(0x0FFFF != *(__IO uint16_t *)(NVS_ADDRESS + (2*i))) 
      {
          ret = 0;
      }
    }
    if(0 == ret)
    {
      memcpy(data, (void*)NVS_ADDRESS, length);
    }
  }
  return ret;
}

/* ---------------------------------------------------------------------------*/


