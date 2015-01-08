/*
 * nvs.h
 *
 *  Created on: January 2015
 *      Author: Jari Rene Jensen <rj@xtel.dk>
 *
 * Interface definition file for non volatile storage
 */

#ifndef NVS_H_
#define NVS_H_

#include <stm32f10x_flash.h>

/* Exported functions ---------------------------------------------------------*/
/*  */
int NVSwrite(uint16_t length, void *data);
/*  */
int NVSread(uint16_t length, void *data);

#endif /* NVS_H_ */

