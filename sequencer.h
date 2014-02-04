/*
 * pwm.h
 *
 *  Created on: Okt 3, 2013
 *      Author: Tommy Kristensen <tfk@xtel.dk>
 */

#ifndef SEQ_H_
#define SEQ_H_

#include "stm32f10x.h"




// The macros are defined in a more fundamental .h file (say defs.h);
#define ENUM_BEGIN(typ) enum {
#define ENUM(nam) nam
#define ENUM_END(typ) }typ;


const char *  signals_txt[] = {
  "FIRST_MSG",
  "WRITE_MODBUS_REGS",
  "WRITE_MODBUS_REGS_RES",
  "TIMER_EXPIRED",
  "START_TUBE_SEQ",
  "TUBE_TEST_SEQ",
  "NEXT_TUBE_STATE",
  "READ_MODBUS_REGS",
  "READ_MODBUS_REGS_RES",
  "START_TUBE",
  "DATA_FROM_TUBE",
  "SET_FAN",
  "SET_FAN_RES",
  "SET_COOLE_TEMP",
  "SET_COOLE_TEMP_RES",
  "SET_LID_TEMP",
  "SET_LID_TEMP_RES",
  "START_LOG",
  "END_LOG",
  "LAST_MSG"
};


const char *  tube_states[] = {
"TUBE_INIT",
"TUBE_IDLE",
"TUBE_WAIT_TEMP", /*Wait until desired temperature are reached*/
"TUBE_WAIT_TIME", /*Wait the specified time in the sequence*/
"TUBE_NOT_INITIALIZED"
};





extern void Heater_PinConfig(void);
extern void heaterIrqInit(void);

extern void start_tube_seq(long TubeId);




#endif /* SEQ_H_ */
