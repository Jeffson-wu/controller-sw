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

extern void Heater_PinConfig(void);
extern void heaterIrqInit(void);

extern int create_seq(long TubeId, uint16_t temp, int Nstages);
extern bool insert_state_to_seq(long TubeId, char stageChar, uint32_t time, uint16_t temp );
extern void start_tube_seq(long TubeId);
extern void stop_tube_seq(long TubeId);
extern void pause_tube_seq(void);
extern void continue_tube_seq(void);

extern char *get_tube_state(long TubeId, char *poutText);
extern char *get_system_state(char *poutText);
extern void set_log_interval(long Log_Interval);

#endif /* SEQ_H_ */
