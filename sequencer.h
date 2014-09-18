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

typedef enum
{
  Melting,
  Annealing,
  Extension,
  Incubation,
  Pause,
  LoopStart,
  LoopEnd,
  End
}Tubestage_t;

typedef struct
{
  u16 seq_num;
  uint16_t temp;      /*Settemp in 0.1 degrees*/
  Tubestage_t stage;  /*Current stage:[M]elting(1), [A]nnealing(2), [E]xtension(3) or [I]ncubation(4) #### [P]aused #### */
  uint32_t time;      /*time in 0.1 secs*/
}stageCmd_t;

void Heater_PinConfig(void);
void heaterIrqInit(void);

int create_seq(long TubeId, uint16_t temp, int Nstages);
bool insert_state_to_seq(long TubeId, char stageChar, uint32_t time, uint16_t temp );
bool start_tube_seq(long TubeId);
bool stop_tube_seq(long TubeId);
void pause_tube_seq(void);
void continue_tube_seq(void);

char *get_tube_state(long TubeId, char *poutText);
char *get_system_state(char *poutText);
void set_log_interval(long Log_Interval);
bool tubedataQueueAdd(u8 tubeId, u16 seq_num, char state, stageCmd_t *data);
bool pause_tube_state(long TubeId);

/*extern bool pause_tube_state(long TubeId);*/

#endif /* SEQ_H_ */
