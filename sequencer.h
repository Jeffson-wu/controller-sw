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

bool start_tube_seq(long TubeId);
bool stop_tube_seq(long TubeId);

char *get_tube_state(long TubeId, char *poutText);
bool tubedataQueueAdd(u8 tubeId, u16 seq_num, char state, stageCmd_t *data);
bool pause_tube_state(long TubeId);

/*extern bool pause_tube_state(long TubeId);*/

#endif /* SEQ_H_ */
