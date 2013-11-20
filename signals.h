#ifndef __SIGNALS_H
#define __SIGNALS_H

#include "queue.h"

enum
{
  FIRST_MSG,
  WRITE_MODBUS_REGS,
  WRITE_MODBUS_REGS_RES,
  TIMER_EXPIRED,
  START_TUBE_SEQ,
  NEXT_TUBE_STATE,
  READ_MODBUS_REGS,
  READ_MODBUS_REGS_RES,
  START_TUBE,
  DATA_FROM_TUBE,
  LAST_MSG
};



typedef struct 
{
   portCHAR ucMessageID;
   portCHAR ucData[1];
}xMessage;

typedef struct
{
  u8 slave;
  u16 addr;
  u16 datasize;
  xQueueHandle reply; 
  u8 data[1];
}WriteModbusRegsReq;

typedef struct
{
  u8 slave;
  u16 addr;
  u16 datasize;
  bool resultOk;
}WriteModbusRegsRes;

typedef struct
{
  u8 slave;
  u16 addr;
  u16 datasize;
  xQueueHandle reply; 
}ReadModbusRegsReq;

typedef struct
{
  u8 slave;
  u16 addr;
  u16 datasize;
  bool resultOk;
  u8 data[1];
}ReadModbusRegsRes;

#endif
