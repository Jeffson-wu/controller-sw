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
  TUBE_TEST_SEQ,
  NEXT_TUBE_STAGE,
  READ_MODBUS_REGS,
  READ_MODBUS_REGS_RES,
  START_TUBE,
  DATA_FROM_TUBE,
  SET_FAN,
  SET_FAN_RES,
  SET_COOLE_TEMP,
  SET_COOLE_TEMP_RES,
  SET_LID_TEMP,
  SET_LID_TEMP_RES,
  START_LOG,
  END_LOG,
  SET_LOG_INTERVAL,
  GDI_NEW_CMD,
  BROADCAST_MODBUS,
  SET_LID_LOCK,
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
  s16 value;
} SetCooleAndLidReq;

typedef struct
{
  u8 slave;
  u16 addr;
  u16 datasize;
  bool resultOk;
  u8 data[1];
}ReadModbusRegsRes;

typedef enum 
{
NO_ERROR = 0,
TIMEOUT,
OVERRUN,
WRONG_TEL_LENGHT
}USART_ERROR;

#endif
