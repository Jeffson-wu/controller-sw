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
  SET_COOL_AND_LID,
  SET_COOL_AND_LID_RES,
  SET_FAN_SPEED,
  SET_FAN_RES,
  SET_COOL_TEMP,
  SET_COOL_TEMP_RES,
  SET_LID_TEMP,
  SET_LID_TEMP_RES,
  SET_LID_LOCK,
  SET_LID_LOCK_RES,
  START_DEV_LOG,
  END_DEV_LOG,
  SET_DEV_LOG_INTERVAL,
  GDI_NEW_CMD,
  BROADCAST_MODBUS,
  START_SWU,
  LAST_MSG,
  START_LID_HEATING,
  STOP_LID_HEATING,
  SET_LID_PWM,
  SET_PWM,
  SET_PWM_RES,
  nofSIG
};

typedef enum 
{
  NO_ERROR = 0,
  TIMEOUT,
  OVERRUN,
  WRONG_TEL_LENGHT
}USART_ERROR;

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
  USART_ERROR resultOk;
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
  s16 idx;
} SetCooleAndLidReq;

typedef struct
{
  s16 value;
} SetSSUpdateReq;


typedef struct
{
  u8 slave;
  u16 addr;
  u16 datasize;
  USART_ERROR resultOk;
  u8 data[1];
}ReadModbusRegsRes;

typedef struct
{
  s16 value;
  s16 idx;
} SetPWMReq;

void ResetHeaters();


#endif

