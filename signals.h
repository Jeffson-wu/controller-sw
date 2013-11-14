#ifndef __SIGNALS_H
#define __SIGNALS_H

enum
{
  FIRST_MSG,
  WRITE_MODBUS_REGS,
  WRITE_MODBUS_REGS_RES,
  READ_MODBUS_REGS,
  READ_MODBUS_REGS_RES,
  SET_FAN,
  SET_FAN_RES,
  SET_COOLE_TEMP,
  SET_COOLE_TEMP_RES,
  SET_LID_TEMP,
  SET_LID_TEMP_RES,
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
  xQueueHandle reply; 
}ReadModbusRegsReq;

typedef struct
{
  s16 value;
} SetCooleAndLidReq;

#endif
