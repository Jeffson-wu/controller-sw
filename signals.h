#ifndef __SIGNALS_H
#define __SIGNALS_H

enum
{
  FIRST_MSG,
  WRITE_MODBUS_REGS,
  READ_MODBUS_REGS,
  TIMER_EXPIRED,
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
  u8 data[1];
}WriteModbusRegsReq;

typedef struct
{
  u8 slave;
  u16 addr;
  u16 datasize;
}ReadModbusRegsReq;

#endif