/**
  ******************************************************************************
  * @file    gdi.c (Gnacode Debug Interface)
  * @author  Xtel
  * @version V1.0.0
  * @date    10/16/2013
  * @brief   Debug command interface
  ******************************************************************************
  * @copy
  *
  * 
  */ 

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f10x.h"
#include "stm3210c-eval.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "signals.h"
#include "logtask.h"
#include "sequencer.h"
#include "cooleandlidtask.h"
#include "signals.h"
#include <heater_reg.h>
#include "gdi.h"
#include "serial.h"
#include <ctype.h>
#include "debug.h"

/* Private feature defines ---------------------------------------------------*/
#define USE_FLOAT_REG_FEATURE
/* '\n'=0x0D=13, '\r'=0x0A=10 */
#define CHAR_ENTER '\r'
#define CHAR_BACKSPACE '\b'
char *command_prefix = "at@gdi:";
int uid;
// longest command is ?? // TODO: #### investigate this
#define INPUT_BUF_SIZE 500
#define SIZE_OF_STR_RESULT 400
#define CRASH_KEY 666
#define ASSERT_KEY 333

/* Private debug define ------------------------------------------------------*/
//#define DEBUG_USE_ECHO_AS_DEFAULT
//#define DEBUG
#ifdef DEBUG
#define GDI_PRINTF(fmt, args...)      sprintf(dbgbuf, fmt, ## args);  send_msg_on_monitor(dbgbuf);
#else
#define GDI_PRINTF(fmt, args...)    /* Don't do anything in release builds */
#endif


/* Private variables ---------------------------------------------------------*/
static char inputCmdBuf = 0;
static char input_buffer[2][INPUT_BUF_SIZE];
char str[20];

#ifdef DEBUG_USE_ECHO_AS_DEFAULT
static u8 gdiEcho = TRUE;
#else
static u8 gdiEcho = FALSE;
#endif

xSemaphoreHandle GDI_RXSemaphore = NULL;

USART_TypeDef *uart = USART1;
u32 test_variable= 9876543;

u8 DebugModbusReadRegs(u8 slave, u16 addr, u16 register_count, u8 *buffer);
u8 DebugReadRegs(xQueueHandle destQueue, u8 slave, u16 addr, u16 register_count, u8 *buffer);
bool DebugModbusWriteRegs(u8 slave, u16 addr, u8 *data, u16 register_count);
bool DebugWriteRegs(xQueueHandle destQueue, u8 slave, u16 addr, u8 *data, u16 register_count);

enum gdi_response_type
{
  response_OK,
  response_ERROR
};

enum gdi_func_type
{
  at,
  help,
  reset,
  echo,
  print,
  coolandlid,
  cool,
  lid,
  mid,
  fantemp,
  seq_cmd,
  test_func,
#ifdef USE_FLOAT_REG_FEATURE
  modbus_read_regs_float,
  modbus_write_regs_float,
#endif
  modbus_read_regs,
  modbus_write_regs,
  modbus_Synchronize_LED,
  modbus_LED,
  setpwm,
  setdac,
  getadc,
  getclmonitor,
  setprodtest,
  crash_cmd,
  bu_cmd,     // Crash cmd - used after crashes - see debug.c
  hyst,
  invalid_command
};

enum gdi_newline_type
{
  no_newline,
  newline_start,
  newline_end,
  newline_both,
  space_end
};

typedef struct 
{
  char *command_name; 
  char *func_info;
  char *func_format;
  u8 command_index;
} gdi_func_table_type;

gdi_func_table_type gdi_func_info_table[] =
{    
  {"seq_cmd",     " Set seq start, stop, state, pause, continue, log, getlog", "at@gdi:seq_cmd(tube, cmd)",seq_cmd },
  {"coolandlid",  " Set temperatures and fan speed",  "at@gdi:coolandlid(idx, setpoint)",   coolandlid },
  {"hyst", " Set hysteresis for tube heater", "at@gdi:hyst(idx, hystVal)", hyst },
  {"led",         " Set tube LED function",           "at@gdi:led(tube,fn)",        modbus_LED},
  {"synchronizeled",    " Synchronize tube LEDs",     "at@gdi:SynchronizeLED()",    modbus_Synchronize_LED},
  {"help",    " Help command",                        "at@gdi:help()",              help},
  {"reset",   " Reset M3 command",                    "at@gdi:reset()",             reset},
  {"echo",    " Echo command",                        "at@gdi:echo(<e>) e=1=> Echo on, e=0=> echo off", echo},    
  {"print",   " Print the debug variable values",     "at@gdi:print()",             print },
  {"cool",    " Set cooling temperature & start",     "at@gdi:cool(setpoint)",      cool },
  {"lid",     " Set lid temperature & start",         "at@gdi:lid(setpoint)",       lid },
  {"mid",     " Set mid temperature & start",         "at@gdi:mid(setpoint)",       mid },
  {"fantemp", " Set fan temperature & start",         "at@gdi:fantemp(setpoint)",   fantemp },
  {"test_func",         " Test function call",        "at@gdi:test_func(parameter1,parameter2)",test_func},
#ifdef USE_FLOAT_REG_FEATURE
  {"modbus_read_regs_float",  " Read register values as float",  "at@gdi:modbus_read_regs_float(slave,addr,datasize)",modbus_read_regs_float},
  {"modbus_write_regs_float", " Write register values as float", "at@gdi:modbus_write_regs_float(slave,addr,[data1,data2,..],datasize)",modbus_write_regs_float},
#endif
  {"modbus_read_regs",  " Read register values",      "at@gdi:modbus_read_regs(slave,addr,datasize)",modbus_read_regs},
  {"modbus_write_regs", " Write register values",     "at@gdi:modbus_write_regs(slave,addr,[data1,data2,..],datasize)",modbus_write_regs},
  {"setpwm",            " Set PWM [%]",               "at@gdi:setpwm(mcu, idx, pwm)", setpwm },
  {"setdac",            " Set DAC [%]",               "at@gdi:setdac(idx, dac)",    setdac },
  {"getadc",            " Get latest ADC values",     "at@gdi:getadc()",            getadc },
  {"getclmonitor",      " Get CL running variables",  "at@gdi:getclmonitor",        getclmonitor },
  {"setprodtestmode",   " Start prodtest mode ",      "at@gdi:setprodtest",         setprodtest},
  {"crash",             " Force crash",               "at@gdi:crash(key)",          crash_cmd },
  {"bu",                "",                           "",                           bu_cmd }, // cmd used only after a crash - see debug.c
  { NULL, NULL, NULL, 0 }
}; 

#ifdef USE_FLOAT_PRECICION_DOUBLE
  typedef double gdi_float;
#else
  typedef float gdi_float;
#endif

typedef struct 
{
  u8 func_type;
  char * func_info;
  u8 number_of_parameters;
  char **parameters;
  u8 return_value;
} struct_gdi_req_func_info;

struct_gdi_req_func_info gdi_req_func_info;

/* Functions -----------------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/
void (*forceHardFault)(void);

/* ---------------------------------------------------------------------------*/
int sendHyst(long TubeId, u16 hyst)
{
  xMessage *msg;
  WriteModbusRegsReq *p;
  int rtc=pdFALSE;

  msg = pvPortMalloc(sizeof(xMessage)+sizeof(WriteModbusRegsReq)+sizeof(u16));
  if(NULL != msg)
  {
    hyst = ((hyst&0xFF)<<8)|(hyst>>8);
    msg->ucMessageID = WRITE_MODBUS_REGS;
    p = (WriteModbusRegsReq *)msg->ucData;
    p->slave = TubeId;
    p->addr = SET_HYST_REG;
    memcpy(p->data, &hyst, sizeof(u16));
    p->datasize = 1; //datasize (nof regs)
    p->reply = NULL; //No reply
    rtc=xQueueSend(ModbusQueueHandle, &msg, portMAX_DELAY);     
  }
  return rtc;
}

int send_led_cmd(u16 fn, long TubeId) 
{
  xMessage *msg;
  WriteModbusRegsReq *p;

        PRINTF("LED %d %d\n\r",fn , (int)TubeId);
  msg = pvPortMalloc(sizeof(xMessage)+sizeof(WriteModbusRegsReq)+sizeof(u16));
  if(NULL != msg)
  {
    fn = ((fn&0xFF)<<8)|(fn>>8);
    msg->ucMessageID = WRITE_MODBUS_REGS;
    p = (WriteModbusRegsReq *)msg->ucData;
    p->slave = TubeId;
    p->addr = TUBE_COMMAND_REG;
    memcpy(p->data, &fn, sizeof(u16));
    p->datasize = 1; //datasize (nof regs)
    p->reply = NULL; //No reply
    return xQueueSend(ModbusQueueHandle, &msg, portMAX_DELAY);     
  }
  return pdFALSE;
}

/* ---------------------------------------------------------------------------*/
void gdi_send_result(u8 result)
{
  while(USART_GetFlagStatus(uart, USART_FLAG_TXE)==RESET);
  if (result == response_OK)
  {
    if(gdiEcho) {
      UART_SendMsg(uart, (u8 *)"\r\nOK\r\n" , 6);
    } else {
      UART_SendMsg(uart, (u8 *)"OK\r" , 6);
    }
  }
  else if (result == response_ERROR) 
  {
    if(gdiEcho) {
      UART_SendMsg(uart, (u8 *)"\r\nERROR\r\n" , 9);
    } else {
      UART_SendMsg(uart, (u8 *)"NOK\r" , 9);
    }   
  }
}

/* ---------------------------------------------------------------------------*/
void gdi_send_response_seq(void)
{
  char str[10];
  int i;
  sprintf(str, "%d,", uid);
  
  for(i=0;i<strlen(str);i++)
  {
    USART_SendData(uart, str[i]);
    while(USART_GetFlagStatus(uart, USART_FLAG_TXE)==RESET);
  }
}

/* ---------------------------------------------------------------------------*/
void gdi_send_data_response(const char * response, u8 status)
{
  if(gdiEcho) 
  {
    if(status == newline_start || status == newline_both)
    {
      USART_SendData(uart, '\r');
      while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
      USART_SendData(uart, '\n');
      while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
    }
  }
  else
  {
    gdi_send_response_seq();
  }
  for (; *response; ++response) 
  {
    USART_SendData(uart, *response);
    while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
  }
  if(status == newline_end || status == newline_both)
  {
    USART_SendData(uart, '\r');
    while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
    if(gdiEcho) 
    {
      USART_SendData(uart, '\n');
      while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
    }
  }
}

/* ---------------------------------------------------------------------------*/
void gdi_send_msg_response(char * response)
{
  char i;
  char message[strlen(response)+5];
  strcpy(message, "\0");
  strcat(message, response);
  strcat(message, "\r\n");
  for(i=0;i<strlen(message);i++)
  {
    while(USART_GetFlagStatus(uart, USART_FLAG_TXE)==RESET);
    USART_SendData(uart, *(message+i));
  }
}

/* ---------------------------------------------------------------------------*/
u8 gdi_get_command_type(char *command)
{
  int i;
  int command_len = strlen(command);
  int no_of_elements = sizeof(gdi_func_info_table)/sizeof(gdi_func_info_table[0]);
  for(i = 0; i < no_of_elements; i++)
  {
    if((command_len == strlen(gdi_func_info_table[i].command_name)) && (0 == strcmp(gdi_func_info_table[i].command_name, command)))
    {
      gdi_req_func_info.func_info = gdi_func_info_table[i].func_info;
      return gdi_func_info_table[i].command_index;
    }
  }
  return invalid_command;
}

/* ---------------------------------------------------------------------------*/
void gdi_get_func_parameters(char *param_list)
{
  u8 count = 0;
  char* tmp = param_list;
  char* last_comma = 0;
  //    GDI_PRINTF("%s/r/n",param_list);

  while (*tmp)
  {
    if (',' == *tmp)
    {
      count++;
      last_comma = tmp;
    }
    tmp++;
  }

  count += last_comma < (param_list+ strlen(param_list) - 1);
  gdi_req_func_info.number_of_parameters = count;
  count++;

  gdi_req_func_info.parameters = pvPortMalloc(sizeof(char*) * count);

  if (gdi_req_func_info.parameters)
  {
    u8 idx  = 0;
    char* token = strtok(param_list, ",");
    while (token)
    {
      if (idx < count)
      {
        *(gdi_req_func_info.parameters + idx) = pvPortMalloc(strlen(token)+1);
        if(NULL == *(gdi_req_func_info.parameters + idx)) { configASSERT(pdFALSE); } // This is a fatal error
        //GDI_PRINTF("T:%s-%d--%x", token, strlen(token)+1,(unsigned int)*(gdi_req_func_info.parameters + idx));
        strcpy(*(gdi_req_func_info.parameters + idx++), token);
        token = strtok(0, ",");
        //GDI_PRINTF("%s",token);
      }
      else
      {
        token = NULL;
        idx--;
      }  
    }
    //  if (idx == count - 1)
    *(gdi_req_func_info.parameters + idx) = NULL;
  }
}

/* ---------------------------------------------------------------------------*/
void gdi_parse_command(char * inputbuffer)
{
  int i=0, j=0;
  char command_func[50];
  char parameters[50];
  bool found_command = 0;
  char *p;

  /* change the input to lowercase characters */
  for (p = inputbuffer ; *p; ++p) *p = (char)tolower((int)*p);

  if (strncmp(inputbuffer, command_prefix,strlen(command_prefix)) != 0)
  {
    if((strlen(inputbuffer) == 2) && ( !strcmp(inputbuffer, "at") ))
    {
      gdi_req_func_info.func_type = at;
    }
    else
    {
      gdi_req_func_info.func_type = invalid_command;
    }
  }
  else
  {
    i= strlen(command_prefix);	
    while (inputbuffer[i] != '\0')
    {
      if (!found_command)
      {
      if (inputbuffer[i] == '(')
        {
          command_func[j] = '\0';
          found_command = 1;
          j = 0;
          gdi_req_func_info.func_type = gdi_get_command_type(command_func);
        }
        else 
        {
          command_func  [j++] = inputbuffer[i];
          if (inputbuffer[i+1] == '\0') {
            command_func[j] = '\0';
            gdi_req_func_info.func_type = gdi_get_command_type(command_func);
          }
        }
      }
      else if (gdi_req_func_info.func_type != invalid_command)
      {
        /* get the comma separated parameters */
        if (inputbuffer[i] == ')')
        {
          parameters[j] = '\0';
          gdi_get_func_parameters(parameters);
        }
        else
        {
          parameters[j++] = inputbuffer[i];
        }
      }
      i++;
    }
  }
}

/* ---------------------------------------------------------------------------*/
u16 test_function(u16 value)
{
  return value;
}

/* ---------------------------------------------------------------------------*/
void gdi_print_number(int number, u8 status)
{
  char digit[10];
  int index=0;

  if(status == newline_start || status == newline_both)
  {
    USART_SendData(uart, '\r');
    while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
    USART_SendData(uart, '\n');
    while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
  }

  if (number == 0)
  {
    USART_SendData(uart, '0');
    while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
  }
  else {
    while(number)
    {
      digit[index++] = (char) (((int)'0') + (number % 10));
      number /= 10;
    }

    for(; index > 0;index--)
    {
      USART_SendData(uart, digit[index-1]);
      while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
    }
  }

  if(status == space_end)
  {
    USART_SendData(uart, ' ');
    while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
  }

  if(status == newline_end || status == newline_both)
  {
    USART_SendData(uart, '\r');
    while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
    USART_SendData(uart, '\n');
    while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
  }
}

/* ---------------------------------------------------------------------------*/
void gdi_print_wrong_endian_number(int number, u8 status)
{
  gdi_print_number((number>>8 & 0x00FF) + (number<<8 &0xFF00), status);
}

/* ---------------------------------------------------------------------------*/
void my_sprintf(char *out, gdi_float input)
{
	char sign;
	int d1 = input;            // Get the integer part
	gdi_float f2 = fabs(input) - fabs(d1);     // Get fractional part
	int d2 = trunc(f2 * 10000);   // Turn into integer
	gdi_float f3 = f2 * 10000 - d2;   // Get next fractional part
	int d3 = trunc(f3 * 10000);   // Turn into integer.

	if (input < 0 && d1 == 0)
		sign = '-';
	else if (input > 0 && d1 > 0)
		sign = '+';

	sprintf (out, "%c%d.%04d%04d\n", sign, d1, d2, d3);
}

/* ---------------------------------------------------------------------------*/
#ifdef USE_FLOAT_REG_FEATURE
#if 1 //sprintf(str, "%f", number); causes a Hardfault print hex instead
void gdi_print_wrong_endian_gdi_float(gdi_float number, u8 status)
{
  gdi_float in = number;
  unsigned int *pval = (unsigned int *)&in;
  u16 * swap_p = (u16*)&in;
  u16 swap_tmp;

  if(status == newline_start || status == newline_both)
  {
    USART_SendData(uart, '\r');
    while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
    USART_SendData(uart, '\n');
    while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
  }

  if (number == 0)
  {
    USART_SendData(uart, '0');
    while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
  }
  else {

    int i;
    
    //ByteSwap for Modbus
    for(i = 0; i < (sizeof(gdi_float)/sizeof(u16)); i++)
    {
      swap_tmp = *swap_p;
      *swap_p = ( (swap_tmp>>8 & 0x00FF) + (swap_tmp<<8 &0xFF00) );
      swap_p++;
    }
#ifdef USE_FLOAT_PRECICION_DOUBLE
    //sprintf(str, "0x%08x%08x", *(pval+1), *pval);

    uint64_t tmp = (uint64_t)(*(pval+1)) << 32 | (uint32_t)(*pval);
    my_sprintf(str, *((gdi_float*)&tmp));

#else
    sprintf(str, "0x%08x", *pval);
#endif
    for(i=0;i<strlen(str);i++)
    {
      USART_SendData(uart, str[i]);
      while(USART_GetFlagStatus(uart, USART_FLAG_TXE)==RESET);
    }
  }

  if(status == space_end)
  {
    USART_SendData(uart, ' ');
    while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
  }

  if(status == newline_end || status == newline_both)
  {
    USART_SendData(uart, '\r');
    while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
    USART_SendData(uart, '\n');
    while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
  }
}
#else
void gdi_print_wrong_endian_gdi_float(gdi_float number, u8 status)
{
  gdi_float in = number;
  u16 * swap_p = (u16*)&in;
  u16 swap_tmp;

  if(status == newline_start || status == newline_both)
  {
    USART_SendData(uart, '\r');
    while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
    USART_SendData(uart, '\n');
    while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
  }

  if (number == 0)
  {
    USART_SendData(uart, '0');
    while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
  }
  else {
    char str[20];
    int i;
    //ByteSwap for Modbus
    for(i = 0; i < (sizeof(gdi_float)/2); i++)
    {
      swap_tmp = *swap_p;
      *swap_p = ( (swap_tmp>>8 & 0x00FF) + (swap_tmp<<8 &0xFF00) );
      swap_p++;
    }
    sprintf(str, "%f", number); // This causes a Hardfault
    for(i=0;i<strlen(str);i++)
    {
      USART_SendData(uart, str[i]);
      while(USART_GetFlagStatus(uart, USART_FLAG_TXE)==RESET);
    }
  }

  if(status == space_end)
  {
    USART_SendData(uart, ' ');
    while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
  }

  if(status == newline_end || status == newline_both)
  {
    USART_SendData(uart, '\r');
    while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
    USART_SendData(uart, '\n');
    while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
  }
}
#endif //1 //sprintf(str, "%f", number); causes a Hardfault print hex instead

/* ---------------------------------------------------------------------------*/
int gdi_get_regwrite_float_values(gdi_float * fbuffer)
{
  int i = 0, j = 0, start_pos = 0, end_pos = 0;
  char *token;
  u16 swap_tmp;

  u16 * swap_p = (u16*)&fbuffer[0];
  while(gdi_req_func_info.parameters[i])
  {
    if (strchr(gdi_req_func_info.parameters[i], '[') != NULL)
      start_pos= i;
    if ((strchr(gdi_req_func_info.parameters[i], ']') != NULL) && (0 != start_pos))
      end_pos=i;
      i++;
  }
  if((end_pos != 0) && (start_pos <= end_pos))
  {
    gdi_float tmp;
    char *ptr;

    if (strlen((*(gdi_req_func_info.parameters + start_pos))+1) > 15) //Otherwise atof fails, precision...
    	return 0;

    tmp = (gdi_float)strtod((*(gdi_req_func_info.parameters + start_pos))+1, &ptr);
   	//tmp = atof((*(gdi_req_func_info.parameters + start_pos))+1);
    fbuffer[j++] = tmp;
    for(i = start_pos + 1; i < end_pos; i++)
    {
      tmp = (gdi_float) atof(*(gdi_req_func_info.parameters + i));
      fbuffer[j++] = tmp;
    }
    if(end_pos > start_pos + 1)
    {
      token = *(gdi_req_func_info.parameters + end_pos);
      token[strlen(token) - 1] = '\0'; // Remove trailing ']'
      tmp = (gdi_float) atof(token);
      fbuffer[j] = tmp;
    }
    //ByteSwap for Modbus
    for(i = 0; i < (end_pos-start_pos+1)*(sizeof(gdi_float)/2); i++)
    {
      swap_tmp = *swap_p;
      *swap_p = ( (swap_tmp>>8 & 0x00FF) + (swap_tmp<<8 &0xFF00) );
      swap_p++;
    }
    return end_pos + 1;
  }

  return 0;
}
#endif // USE_FLOAT_REG_FEATURE

/* ---------------------------------------------------------------------------*/
int gdi_get_regwrite_values(u16 * buffer)
{
  int i = 0, j = 0, start_pos = 0, end_pos = 0;
  char *token;
  while(gdi_req_func_info.parameters[i])
  {
    if (strchr(gdi_req_func_info.parameters[i], '[') != NULL)
      start_pos= i;
    if ((strchr(gdi_req_func_info.parameters[i], ']') != NULL) && (0 != start_pos))
      end_pos=i;
      i++;
  }
  if((end_pos != 0) && (start_pos <= end_pos))
  {
    u16 tmp;
    tmp =  (u16) strtol((*(gdi_req_func_info.parameters + start_pos))+1, (char **)NULL, 10);
    buffer[j++] = ( (tmp>>8 & 0x00FF) + (tmp<<8 &0xFF00) );
    for(i=start_pos + 1;i<end_pos;i++)
    {
      tmp = (u16) strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10);
      buffer[j++] = ( (tmp>>8 & 0x00FF) + (tmp<<8 &0xFF00) );
    }
    token = *(gdi_req_func_info.parameters + end_pos);
    token[strlen(token) - 1] = '\0';
    tmp = (u16) strtol(token, (char **)NULL, 10);
    buffer[j] =  ( (tmp>>8 & 0x00FF) + (tmp<<8 &0xFF00) );
    return end_pos + 1;
  }
  return 0;

}

/* ---------------------------------------------------------------------------*/
long tubeId2PhysicalId(long raw_ID){
    // Convert the command LED from MainBoard to corresponding physical LED.
    // 0 is for Tubemand to make an action on ALL LED's at once.
    long TubeIdConvertionTable[] =
	{ 0,
	 16, 15, 14, 13,
	 12, 11, 10,  9,
	  1,  2,  3,  4,
	  5,  6,  7,  8};
    return TubeIdConvertionTable[raw_ID];
}

long tubeId2PhysicalId2(long raw_ID){
    // Convert the command LED from MainBoard to corresponding physical LED.
    // 0 is for Tubemand to make an action on ALL LED's at once.
    long TubeIdConvertionTable[] =
	{ 0,
	 16, 14, 15, 13,
	 12, 10, 11,  9,
	  1,  3,  2,  4,
	  5,  7,  6,  8};
    return TubeIdConvertionTable[raw_ID];
}


/* ---------------------------------------------------------------------------*/
void gdi_map_to_functions()
{
  int retvalue, i=0;
  u16 p = 9876;
  u8 slave;
  u16 addr, datasize;
#ifdef USE_FLOAT_REG_FEATURE
  union /*DBG__attribute__ ((aligned (16)))*/ {
    gdi_float gdi_float[50/(sizeof(gdi_float)/sizeof(uint16_t))];
    uint16_t  uint16[50];
  } buffer;
#else
  union {
    u16 uint16[50];
  } buffer;
#endif
  char str[SIZE_OF_STR_RESULT];
  u16 seq_num;
  char state;
  stageCmd_t data;

  uid=0; //Rease value from last cmd
  switch(gdi_req_func_info.func_type)
  {
    case at :
      gdi_send_data_response("OK", newline_both);
    break;

    case help :
      gdi_send_data_response("All commands are shown as if echo is on. (i.e. no uid parameter)", newline_end);
      gdi_send_data_response("If echo is off add uid as first parameter (increasing the total param count by 1.)", newline_end);
      while(gdi_func_info_table[i].command_name != NULL)
      {
        gdi_send_data_response(gdi_func_info_table[i].func_info, newline_start);
        gdi_send_data_response(" - ", no_newline);
        gdi_send_data_response(gdi_func_info_table[i].func_format, newline_end);
        i++;
      }
      gdi_send_data_response("OK", newline_end);
    break;

    case reset :
      if(!gdiEcho) {
        uid = (u16) strtol(*(gdi_req_func_info.parameters), (char **)NULL, 10);
      }
      //TODO: reset M3
    break;

    case echo :
    {
      u8 echoSwitch;
      echoSwitch = (u8)  strtol(*(gdi_req_func_info.parameters + 0), (char **)NULL, 10);
      gdiEcho = echoSwitch;
      gdi_send_data_response("OK", newline_end);
    }
    break;

    case print :
      gdi_print_number(test_variable,newline_both);
      gdi_send_data_response("OK", newline_end);
    break;
      
    /***************************************************************/
    /* Commands for cool and lid                                   */
    /***************************************************************/
    case coolandlid:
    {
      s16 setpoint;
      u8  fn_idx;
      xMessage *msg;        
      int result = TRUE;
      u8 paramcount;
      
      if(!gdiEcho) {
        uid = (u16) strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10);
        i++;
        paramcount = 3; // 3 or 2 params
      } else {
        paramcount = 2; // 2 or 1 params
      }
      if (gdi_req_func_info.number_of_parameters > 1) {
        fn_idx   = (u8)  strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10);
      }
      else
      {
        gdi_send_data_response("NOK invalid parameter count", newline_both);
        break;
      }
      if( ( (6 == fn_idx) && (gdi_req_func_info.number_of_parameters != (paramcount - 1) ) ) ||
          ( (6 != fn_idx) && (gdi_req_func_info.number_of_parameters != paramcount ) )  )
      {
        gdi_send_data_response("NOK invalid parameter count", newline_both);
        break;
      }

      if(6 == fn_idx) {
        int tubeNum;
        int dataSent = 0;
        /* Get log */
        if(getClLog(str, SIZE_OF_STR_RESULT))
        { /* CL log data was retrieved */
          dataSent = 1;
        }
        if (getCoolandlidHWReport(str))
        { /* HW event was retrieved */
          dataSent = 1;
        }
        for(tubeNum = 1; tubeNum < 17; tubeNum++)
        {
          if (getTubeHWReport(str, tubeNum))
          { /* Tube HW report was retrieved */
            dataSent = 1;
          }
        }
        if(dataSent)
        {
          gdi_send_data_response(str, newline_end);
        }
        else
        {
          gdi_send_data_response("OK", newline_end);
        }
      } 
      else
      {
        SetCooleAndLidReq *p;
        USART_SendData(uart, 'c');
        while(USART_GetFlagStatus(uart, USART_FLAG_TXE)==RESET);
        msg = pvPortMalloc(sizeof(xMessage)+sizeof(SetCooleAndLidReq)+20);
        if(NULL == msg) { configASSERT(pdFALSE); } // This is a fatal error
        if(msg)
        {
          i++;
          setpoint = (s16) strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10);
          if(6 > fn_idx) {
            msg->ucMessageID = SET_COOL_AND_LID;
          } else {
            result = FALSE;
            gdi_send_data_response("NOK invalid fn", newline_end);
          }
          p = (SetCooleAndLidReq *)msg->ucData;
          p->value = setpoint;
          p->idx   = fn_idx;
          xQueueSend(CoolAndLidQueueHandle, &msg, portMAX_DELAY);
        }
        if(result) { gdi_send_data_response("OK", newline_end); }
      }
    }
    break;
      
    case cool:
      {
        s16 setpoint;
        xMessage *msg;        
        int result = TRUE;
        u8 paramcount;

        if(!gdiEcho) {
          uid = (u16) strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10);
          i++;
          paramcount = 2;
        } else {
          paramcount = 1;
        }
        if (gdi_req_func_info.number_of_parameters != paramcount) {
          gdi_send_data_response("NOK invalid parameter count", newline_both);
          break;
        }
        SetCooleAndLidReq *p;
        msg = pvPortMalloc(sizeof(xMessage)+sizeof(SetCooleAndLidReq)+20);
        if(msg)
        {
          setpoint = (s16) strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10);
          if((setpoint >= -100)&&(setpoint <= 300)) {     // <= 100
            msg->ucMessageID = SET_COOL_TEMP;
          } else {
            result = FALSE;
            gdi_send_data_response("NOK invalid parameter", newline_end);
          }
          p = (SetCooleAndLidReq *)msg->ucData;
          p->value = setpoint;
          xQueueSend(CoolAndLidQueueHandle, &msg, portMAX_DELAY);
        }
        if(result) { gdi_send_data_response("OK", newline_end); }
      }
      break;

    case lid:
      {
        s16 setpoint;
        xMessage *msg;        
        int result = TRUE;
        u8 paramcount;

        if(!gdiEcho) {
          uid = (u16) strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10);
          i++;
          paramcount = 2;
        } else {
          paramcount = 1;
        }
        
        if (gdi_req_func_info.number_of_parameters != paramcount) {
          gdi_send_data_response("NOK invalid parameter count", newline_both);
          break;
        }
        SetCooleAndLidReq *p;
        msg = pvPortMalloc(sizeof(xMessage)+sizeof(SetCooleAndLidReq)+20);
        if(msg)
        {
          setpoint = (s16)strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10);
          if((setpoint >= 0) && (setpoint <= 1300))
          {
            msg->ucMessageID = SET_LID_TEMP;
          } else {
            result = FALSE;
            gdi_send_data_response("NOK invalid fn", newline_end);
          }
          p = (SetCooleAndLidReq *)msg->ucData;
          p->value = setpoint;
          p->idx   = 0;
          xQueueSend(CoolAndLidQueueHandle, &msg, portMAX_DELAY);
        }
        if(result) { gdi_send_data_response("OK", newline_end); }
      }
      break;

    case mid:
      {
        s16 setpoint;
        xMessage *msg;
        int result = TRUE;
        u8 paramcount;

        if(!gdiEcho) {
          uid = (u16) strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10);
          i++;
          paramcount = 2;
        } else {
          paramcount = 1;
        }

        if (gdi_req_func_info.number_of_parameters != paramcount) {
          gdi_send_data_response("NOK invalid parameter count", newline_both);
          break;
        }
        SetCooleAndLidReq *p;
        msg = pvPortMalloc(sizeof(xMessage)+sizeof(SetCooleAndLidReq)+20);
        if(msg)
        {
          setpoint = (s16)strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10);
          if((setpoint >= 0) && (setpoint <= 1300))
          {
            msg->ucMessageID = SET_MID_TEMP;
          } else {
            result = FALSE;
            gdi_send_data_response("NOK invalid fn", newline_end);
          }
          p = (SetCooleAndLidReq *)msg->ucData;
          p->value = setpoint;
          p->idx   = 0;
          xQueueSend(CoolAndLidQueueHandle, &msg, portMAX_DELAY);
        }
        if(result) { gdi_send_data_response("OK", newline_end); }
      }
      break;
    case fantemp:
      {
        s16 setpoint;
        xMessage *msg;        
        int result = TRUE;
        u8 paramcount;

        if(!gdiEcho) {
          uid = (u16) strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10);
          i++;
          paramcount = 2;
        } else {
          paramcount = 1;
        }
        
        if (gdi_req_func_info.number_of_parameters != paramcount) {
          gdi_send_data_response("NOK invalid parameter count", newline_both);
          break;
        }
        SetCooleAndLidReq *p;
        msg = pvPortMalloc(sizeof(xMessage)+sizeof(SetCooleAndLidReq)+20);
        if(msg)
        {
          setpoint = (s16) strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10);
          if((setpoint >= 0)&&(setpoint <= 1000)) {
            msg->ucMessageID = SET_FAN_TEMP;
          } else {
            result = FALSE;
            gdi_send_data_response("NOK invalid parameter", newline_end);
          }
          p = (SetCooleAndLidReq *)msg->ucData;
          p->value = setpoint;
          xQueueSend(CoolAndLidQueueHandle, &msg, portMAX_DELAY);
        }
        if(result) { gdi_send_data_response("OK", newline_end); }
      }
      break;
      
    case setpwm:
      {
        s16 pwm;
        s16 chn;
        s16 mcu;
        u8 result;
        xMessage *msg;        
        u8 paramcount;

        if(!gdiEcho) {
          uid = (u16) strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10);
          i++;
          paramcount = 4;
        } else {
          paramcount = 3;
        }

        if (gdi_req_func_info.number_of_parameters != paramcount) {
          gdi_send_data_response("NOK invalid parameter count", newline_both);
          break;
        }
        SetPWMReq *p;
        msg = pvPortMalloc(sizeof(xMessage)+sizeof(SetPWMReq)+20);
        if(msg)
        {
          mcu = (s16) strtol(*(gdi_req_func_info.parameters + i++), (char **)NULL, 10);
          chn = (s16) strtol(*(gdi_req_func_info.parameters + i++), (char **)NULL, 10);
          pwm = (s16) strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10);
          if((mcu < 0)||(mcu > 4)||(pwm < 0)||(pwm > 100)||(chn < 0)||(chn > 5)||( (5==chn)&&(0<mcu) ) ) {
            gdi_send_data_response("NOK invalid parameter", newline_end);
            break;
          }
          if(0 == mcu)
          {
            msg->ucMessageID = SET_PWM;
            p = (SetPWMReq *)msg->ucData;
            p->value = pwm;
            p->idx =   chn;
            xQueueSend(CoolAndLidQueueHandle, &msg, portMAX_DELAY);
            gdi_send_data_response("OK", newline_end);
          }
          else
          {
            if(0<pwm) 
            {
              u16 data = 0;
              data = SET_MANUEL_MODE;
              data = ((data<<8)&0x0ff00) + ((data>>8)&0x00ff);
              result = DebugModbusWriteRegs( (mcu-1)*4+chn , TUBE_COMMAND_REG, (u8*)&data, 1);
              data = (pwm * 32768UL) / 100;            
              data = ((data<<8)&0x0ff00) + ((data>>8)&0x00ff);
              if (NO_ERROR == result) {
                result = DebugModbusWriteRegs( (mcu-1)*4+chn , PWM_1_REG + mcu -1, (u8*)&data, 1);
              }
            }
            else
            { // Switch off
              u16 data = 0;
              data = SET_MANUEL_STOP;
              data = ((data<<8)&0x0ff00) + ((data>>8)&0x00ff);
              result = DebugModbusWriteRegs( (mcu-1)*4+chn , TUBE_COMMAND_REG, (u8*)&data, 1);
              data = 0;            
              if (NO_ERROR == result) {
                result = DebugModbusWriteRegs( (mcu-1)*4+chn , PWM_1_REG + mcu -1, (u8*)&data, 1);
              }
            }
            if(result == NO_ERROR) {
              gdi_send_data_response("OK", newline_end);
            } else {
              gdi_send_data_response("NOK MB-err", newline_end);
            }
          }
        }
      }
      break;
      
    case setdac:
      {
        s16 dac;
        s16 chn;
        xMessage *msg;        
    
        if(!gdiEcho) {
          uid = (u16) strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10);
          i++;
        }
        SetDACReq *p;
        msg = pvPortMalloc(sizeof(xMessage)+sizeof(SetDACReq)+20);
        if(msg)
        {
          chn = (s16) strtol(*(gdi_req_func_info.parameters + i++), (char **)NULL, 10);
          dac = (s16) strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10);
          if((dac >= 0)&&(dac <= 100)&&(chn >= 0)&&(chn <= 1)) {
            msg->ucMessageID = SET_DAC;
            p = (SetDACReq *)msg->ucData;
            p->value = dac;
            p->idx =   chn;
            xQueueSend(CoolAndLidQueueHandle, &msg, portMAX_DELAY);
            gdi_send_data_response("OK", newline_end);
          } else {
            gdi_send_data_response("NOK invalid parameter", newline_end);
          }
        }
      }
      break;
        
    case getadc:
      {
        if(!gdiEcho) {
          uid = (u16) strtol(*(gdi_req_func_info.parameters), (char **)NULL, 10);
        }
        getAdc(str);
        gdi_send_data_response(str, newline_end);
      }
      break;

    case getclmonitor:
      if(!gdiEcho) {
        uid = (u16) strtol(*(gdi_req_func_info.parameters), (char **)NULL, 10);
      }
      getCLMonitor(str);
      gdi_send_data_response(str, newline_end);
      break;

    /***************************************************************/
    /* Commands for sequence control                               */
    /***************************************************************/
    case seq_cmd:
      {
        long TubeId = 0, TubeIdHeat=0;
        if(!gdiEcho) {
          uid = (u16) strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10);
          i++;
        }
        TubeId = (u16) strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10);
        TubeIdHeat = tubeId2PhysicalId(TubeId);

        if((TubeIdHeat < 17)||(TubeIdHeat > 0))
        {
        TubeIdHeat = tubeId2PhysicalId(TubeId);
          // "at@gdi:seq_cmd(<uid>,<tube>,tubestart)\r"
          if(!strncmp((*(gdi_req_func_info.parameters + gdi_req_func_info.number_of_parameters - 1)),"tubestart", strlen("tubestart")))
          {
            u32 syncId=0;
            if(gdi_req_func_info.number_of_parameters ==  i+2) {
              syncId=0;
            } else {
              syncId = (u16) strtol(*(gdi_req_func_info.parameters + gdi_req_func_info.number_of_parameters - 2), (char **)NULL, 10);
            }
            GDI_PRINTF("T%ld: Start seq", TubeIdHeat);
            if(start_tube_seq((u8)TubeIdHeat, syncId))    /*Start the seq*/
            {
              gdi_send_data_response("OK", newline_end);
            }
            else
            {
              gdi_send_data_response("NOK No sequence", newline_end);
            }
          }
          // "at@gdi:seq_cmd(<uid>,<tube>,tubestop)\r"
          else if(!strncmp((*(gdi_req_func_info.parameters + i + 1)),"tubestop", strlen("tubestop")))
          {
            GDI_PRINTF("T%ld: Stop seq", TubeIdHeat);
            if(stop_tube_seq(TubeIdHeat))    /*Stop the seq*/
            {
              gdi_send_data_response("OK", newline_end);
            }
            else
            {
              gdi_send_data_response("NOK No sequence", newline_end);
            }
          }
          // "at@gdi:seq_cmd(<uid>,<tube>,tubepause)\r"
          else if(!strncmp((*(gdi_req_func_info.parameters +gdi_req_func_info.number_of_parameters-1)), "tubepause", strlen("tubepause")))
          {
            GDI_PRINTF("T%ld: pause seq", TubeIdHeat);
            if(pause_tube_state(TubeIdHeat))    /*Pause the seq*/
            {
              gdi_send_data_response("OK", newline_end);
            }
            else
            {
              gdi_send_data_response("NOK ", newline_end);
            }
          }
          // "at@gdi:seq_cmd(<uid>,<tube>,tubestatus)\r"
          else if(!strncmp((*(gdi_req_func_info.parameters + i + 1)),"tubestatus",strlen("tubestatus")))
          {
            GDI_PRINTF("T%ld: Get tubestatus.", TubeIdHeat);
            if((1 > TubeIdHeat) || (16 < TubeIdHeat)) 
            {
              gdi_send_data_response("NOK invalid tube", newline_end);
            }
            else
            {
              gdi_send_data_response(get_tube_state(TubeIdHeat, str, SIZE_OF_STR_RESULT), newline_end);
            }
          }
          // "at@gdi:seq_cmd(<uid>,<tube>,<stage number>,<temp>,<time>,<stage>,tubestage)\r"
          else if(!strncmp((*(gdi_req_func_info.parameters + gdi_req_func_info.number_of_parameters-1)),"tubestage",strlen("tubestage")))
          {
            seq_num = (u16) strtol(*(gdi_req_func_info.parameters + i + 1), (char **)NULL, 10);
            data.temp = (u16) strtol(*(gdi_req_func_info.parameters + 2 + i), (char **)NULL, 10);
            data.time = (u32) strtol(*(gdi_req_func_info.parameters + 3 + i), (char **)NULL, 10);
            state =  (**(gdi_req_func_info.parameters + 4 + i));
            if(tubedataQueueAdd(TubeIdHeat, seq_num, state, &data)== TRUE) //Insert next state into sequence
            {
              gdi_send_data_response("OK", newline_end);
            }
            else
            {
              gdi_send_data_response("NOK Queue full", newline_end);
            }
          }
          else
          {
            gdi_send_data_response("NOK SEQ_CMD not found", newline_end);
            gdi_send_data_response(input_buffer[0], newline_end);
            gdi_send_data_response(input_buffer[1], newline_end);
          }
        }
        else
        {
          GDI_PRINTF("ERROR TubeID out of range Tube:%ld", TubeIdHeat);
          gdi_send_data_response("NOK TubeID out of range", newline_end);
          break;
        }
      }
      //gdi_send_data_response("OK", newline_end);
      break;

      case test_func :
        if(!gdiEcho) {
          uid = (u16) strtol(*(gdi_req_func_info.parameters), (char **)NULL, 10);
        }
        retvalue = test_function(p);
        gdi_send_data_response("Function return value is : ", newline_start);
        gdi_print_number(retvalue,newline_end);
        gdi_send_data_response("OK", newline_end);
      break;

      /***************************************************************/
      /* Commands for modbus access                                  */
      /***************************************************************/
#ifdef USE_FLOAT_REG_FEATURE
      case modbus_read_regs_float :
      {
        u8 result;
        u8 paramcount;
        if(!gdiEcho) {
          uid = (u16) strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10);
          i++;
          paramcount = 4;
        } else {
          paramcount = 3;
        }
        if (gdi_req_func_info.number_of_parameters < paramcount)
          gdi_send_data_response("ERROR", newline_both);
        else
        {
          slave = (u8) strtol(*(gdi_req_func_info.parameters + i + 0), (char **)NULL, 10);
          addr = (u16) strtol(*(gdi_req_func_info.parameters + i + 1), (char **)NULL, 10);
          datasize = (u16) strtol(*(gdi_req_func_info.parameters + i + 2), (char **)NULL, 10);
          if(gdiEcho) {
            gdi_send_data_response("slave, addr and datasize are = ", newline_start);
            gdi_print_number(slave, space_end);
            gdi_print_number(addr, space_end);
            gdi_print_number(datasize, newline_end);
          }
          if((0 == addr) || (0 == datasize)) {
            gdi_send_data_response("ERROR", newline_both);
          } else {
            //gdi_float - adj datasize to float size
            result = DebugModbusReadRegs(slave, addr, ((datasize * sizeof(gdi_float)) / 2), (u8 *)buffer.gdi_float);
            if(NO_ERROR == result)
            {
              if(gdiEcho) { gdi_send_data_response("The register values read are : ", no_newline); }
              for (i=0; i<datasize;i++) { 
                gdi_print_wrong_endian_gdi_float(buffer.gdi_float[i], space_end); 
              }
            }
            if(gdiEcho) { 
              gdi_send_data_response("The return value is : ", newline_start);
              gdi_print_number(result, newline_end);
            }
            if(result == NO_ERROR)
            {
              gdi_send_data_response("OK", newline_end);
            }
            else
            {
              gdi_send_data_response("ERROR", newline_end);
            }

          }
        }
      }
      break;

      case modbus_write_regs_float :
      {
        u8 result;
        u8 paramcount;
        if(!gdiEcho) {
          uid = (u16) strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10);
          i++;
          paramcount = 4;
        } else { 
          paramcount = 3; 
        }
        if (gdi_req_func_info.number_of_parameters < paramcount)
          gdi_send_data_response("ERROR", newline_both);
        else
        {
          result = gdi_get_regwrite_float_values(buffer.gdi_float);
          if (result == 0) {
            gdi_send_data_response("ERROR", newline_both);
          } else {
            slave = (u8) strtol(*(gdi_req_func_info.parameters + i + 0), (char **)NULL, 10);
            addr = (u16) strtol(*(gdi_req_func_info.parameters + i + 1), (char **)NULL, 10);
            datasize = (u16) strtol(*(gdi_req_func_info.parameters + result), (char **)NULL, 10); //Do not add i as result already points to the parameter after values to write

            if(gdiEcho) {
              gdi_send_data_response("slave, addr and datasize are : ", newline_start);
              gdi_print_number(slave, space_end);
              gdi_print_number(addr, space_end);
              gdi_print_number(datasize, newline_end);
              gdi_send_data_response("The register values to write are : ", no_newline);
              for(i=0; i < datasize; i++) {
                gdi_print_wrong_endian_gdi_float(buffer.gdi_float[i], space_end);
              }
            }
            result = DebugModbusWriteRegs(slave, addr, (u8 *)buffer.gdi_float, (datasize * sizeof(gdi_float) / 2));
            if(gdiEcho) {
              gdi_send_data_response("The return value is : ", newline_start);
              gdi_print_number(result, newline_end);
            }
            if (result != NO_ERROR) {
              gdi_send_data_response("Register Write Failed!", newline_end);
            } else {
              gdi_send_data_response("OK", newline_end);
            }
          }
        }
      }
      break;
#endif

      case modbus_read_regs :
      {
        u8 result;
        u8 paramcount;
        if(!gdiEcho) {
          uid = (u16) strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10);
          i++;
          paramcount = 4;
        } else {
          paramcount = 3;
        }
        if (gdi_req_func_info.number_of_parameters < paramcount)
          gdi_send_data_response("NOK missing param", newline_both);
        else
        {
          slave = (u8) strtol(*(gdi_req_func_info.parameters + i + 0), (char **)NULL, 10);
          addr = (u16) strtol(*(gdi_req_func_info.parameters + i + 1), (char **)NULL, 10);
          datasize = (u16) strtol(*(gdi_req_func_info.parameters + i + 2), (char **)NULL, 10);

          if(gdiEcho) {
            gdi_send_data_response("slave, addr and datasize are = ", newline_start);
            gdi_print_number(slave, space_end);
            gdi_print_number(addr, space_end);
            gdi_print_number(datasize, newline_end);
          }

          if((0 == addr) || (0 == datasize)) {
            gdi_send_data_response("NOK param err", newline_both);
          } else {
            if(slave <= 16)
            { // Adresses 0 - 16 are actually on the MODBUS
              result = DebugModbusReadRegs(slave, addr, datasize, (u8 *)buffer.uint16);
            }
            else if(slave <= 21)
            { // Adresses 17 - 20 are mapped to cool and lid, addr is reg, datasize is reg count, buffer is wrong endian
              //result = coolLidReadRegs(slave, addr, datasize, (u16 *)buffer.uint16);
              result = DebugReadRegs(CoolAndLidQueueHandle, slave, addr, datasize, (u8 *)buffer.uint16);
            }
            else
            {
              result = -1;
            }
            if(NO_ERROR == result)
            {
              if(gdiEcho) { gdi_send_data_response("The register values read are : ", no_newline); }
              for (i=0; i<datasize;i++) { 
                gdi_print_wrong_endian_number(buffer.uint16[i], space_end); 
              }
            }
            if(gdiEcho) {
              gdi_send_data_response("The return value is : ", newline_start);
              gdi_print_number(result, newline_end);
            }
            if(result == NO_ERROR)
            {
              gdi_send_data_response("OK", newline_end);
            }
            else
            {
              gdi_send_data_response("NOK ModbusReadRegs failed", newline_end);
            }

          }
        }
      }
      break;

      case modbus_write_regs :
      {
        u8 result;
        u8 paramcount;
        if(!gdiEcho) {
          uid = (u16) strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10);
          i++;
          paramcount = 4;
        } else { 
          paramcount = 3; 
        }
        if (gdi_req_func_info.number_of_parameters < paramcount)
          gdi_send_data_response("NOK missing param", newline_both);
        else
        {
          result = gdi_get_regwrite_values(buffer.uint16);
          if (result == 0) {
            gdi_send_data_response("ERROR", newline_both);
          } else {
            slave = (u8) strtol(*(gdi_req_func_info.parameters + i + 0), (char **)NULL, 10);
            addr = (u16) strtol(*(gdi_req_func_info.parameters + i + 1), (char **)NULL, 10);
            datasize = (u16) strtol(*(gdi_req_func_info.parameters + result), (char **)NULL, 10); //Do not add i as result already points to the parameter after values to write

            if(gdiEcho) {
              gdi_send_data_response("slave, addr and datasize are : ", newline_start);
              gdi_print_number(slave, space_end);
              gdi_print_number(addr, space_end);
              gdi_print_number(datasize, newline_end);
              gdi_send_data_response("The register values to write are : ", no_newline);
              for(i=0; i < datasize; i++) {
                gdi_print_wrong_endian_number(buffer.uint16[i], space_end);
              }
            }
            if(slave <= 16)
            { // Adresses 0 - 16 are actually on the MODBUS
              result = DebugModbusWriteRegs(slave, addr, (u8 *)buffer.uint16, datasize);
            }
            else if(slave <= 20)
            { // Adresses 17 - 19 are mapped to cool and lid, addr is reg, datasize is reg count, buffer is wrong endian
              //result = coolLidWriteRegs(slave, addr, (u16 *)buffer.uint16, datasize);
              result = DebugWriteRegs(CoolAndLidQueueHandle, slave, addr, (u8 *)buffer.uint16, datasize);
            }
            else if(slave <= 21)
            { // Adresses 20 is mapped to sequencer, addr is reg, datasize is reg count, buffer is wrong endian
              //result = seqWriteRegs(slave, addr, (u16 *)buffer.uint16, datasize);
              result = DebugWriteRegs(TubeSequencerQueueHandle, slave, addr, (u8 *)buffer.uint16, datasize);
            }
            if(gdiEcho) {
              gdi_send_data_response("The return value is : ", newline_start);
              gdi_print_number(result, newline_end);
            }
            if (result != NO_ERROR) {
              gdi_send_data_response("NOK Register Write Failed!", newline_end);
            } else {
              gdi_send_data_response("OK", newline_end);
            }
          }
        }
      }
      break;

    case hyst:
      {
        u16 hystVal;

        i++; // Goto param
        hystVal=atoi(*(gdi_req_func_info.parameters + i));
//        if (gdi_req_func_info.number_of_parameters > 0) {
//          hystVal   = (u16)strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10);
//        }
        PRINTF("Hyst %d %d\n\r",i , hystVal);
        if (hystVal>=15 && hystVal<=500) {
           hystVal=(hystVal*3333)/10;
           sendHyst(1,hystVal);
           sendHyst(5,hystVal);
           sendHyst(9,hystVal);
           sendHyst(13,hystVal);
        }
      }
      break;

    case modbus_Synchronize_LED:
      //void WriteTubeHeaterReg(u8 tube '=0', u16 reg '=0', u16 *data 'no data', u16 datasize '=0')
      {
        xMessage *msg;
        WriteModbusRegsReq *p;

        if(!gdiEcho) {
          uid = (u16) strtol(*(gdi_req_func_info.parameters), (char **)NULL, 10);
        }
        msg=pvPortMalloc(sizeof(xMessage)+sizeof(WriteModbusRegsReq)/*+datasize*sizeof(u16)*/);
        if(msg)
        {
          //*data=((*data&0xFF)<<8)|(*data>>8);
          msg->ucMessageID=BROADCAST_MODBUS;
          p=(WriteModbusRegsReq *)msg->ucData;
          p->slave=0; //not used for broadcast
          p->addr=0;
          //No data memcpy(p->data, data, datasize*sizeof(u16));
          p->datasize=0; //datasize;
          p->reply=NULL; //No reply
          xQueueSend(ModbusQueueHandle, &msg, portMAX_DELAY);      
          gdi_send_data_response("OK", newline_end);
        }
      }
      break;

    case modbus_LED:
      //  {"led", " Set tube LED function", "at@gdi:led(fn)",modbus_LED},
      {
        long TubeId = 0;
        int cmd;
        u16 fn;
        int result = 1;
        
        if(!gdiEcho) {
          uid = (u16) strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10);
          i++;
        }

        TubeId = (u16) strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10);
        i++;
        cmd = (u16) strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10);
PRINTF("LED: %d %d\n", (int)TubeId, (int)cmd);
        if (cmd==SET_LED_SLOW_BLINK || cmd==SET_LED_FAST_BLINK || cmd==SET_LED_PULSE)
            TubeId=tubeId2PhysicalId2(TubeId);
        else
            TubeId=tubeId2PhysicalId(TubeId);
        i++;
        if( (0<=TubeId) && (16>=TubeId) )
        {
          switch(cmd)
          {
            case 0:
              fn=SET_LED_OFF;
              break;
            case 1:
              fn=SET_LED_ON;
              break;
            case 2:
              fn=SET_LED_SLOW_BLINK;
              break;
            case 3:
              fn=SET_LED_FAST_BLINK;
              break;
            case 4:
              fn=SET_LED_PULSE;
              break;
            default:
              result=0;
          }
          if(0==TubeId)
          {
            while(TubeId < 16)
            {
              TubeId++;
              if(pdTRUE != send_led_cmd(fn, TubeId)) { result = 0; }
            }
          }
          else
          {
            if(pdTRUE != send_led_cmd(fn, TubeId)) { result = 0; }
          }
        }
        if(result) { 
          gdi_send_data_response("OK", newline_end);
        } else {
          gdi_send_data_response("NOK Param out of range", newline_end);
        }
      }
      break;

    /***************************************************************/
    /* Misc commands                                               */
    /***************************************************************/
    case setprodtest:
      {
        u8 paramcount;
        if(!gdiEcho) {
          uid = (u16) strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10);
          i++;
          paramcount = 2;
        } else { 
          paramcount = 1; 
        }
        if (gdi_req_func_info.number_of_parameters != paramcount)
        {
          gdi_send_data_response("NOK wrong param count", newline_both);
        }
        else
        {
          if(1 == (u16) strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10) )
          {
            // Enable periodic pinging of all M0s 
            setPingMode(1);
            // Enable production test mode
            setProdTestMode(1);
            PRINTF("Prodtest mode");
          }
          else
          {
            // Disable periodic pinging of all M0s
            setPingMode(0);
            // Disable production test mode
            setProdTestMode(0);
            PRINTF("Normal mode");
          }
          gdi_send_data_response("OK", newline_end);
        }
      }
      break;

    case crash_cmd:
    {
      if(!gdiEcho) {
        uid = (u16) strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10);
        i++;
      }
      if(ASSERT_KEY == (u16) strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10) )
      {
        configASSERT(1);
      }
      if(CRASH_KEY == (u16) strtol(*(gdi_req_func_info.parameters + i), (char **)NULL, 10) )
      {
        forceHardFault();
      }
    }
    break; // Will not reach this point

    case bu_cmd:
    {
      if(gdiEcho) {
        gdi_send_data_response("BU recieved.", newline_both);
      }
    }
    break;

    case invalid_command :
    {
      if(gdiEcho) {
        gdi_send_data_response("ERROR", newline_both);
        gdi_send_data_response("Invalid Command - type at@gdi:help()", newline_end);
      }else{
        gdi_send_data_response("NOK Invalid Command", newline_end);
        gdi_send_data_response(input_buffer[0], newline_end);
        gdi_send_data_response(input_buffer[1], newline_end);
      }
    }
    break;
  }
}

/* ---------------------------------------------------------------------------*/
void recieveCMD(void)
{
  static u8 index=0;
  u8 data;
/*
 * xSemaphoreGiveFromISR() will set *pxHigherPriorityTaskWoken to pdTRUE 
 * if giving the semaphoree caused a task to unblock, and the unblocked 
 * task has a priority higher than the currently running task.
 */
  portBASE_TYPE xHigherPriorityTaskWoken;

  while(USART_GetFlagStatus(uart, USART_FLAG_RXNE)==RESET);
  data = USART_ReceiveData(uart);

  if (CHAR_ENTER == data)/*Enter detected interpret CMD */
  {
    input_buffer[(int)inputCmdBuf][index] = '\0';
    /*unlock mutex to continue in GDI task*/
    xSemaphoreGiveFromISR( GDI_RXSemaphore, &xHigherPriorityTaskWoken);
    
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    inputCmdBuf = !inputCmdBuf;
    index = 0;
  }
  else
  {
    if(CHAR_BACKSPACE == data) /* support backspace in case of typing errors */
    {
      if (index > 0)
      index = index - 1;
    }
    else
    {
      if((' ' <= data) && ('~' >= data))
      {
        input_buffer[(int)inputCmdBuf][index++] = data;
        if(INPUT_BUF_SIZE <= index)
        {
          gdi_send_msg_response("NOK INPUT BUFFER OVERRUN !!!");
          index = 0;
        }
      }
    }
    if(gdiEcho) {
      while(USART_GetFlagStatus(uart, USART_FLAG_TXE)==RESET);
      USART_SendData(uart, data);
    }
  }
  // vTraceUserEvent(4);
  //portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

/* ---------------------------------------------------------------------------*/
void gdi_init_req_func_info()
{
  gdi_req_func_info.func_type = invalid_command;
  gdi_req_func_info.number_of_parameters = 0;
  gdi_req_func_info.parameters = NULL;
}

/* ---------------------------------------------------------------------------*/
void gdi_deinit_req_func_info()
{
  if (gdi_req_func_info.parameters)
  {
 //   GDI_PRINTF("GDI deinit params:%x",gdi_req_func_info.number_of_parameters);
    int i;
    for (i = 0; *(gdi_req_func_info.parameters + i); i++)
    {
      //gdi_send_data_response(*(gdi_req_func_info.parameters + i), newline_both);
      vPortFree(*(gdi_req_func_info.parameters + i));
   //   GDI_PRINTF("%d",i);
    }
   // GDI_PRINTF("free struct %x",gdi_req_func_info.parameters);
    vPortFree(gdi_req_func_info.parameters);
  }
}

/* ---------------------------------------------------------------------------*/
void gdi_init()
{
  UART_Init(uart, recieveCMD);
}

/* ---------------------------------------------------------------------------*/
void gdi_task(void *pvParameters)
{
  GDI_RXSemaphore = xSemaphoreCreateMutex();
  vQueueAddToRegistry(GDI_RXSemaphore,(char *)"GDIRx sem");
  xSemaphoreTake( GDI_RXSemaphore, portMAX_DELAY );
  gdi_init(); /*Setup debug uart, This must be after the GDI_RXSemaphore is instantiated */
#ifdef DEBUG_USE_ECHO_AS_DEFAULT
  PRINTF("!! Echo=1 Not for use with Linux box !!");
#endif
  while(1)
  {
    /*wait for mutex*/
    xSemaphoreTake( GDI_RXSemaphore, portMAX_DELAY );
    gdi_init_req_func_info();
    gdi_parse_command(input_buffer[!inputCmdBuf]);
    gdi_map_to_functions();
    if(gdiEcho) {
      gdi_send_data_response("", newline_end);
    }
    gdi_deinit_req_func_info();
  }
  configASSERT(0);

  vTaskDelete(NULL);
}

/* ---------------------------------------------------------------------------*/
u8 DebugModbusReadRegs(u8 slave, u16 addr, u16 register_count, u8 *buffer)
{
  xMessage *msgin;
  xMessage *msgout;
  ReadModbusRegsReq *pReq;
  msgout=pvPortMalloc(sizeof(xMessage)+sizeof(ReadModbusRegsReq));
  if(NULL == msgout)
  {
    GDI_PRINTF("Malloc failed! DebugModbusReadRegs Tube %d, Reg %d",slave,addr);
    return -1;
  }
  else
  {
    msgout->ucMessageID = READ_MODBUS_REGS;
    pReq = (ReadModbusRegsReq *)msgout->ucData;
    pReq->slave = slave;
    pReq->addr = addr;
    pReq->datasize = register_count;
    pReq->reply = GDIQueueHandle;
    assert_param(xQueueSend(ModbusQueueHandle, &msgout, portMAX_DELAY)== pdPASS);
    if( xQueueReceive(GDIQueueHandle , &msgin, (TickType_t)0 ) == pdPASS ) // Empty the queue. do not wait if the queue is empty
    {
       vPortFree(msgin);
    }
    /* wait for queue msg */
    if( xQueueReceive( GDIQueueHandle, &msgin, (TickType_t)500 ) == pdPASS ) // How long to wait. MB queue size is 10, MB timeout is 100ms -> more than a sec??
    {
      ReadModbusRegsRes *preg;
      u16 *modbus_data = (u16*)buffer;
      int i=0;
      preg = (ReadModbusRegsRes *)msgin->ucData;
      for(i = 0; i < (preg->datasize); i++)
      {
        modbus_data[i] =(((u16)(preg->data[i*2])) | (preg->data[(i*2)+1]<<8));
      }
      vPortFree(msgin);
    }
    else
    {
      return -2;
    }
  }
  return 0;
}

/* ---------------------------------------------------------------------------*/
bool DebugModbusWriteRegs(u8 slave, u16 addr, u8 *data, u16 register_count)
{
  xMessage *msgout;
  WriteModbusRegsReq *pReq;
  msgout=pvPortMalloc(sizeof(xMessage)+sizeof(WriteModbusRegsReq)+(register_count*sizeof(u16)));
  if(NULL == msgout)
  {
    GDI_PRINTF("Malloc failed! DebugModbusWriteRegs Tube %d, Reg %d",slave,addr);
    return -1;
  }
  else
  {
    msgout->ucMessageID = WRITE_MODBUS_REGS;
    pReq = (WriteModbusRegsReq *)msgout->ucData;
    pReq->slave = slave;
    pReq->addr = addr;
    pReq->datasize = register_count;
    pReq->reply = NULL; //we do not want the reply
    memcpy(pReq->data, data, register_count*sizeof(u16));
    assert_param(xQueueSend(ModbusQueueHandle, &msgout, portMAX_DELAY)== pdPASS);
  }
  return 0;
}

/* ---------------------------------------------------------------------------*/
u8 DebugReadRegs(xQueueHandle destQueue, u8 slave, u16 addr, u16 register_count, u8 *buffer)
{
  xMessage *msgin;
  xMessage *msgout;
  ReadModbusRegsReq *pReq;
  msgout=pvPortMalloc(sizeof(xMessage)+sizeof(ReadModbusRegsReq));
  if(NULL == msgout)
  {
    GDI_PRINTF("Malloc failed! DebugReadRegs: Slave %d, Reg %d",slave,addr);
    return -1;
  }
  else
  {
    //GDI_PRINTF("DebugReadRegs: Slave %d, Reg %d",slave,addr);
    msgout->ucMessageID = READ_MODBUS_REGS;
    pReq = (ReadModbusRegsReq *)msgout->ucData;
    pReq->slave = slave;
    pReq->addr = addr;
    pReq->datasize = register_count;
    pReq->reply = GDIQueueHandle;
    assert_param(xQueueSend(destQueue, &msgout, portMAX_DELAY)== pdPASS);
    if( xQueueReceive(GDIQueueHandle , &msgin, (TickType_t)0 ) == pdPASS ) // Empty the queue. do not wait if the queue is empty
    {
       vPortFree(msgin);
    }
    /* wait for queue msg */
    if( xQueueReceive( GDIQueueHandle, &msgin, (TickType_t)500 ) == pdPASS ) // How long to wait. MB queue size is 10, MB timeout is 100ms -> more than a sec??
    {
      ReadModbusRegsRes *preg;
      u16 *modbus_data = (u16*)buffer;
      int i=0;
      
      preg = (ReadModbusRegsRes *)msgin->ucData;
      for(i = 0; i < (preg->datasize); i++)
      {
        modbus_data[i] =(((u16)(preg->data[i*2])) | (preg->data[(i*2)+1]<<8));
      }
      vPortFree(msgin);
    }
    else
    {
      return -2;
    }
  }
  return 0;
}

/* ---------------------------------------------------------------------------*/
bool DebugWriteRegs(xQueueHandle destQueue, u8 slave, u16 addr, u8 *data, u16 register_count)
{
  xMessage *msgout;
  WriteModbusRegsReq *pReq;
  msgout=pvPortMalloc(sizeof(xMessage)+sizeof(WriteModbusRegsReq)+register_count*sizeof(u16));
  if(NULL == msgout)
  {
    GDI_PRINTF("Malloc failed! DebugWriteRegs Slave %d, Reg %d",slave,addr);
    return -1;
  }
  else
  {
    //GDI_PRINTF("DebugWriteRegs Slave %d, Reg %d",slave,addr);
    msgout->ucMessageID = WRITE_MODBUS_REGS;
    pReq = (WriteModbusRegsReq *)msgout->ucData;
    pReq->slave = slave;
    pReq->addr = addr;
    pReq->datasize = register_count;
    pReq->reply = NULL; //we do not want the reply
    memcpy(pReq->data, data, register_count*sizeof(u16));
    assert_param(xQueueSend(destQueue, &msgout, portMAX_DELAY)== pdPASS);
  }
  return 0;
}

/* ---------------------------------------------------------------------------*/


