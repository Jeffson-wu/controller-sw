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
#include "swupdatetask.h"
#include "signals.h"
#include "../heater-sw/heater_reg.h"

#define MAIN_IF_REV2

//#define GDI_ON_USART3
//also in main.c

/* '\n'=0x0D=13, '\r'=0x0A=10 */
#define CHAR_ENTER '\r'
#define CHAR_BACKSPACE '\b'
char *command_prefix = "at@gdi:";
int uid;
// longest command is "at@gdi:seq_cmd(<tube>,<PauseTemp>,<stage1>,<Temp1>,<Time1>,<stage2>,<Temp2>,<Time2>,.,.,.,.,.,., )\n"
// wich is 25 + 36 * 35 (for 35 cycles of 3 stages) 
#define INPUT_BUF_SIZE 1300
  u8 data;
char a = 0;
char input_buffer[2][INPUT_BUF_SIZE];

static u8 gdiEcho = FALSE;
//static u8 gdiEcho = TRUE;

const char *  tube_st[] = {
"Melting",
"Annealing",
"Extension",
"Incubation",
"Pause",
"LoopStart",
"LoopEnd",
"End",
};

xSemaphoreHandle GDI_RXSemaphore = NULL;

extern xQueueHandle ModbusQueueHandle;
extern xQueueHandle CoolAndLidQueueHandle;
extern xQueueHandle SwuQueueHandle;

#ifdef GDI_ON_USART3
USART_TypeDef *uart = USART3;
#else
USART_TypeDef *uart = USART1;
#endif
u32 test_variable= 9876543;

#if 0
#define GDI_PRINTF(fmt, args...)      sprintf(buf, fmt, ## args);  gdi_send_msg_response(buf);
#else
#define GDI_PRINTF(fmt, args...)    /* Don't do anything in release builds */
#endif

char buf[300]; /*buffer for debug printf*/


u8 DebugModbusReadRegs(u8 slave, u16 addr, u16 datasize, u8 *buffer);
bool DebugModbusWriteRegs(u8 slave, u16 addr, u8 *data, u16 datasize);


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
	lidpwm,
	fan,
	seq,
	seq_set,
	seq_cmd,
	test_func,
	modbus_read_regs,
	modbus_write_regs,
	modbus_Synchronize_LED,
	modbus_LED,
	swupdate,
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
  {"help", " Help command", "at@gdi:help()",help},		
  {"reset", " Reset M3 command", "at@gdi:reset()",reset},		
  {"echo", " Echo command", "at@gdi:echo(<e>) e=1=> Echo on, e=0=> echo off",echo},    
  {"print", " Print the debug variable values", "at@gdi:print()",print },
  {"coolandlid", " Set temperatures and fan speed", "at@gdi:coolandlid(idx, setpoint)",coolandlid },
  {"cool", " Set cooling temperature", "at@gdi:cool(idx, setpoint)",cool },
  {"lid",   " Set lid temperature",    "at@gdi:lid(idx, setpoint)",lid },
  {"lidpwm",   " Set lid pwm value",    "at@gdi:lid(idx, pwm)",lidpwm },
  {"fan",   " Set fan speed % ",       "at@gdi:fan(idx, setpoint)",fan },
  {"seq",   " Set tube seq temperatures and time", "at@gdi:seq(tube,[temp,time],..)",seq },
  {"seq_set", " Set tube seq: stage,temperatures,time", "at@gdi:seq_set(tube,pauseTemp,[stage,temp,time],..)",seq_set },
  {"seq_cmd", " Set seq start, stop, state, pause, continue, log, getlog", "at@gdi:seq_cmd(tube, cmd)",seq_cmd },
  {"test_func", " Test function call", "at@gdi:test_func(parameter1,parameter2)",test_func},		
  {"modbus_read_regs", " Read register values", "at@gdi:modbus_read_regs(slave,addr,datasize)",modbus_read_regs},
  {"modbus_write_regs", " Write register values", "at@gdi:modbus_write_regs(slave,addr,[data1,data2,..],datasize)",modbus_write_regs},
  {"synchronizeled", " Synchronize tube LEDs", "at@gdi:SynchronizeLED()",modbus_Synchronize_LED},
  {"led", " Set tube LED function", "at@gdi:led(tube,fn)",modbus_LED},
  {"swupdate", " Start M0 SW update", "at@gdi:swupdate(tube,fn)",swupdate},
  { NULL, NULL, NULL, 0 }
}; 

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
int send_led_cmd(u16 fn, long TubeId) 
{
  xMessage *msg;
  WriteModbusRegsReq *p;

  msg=pvPortMalloc(sizeof(xMessage)+sizeof(WriteModbusRegsReq)+sizeof(u16));
  if(NULL != msg)
  {
    fn=((fn&0xFF)<<8)|(fn>>8);
    msg->ucMessageID=WRITE_MODBUS_REGS;
    p=(WriteModbusRegsReq *)msg->ucData;
    p->slave=TubeId;
    p->addr=TUBE_COMMAND_REG;
    memcpy(p->data, &fn, sizeof(u16));
    p->datasize=2; //datasize;
    p->reply=NULL; //No reply
    return xQueueSend(ModbusQueueHandle, &msg, portMAX_DELAY);     
  }
}

/* ---------------------------------------------------------------------------*/
void gdi_send_result(u8 result)
{
	while(USART_GetFlagStatus(uart, USART_FLAG_TXE)==RESET);
	if (result == response_OK)
	{
    if(gdiEcho) {
		  UART_SendMsg(uart, "\r\nOK\r\n" , 6);
    } else {
		  UART_SendMsg(uart, "OK\r" , 6);
    }
	}
	else if (result == response_ERROR) 
  {
    if(gdiEcho) {
 		  UART_SendMsg(uart, "\r\nERROR\r\n" , 9);
    } else {
 		  UART_SendMsg(uart, "NOK\r" , 9);
    }   
	}
}

void gdi_send_response_seq(void)
{
  char str[10];
  int i;
  sprintf(str, "<%d>,", uid);
  
  for(i=0;i<strlen(str);i++)
  {
    USART_SendData(uart, str[i]);
    while(USART_GetFlagStatus(uart, USART_FLAG_TXE)==RESET);
  }
}

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
//     	UART_SendMsg(uart, message , strlen(message));
		USART_SendData(uart, *(message+i));
//     	while(USART_GetFlagStatus(uart, USART_FLAG_TXE)==RESET);
	}
}


void gdi_send_msg_on_monitor(char * response)
{
	char i=0;
   int len = strlen(response)+5;
	char message[strlen(response)+5];
	strcpy(message, "\0");
	strcat(message, response);
	strcat(message, "\r\n");
	for(i=0;i<strlen(message);i++)
	{
     	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE)==RESET);
//     	UART_SendMsg(uart, message , strlen(message));
		USART_SendData(USART3, *(message+i));
//     	while(USART_GetFlagStatus(uart, USART_FLAG_TXE)==RESET);
	}


  
     while(i<len)
     {
       while(USART_GetFlagStatus(uart, USART_FLAG_TXE)==RESET);
       USART_SendData(USART3,*(message+i));
       i++;
     }
}


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
 //       GDI_PRINTF("T:%s-%d--%x/r/n",token,strlen(token)+1,*(gdi_req_func_info.parameters + idx));
				strcpy(*(gdi_req_func_info.parameters + idx++), token);
            			token = strtok(0, ",");
   //               GDI_PRINTF("%s",token);
            		}else
                {
                token = NULL;
                idx--;
                }  
        	}
        //	if (idx == count - 1)
			*(gdi_req_func_info.parameters + idx) = NULL;
    	}

}


void gdi_parse_command(char * inputbuffer)
{
	int i=0, j=0;
	char command_func[50];
	char parameters[50];
	bool found_command = 0;
	char *p;

	/* change the input to lowercase characters */
	for (p = inputbuffer ; *p; ++p) *p = tolower(*p);

	if (strncmp(inputbuffer, command_prefix,strlen(command_prefix)) != 0)
		if((strlen(inputbuffer) == 2) && ( !strcmp(inputbuffer, "at") ))
			gdi_req_func_info.func_type = at;	
		else
			gdi_req_func_info.func_type = invalid_command;
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
					j=0;
					gdi_req_func_info.func_type = gdi_get_command_type(command_func);
				}
				else 
				{
					command_func[j++] = inputbuffer[i];
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
					parameters[j++] = inputbuffer[i];
			}				
			i++;				
		}
	}	
	
}


u16 test_function(u16 value)
{
	return value;
}

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

void gdi_print_wrong_endian_number(int number, u8 status)
{
  gdi_print_number((number>>8 & 0x00FF) + (number<<8 &0xFF00), status);
}


int gdi_get_regwrite_values(u16 * buffer)
{
	int i=0,j=0,start_pos=0,end_pos=0;
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
    tmp =  (u16) atoi((*(gdi_req_func_info.parameters + start_pos))+1);
		buffer[j++] = ( (tmp>>8 & 0x00FF) + (tmp<<8 &0xFF00) );
		for(i=start_pos + 1;i<end_pos;i++)
		{
      tmp = (u16) atoi(*(gdi_req_func_info.parameters + i));
			buffer[j++] = ( (tmp>>8 & 0x00FF) + (tmp<<8 &0xFF00) );
		}
		token = *(gdi_req_func_info.parameters + end_pos);
		token[strlen(token) - 1] = '\0';
    tmp = (u16) atoi(token);
		buffer[j] =  ( (tmp>>8 & 0x00FF) + (tmp<<8 &0xFF00) );
		return end_pos + 1;
	}
	return 0;
	
}


void gdi_map_to_functions()
{
	int retvalue,	i=0;
	u16 p = 9876;
	u8 result, slave;
	u16 addr, datasize;
	u16 buffer[50];
  char str[200];
  u16 seq_num,seq_id;
  				uint16_t temp; /*Settemp in 0.1 degrees*/
				uint32_t time; /*time in secs*/
        char state;
                     stageCmd_t data;
            u8 nParams;

  uid=0; //Rease value from last cmd
	switch(gdi_req_func_info.func_type)
	{
		case at :
			gdi_send_data_response("OK", newline_both);
			break;

		case help :
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
      ResetHeaters();
      //Reset_Handler();
      break;

    case echo :
      {
        u8 echoSwitch;
        echoSwitch = (u8)  atoi(*(gdi_req_func_info.parameters + 0));
        gdiEcho = echoSwitch;
  			gdi_send_data_response("OK", newline_end);
      }
      break;

		case print :
			gdi_print_number(test_variable,newline_both);
			gdi_send_data_response("OK", newline_end);
			break;	
      
    case coolandlid:
      {
        s16 setpoint;
        u8  fn_idx;
        xMessage *msg;        
        int result = TRUE;

        if(!gdiEcho) {
          uid = (u16) atoi(*(gdi_req_func_info.parameters + i));
          i++;
        }
        SetCooleAndLidReq *p;
        msg = pvPortMalloc(sizeof(xMessage)+sizeof(SetCooleAndLidReq)+20);
        if(msg)
        {
  				fn_idx   = (u8)  atoi(*(gdi_req_func_info.parameters + i));
          i++;
  				setpoint = (s16) atoi(*(gdi_req_func_info.parameters + i));
          if(6 > fn_idx) {
            msg->ucMessageID = SET_COOLE_AND_LID;
          } else {
            result = FALSE;
            gdi_send_data_response("NOK invalid fn", newline_end);
          }
          p = (SetCooleAndLidReq *)msg->ucData;
          p->value = setpoint;
          p->idx   = fn_idx;
          xQueueSend(CoolAndLidQueueHandle, &msg, portMAX_DELAY);
        }
      }
			if(result) { gdi_send_data_response("OK", newline_end); }
      break;
      
    case cool:
      {
        s16 setpoint;
        xMessage *msg;        
        int result = TRUE;

        if(!gdiEcho) {
          uid = (u16) atoi(*(gdi_req_func_info.parameters + i));
          i++;
        }
        SetCooleAndLidReq *p;
        msg = pvPortMalloc(sizeof(xMessage)+sizeof(SetCooleAndLidReq)+20);
        if(msg)
        {
  				setpoint = (s16) atoi(*(gdi_req_func_info.parameters + i));
          if((setpoint >= -100)&&(setpoint <= 300)) {			// <= 100
            msg->ucMessageID = SET_COOL_TEMP;
          } else {
            result = FALSE;
            gdi_send_data_response("NOK invalid parameter", newline_end);
          }
          p = (SetCooleAndLidReq *)msg->ucData;
          p->value = setpoint;
          xQueueSend(CoolAndLidQueueHandle, &msg, portMAX_DELAY);
        }
      }
			if(result) { gdi_send_data_response("OK", newline_end); }
      break;

    case lid:
      {
        s16 setpoint;
        u8 fn_idx;
        xMessage *msg;        
        int result = TRUE;

        if(!gdiEcho) {
          uid = (u16) atoi(*(gdi_req_func_info.parameters + i));
          i++;
        }
        SetCooleAndLidReq *p;
        msg = pvPortMalloc(sizeof(xMessage)+sizeof(SetCooleAndLidReq)+20);
        if(msg)
        {

  				fn_idx   = (u8)atoi(*(gdi_req_func_info.parameters + i));
          i++;
  				setpoint = (s16)atoi(*(gdi_req_func_info.parameters + i));
          if((3>fn_idx) && (setpoint >= 0) && (setpoint <= 1200))
          {
            msg->ucMessageID = SET_LID_TEMP;
          } else {
            result = FALSE;
            gdi_send_data_response("NOK invalid fn", newline_end);
          }
          p = (SetCooleAndLidReq *)msg->ucData;
          p->value = setpoint;
          p->idx   = fn_idx;
          xQueueSend(CoolAndLidQueueHandle, &msg, portMAX_DELAY);
        }
      }
			if(result) { gdi_send_data_response("OK", newline_end); }
      break;

    case lidpwm:
      {
        s16 pwm;
        u8 fn_idx;
        xMessage *msg;        
        int result = TRUE;

        if(!gdiEcho) {
          uid = (u16) atoi(*(gdi_req_func_info.parameters + i));
          i++;
        }
        SetCooleAndLidReq *p;
        msg = pvPortMalloc(sizeof(xMessage)+sizeof(SetCooleAndLidReq)+20);
        if(msg)
        {

  				fn_idx   = (u8)atoi(*(gdi_req_func_info.parameters + i));
          i++;
  				pwm = (s16)atoi(*(gdi_req_func_info.parameters + i));
          if((3>fn_idx) && (pwm >= 0) && (pwm <= 65535))
          {
            msg->ucMessageID = SET_LID_PWM;
          } else {
            result = FALSE;
            gdi_send_data_response("NOK invalid fn", newline_end);
          }
          p = (SetCooleAndLidReq *)msg->ucData;
          p->value = pwm;
          p->idx   = fn_idx;
          xQueueSend(CoolAndLidQueueHandle, &msg, portMAX_DELAY);
        }
      }
			if(result) { gdi_send_data_response("OK", newline_end); }
      break;


    case fan:
      {
        s16 setpoint;
        xMessage *msg;        
        int result = TRUE;

        if(!gdiEcho) {
          uid = (u16) atoi(*(gdi_req_func_info.parameters + i));
          i++;
        }
        SetCooleAndLidReq *p;
        msg = pvPortMalloc(sizeof(xMessage)+sizeof(SetCooleAndLidReq)+20);
        if(msg)
        {
  				setpoint = (s16) atoi(*(gdi_req_func_info.parameters + i));
          if((setpoint >= 0)&&(setpoint <= 100)) {
            msg->ucMessageID = SET_FAN_SPEED;
          } else {
            result = FALSE;
            gdi_send_data_response("NOK invalid parameter", newline_end);
          }
          p = (SetCooleAndLidReq *)msg->ucData;
          p->value = setpoint;
          xQueueSend(CoolAndLidQueueHandle, &msg, portMAX_DELAY);
        }
      }
			if(result) { gdi_send_data_response("OK", newline_end); }
      break;

	  case seq:
		  {//Manual sequence setting

				long TubeId = 0;
				int i = 0;
				TubeId = (u16) atoi(*(gdi_req_func_info.parameters + 0 + i));
				i++;
				GDI_PRINTF("SET SEQ PARAMS for Tube[%d]: %d,",TubeId,gdi_req_func_info.number_of_parameters);
            if((int)NULL != create_seq(TubeId, 0, gdi_req_func_info.number_of_parameters))
            {
               while( i < gdi_req_func_info.number_of_parameters)
               {
                 temp = (u16) atoi(*(gdi_req_func_info.parameters + 0 + i));
                 time = (u32) atoi(*(gdi_req_func_info.parameters + 1 + i));
                 GDI_PRINTF("%d:TEMP %d.%02dC @ TIME %d.%02dsecs ",(i+1)/2,temp/10,temp%10,time/10,time%10);
                 
                 i = i + 2;
                 if(insert_state_to_seq(TubeId,'e',time,temp)!= TRUE) //Pause is possible only after E, thus set all at E.
                 {
                   GDI_PRINTF("ERROR INSERTING SEQ cause TUBE:%d not in idle state",TubeId);
                   break;
                 }
               }
               insert_state_to_seq(TubeId,'\0',0,0);/*Last id to indicate end of seq*/
            }else
			{
        GDI_PRINTF("ERROR no memory for sequence");
      }
    }
    //gdi_send_data_response("OK", newline_end);
    break;
    case seq_set:
		{ // Automated Sequence setting
      uint16_t pauseTemp; /*PauseTemp in 0.1 degrees*/
      char stageChar;     /*Sequence stage*/
      uint16_t temp;      /*Settemp in 0.1 degrees*/
      uint32_t time;      /*time in secs*/
			long TubeId = 0;
      int i = 0;
      int result = TRUE;
      
      if(!gdiEcho) {
        uid = (u16) atoi(*(gdi_req_func_info.parameters + i));
        i++;
      }
			TubeId = (u16) atoi(*(gdi_req_func_info.parameters + i));
			i++;
			pauseTemp = (u16) atoi(*(gdi_req_func_info.parameters + i));
			i++;
      // nof stages = (number_of_parameters - i) / 3
      // Starting with i params not being stages. Each stage is 3 parameters
      if((int)NULL != create_seq(TubeId, pauseTemp, (gdi_req_func_info.number_of_parameters - i) / 3))
      {
        while( i < gdi_req_func_info.number_of_parameters)
        {
          stageChar = **(gdi_req_func_info.parameters + 0 + i);
          temp = (u16) atoi(*(gdi_req_func_info.parameters + 1 + i));
          time = (u32) atoi(*(gdi_req_func_info.parameters + 2 + i));
          i = i + 3;
          if(insert_state_to_seq(TubeId,stageChar,time,temp)!= TRUE)
          {
            gdi_send_data_response("NOK Tube not in idle state", newline_end);
            result = FALSE;
            break;
          }
        }
        insert_state_to_seq(TubeId,'\0',0,0);/*Last id to indicate end of seq*/
        if(result) { gdi_send_data_response("OK", newline_end); }
      }else
	    {
        gdi_send_data_response("NOK Insufficient memory for sequence", newline_end);
		  }
 	  }
    break;

    case seq_cmd:
  	  {
        long TubeId = 0;
        int i = 0;
        
       // gdi_send_data_response("SEQ_CMD", newline_end);
        
        
     //   GDI_PRINTF("DATA:%s - %s",(*(gdi_req_func_info.parameters + 0)),(*(gdi_req_func_info.parameters + 1)));
        if(!gdiEcho) {
          uid = (u16) atoi(*(gdi_req_func_info.parameters + i));
          i++;
        }
        if(!strncmp((*(gdi_req_func_info.parameters + i)),"getlog",strlen("getlog")))
        {
          gdi_send_data_response("", no_newline); //Send uid and line feed if appropriate
          sendLog();          
          USART_SendData(uart, '\r');
          while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
        }
        else if(!strncmp((*(gdi_req_func_info.parameters + i)),"pause",strlen("pause")))
        {
          pause_tube_seq();
          gdi_send_data_response("OK", newline_end);
        }
        else if(!strncmp((*(gdi_req_func_info.parameters + i)),"continue",strlen("continue")))
        {
          GDI_PRINTF("CONTINUE SEQ");
          continue_tube_seq();
          gdi_send_data_response("OK", newline_end);
        }
        else 
        {
          TubeId = (u16) atoi(*(gdi_req_func_info.parameters + i));
          if((TubeId < 17)||(TubeId > 0))
          {
            //GDI_PRINTF("SEQ CMD[%s][%d]",(*(gdi_req_func_info.parameters + 1)),strlen((*(gdi_req_func_info.parameters + 1))));
            if(!strncmp((*(gdi_req_func_info.parameters + i + 1)),"status",strlen("status")))
            {
            if(0 == TubeId) 
              {
                gdi_send_data_response(get_system_state(str), newline_end);
              }
              else
              {
                GDI_PRINTF("GET STATE on Tube:%d",TubeId);
                gdi_send_data_response(get_tube_state(TubeId, str), newline_end);
                
              }
            }
            else if(!strncmp((*(gdi_req_func_info.parameters + i + 1)),"start",strlen("start")))
            {
              GDI_PRINTF("START SEQ on Tube:%d",TubeId);
              if(start_tube_seq(TubeId))    /*Start the seq*/
              {
                gdi_send_data_response("OK", newline_end);
              }
              else
              {
                gdi_send_data_response("NOK No sequence", newline_end);
              }
            }
            else if(!strncmp((*(gdi_req_func_info.parameters + i + 1)),"stop",strlen("stop")))
            {
              GDI_PRINTF("STOP SEQ on Tube:%d",TubeId);
              if(stop_tube_seq(TubeId))    /*Stop the seq*/
              {
                gdi_send_data_response("OK", newline_end);
              }
              else
              {
                gdi_send_data_response("NOK Not running", newline_end);
              }                
            }
            else if(!strncmp((*(gdi_req_func_info.parameters + i + 1)),"log",strlen("log")))
            {
              if(0 == TubeId) {
                GDI_PRINTF("LOG off");
              } else {
                GDI_PRINTF("LOG Interval:%d ms",TubeId);
              }
              set_log_interval(TubeId);
              gdi_send_data_response("OK", newline_end);
            }
#ifdef MAIN_IF_REV2            
           else if(!strncmp((*(gdi_req_func_info.parameters + i + 1)),"tubestart",strlen("tubestart")))
            {
              GDI_PRINTF("tubestart on Tube:%d",TubeId);
              if(start_tube_seq(TubeId))    /*Start the seq*/
              {
                gdi_send_data_response("OK", newline_end);
              }
              else
              {
                gdi_send_data_response("NOK No sequence", newline_end);
              }
            }
           else if(!strncmp((*(gdi_req_func_info.parameters + i + 1)),"tubestop",strlen("tubestop")))
            {
              GDI_PRINTF("tubestop on Tube:%d",TubeId);
              if(stop_tube_seq(TubeId))    /*Start the seq*/
              {
                gdi_send_data_response("OK", newline_end);
              }
              else
              {
                gdi_send_data_response("NOK No sequence", newline_end);
              }
            }
           else if(!strncmp((*(gdi_req_func_info.parameters +gdi_req_func_info.number_of_parameters-1)),"tubepause",strlen("tubepause")))
            {
              GDI_PRINTF("tubepause on Tube:%d",TubeId);
              if(pause_tube_state(TubeId))    /*Start the seq*/
              {
                gdi_send_data_response("OK", newline_end);
              }
              else
              {
                gdi_send_data_response("NOK No sequence", newline_end);
              }
            }
           else if(!strncmp((*(gdi_req_func_info.parameters + i + 1)),"tubestatus",strlen("tubestatus")))
            {
              GDI_PRINTF("tubestatus on Tube:%d",TubeId);
              if(0 == TubeId) 
                 {
                   gdi_send_data_response(get_system_state(str), newline_end);
                 }
                 else
                 {
                   GDI_PRINTF("GET STATE on Tube:%d",TubeId);
                   gdi_send_data_response(get_tube_state(TubeId, str), newline_end);
                 }

            }
           else if(!strncmp((*(gdi_req_func_info.parameters + gdi_req_func_info.number_of_parameters-1)),"tubestage",strlen("tubestage")))
            {
            // GDI_PRINTF("tubestage[%s-%s-%s-%s-%s-%s]#%d",(*(gdi_req_func_info.parameters + 0)),(*(gdi_req_func_info.parameters + 1)),(*(gdi_req_func_info.parameters + 2)),(*(gdi_req_func_info.parameters + 3)),(*(gdi_req_func_info.parameters + 4)),(*(gdi_req_func_info.parameters + 5)),(*(gdi_req_func_info.number_of_parameters)));


i=i-1;
                        nParams = (gdi_req_func_info.number_of_parameters);
                        i=i+1;
                        seq_num = (u16) atoi(*(gdi_req_func_info.parameters + i + 1));
                        data.temp = (u16) atoi(*(gdi_req_func_info.parameters + 2 + i));
                        data.time = (u32) atoi(*(gdi_req_func_info.parameters + 3 + i));
                        state =  (**(gdi_req_func_info.parameters + 4 + i));
                  //      GDI_PRINTF("%c-%c",(*(gdi_req_func_info.parameters + 4 + i),state));
//                 GDI_PRINTF("Tube:%d:TEMP %d.%02dC @ TIME %d.%02dsecs STATE:%d SEQ_ID:%d",TubeId,data.temp/10,data.temp%10,data.time/10,data.time%10,data.stage,seq_num);
                  sprintf(buf, "Tube:%d:TEMP %d.%02dC @ TIME %d.%02dsecs STATE:%d SEQ_ID:%d",TubeId,data.temp/10,data.temp%10,data.time/10,data.time%10,data.stage,seq_num); 
                  gdi_send_msg_on_monitor(buf);

               #if 1   
              
                 if(tubedataQueueAdd(TubeId,seq_num,state, &data)== TRUE) //Insert next state into sequence
                 {
                  gdi_send_data_response("OK", newline_end);
                 }
                 else
                 {
                  gdi_send_data_response("NOK Queue full", newline_end);
                 }
              #endif   
            }else
              {
                gdi_send_data_response("NOK SEQ_CMD not found", newline_end);
                i = 0;
                char temp[10];
                sprintf(buf, "CMD NOT FOUND-%d ",gdi_req_func_info.number_of_parameters );

                while(gdi_req_func_info.number_of_parameters > i)
                  {
                sprintf(temp, "%s-",*(gdi_req_func_info.parameters + i));
                strcat(buf,temp);
                i++;
                }
                gdi_send_msg_on_monitor(buf);

              }
            #endif
           
          }
          else
          {
            GDI_PRINTF("ERROR TubeID out of range Tube:%d",TubeId);
            gdi_send_data_response("NOK TubeID out of range", newline_end);
            break;
          }
        }
      }
      //gdi_send_data_response("OK", newline_end);
      break;
		
		case test_func :
			retvalue = test_function(p);
			gdi_send_data_response("Function return value is : ", newline_start);
			gdi_print_number(retvalue,newline_end);
			gdi_send_data_response("OK", newline_end);
			break;	
			
		case modbus_read_regs :
			if (gdi_req_func_info.number_of_parameters < 3)
				gdi_send_data_response("ERROR", newline_both);
			else
			{
				slave = (u8) atoi(*(gdi_req_func_info.parameters + 0));
				addr = (u16) atoi(*(gdi_req_func_info.parameters + 1));
				datasize = (u16) atoi(*(gdi_req_func_info.parameters + 2)); 

				gdi_send_data_response("slave, addr and datasize are = ", newline_start);
				gdi_print_number(slave, space_end);
				gdi_print_number(addr, space_end);
				gdi_print_number(datasize, newline_end);
				
				if((0 == addr) || (0 == datasize))
					gdi_send_data_response("ERROR", newline_both);
				else
				{
					result = DebugModbusReadRegs(slave, addr, datasize, (u8 *)buffer);

          if(NO_ERROR == result)
          {
            gdi_send_data_response("The register values read are : ", no_newline);
  					for (i=0; i<datasize;i++) { 
              gdi_print_wrong_endian_number(buffer[i], space_end); 
            }
          }
					gdi_send_data_response("The return value is : ", newline_start);
					gdi_print_number(result, newline_end);
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
			break;

		case modbus_write_regs :
			if (gdi_req_func_info.number_of_parameters < 3)
				gdi_send_data_response("ERROR", newline_both);
			else
			{
				result = gdi_get_regwrite_values(buffer);	
				if (result == 0)
					gdi_send_data_response("ERROR", newline_both);
				else
				{
					slave = (u8) atoi(*(gdi_req_func_info.parameters + 0));
					addr = (u16) atoi(*(gdi_req_func_info.parameters + 1));
					datasize = (u16) atoi(*(gdi_req_func_info.parameters + result)); 

					gdi_send_data_response("slave, addr and datasize are : ", newline_start);
					gdi_print_number(slave, space_end);
					gdi_print_number(addr, space_end);
					gdi_print_number(datasize, newline_end);
					gdi_send_data_response("The register values to write are : ", no_newline);
					for(i=0;i < datasize;i++) {
						gdi_print_wrong_endian_number(buffer[i], space_end);
          }
					result = DebugModbusWriteRegs(slave,addr, (u8 *)buffer, datasize);

					gdi_send_data_response("The return value is : ", newline_start);
					gdi_print_number(result, newline_end);
					
					if (result != NO_ERROR)
						gdi_send_data_response("Register Write Failed!", newline_end);
					else
						gdi_send_data_response("OK", newline_end);
				}
			}	
			break;

    case modbus_Synchronize_LED:
      //void WriteTubeHeaterReg(u8 tube '=0', u16 reg '=0', u16 *data 'no data', u16 datasize '=0')
      {
        xMessage *msg;
        WriteModbusRegsReq *p;

        if(!gdiEcho) {
          uid = (u16) atoi(*(gdi_req_func_info.parameters));
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
        xMessage *msg;
        WriteModbusRegsReq *p;
        long TubeId = 0;
        int i = 0;
        int cmd;
        u16 fn;
        int result = 1;
        
        if(!gdiEcho) {
          uid = (u16) atoi(*(gdi_req_func_info.parameters + i));
          i++;
        }
        TubeId = (u16) atoi(*(gdi_req_func_info.parameters + i));
        i++;
        cmd = (u16) atoi(*(gdi_req_func_info.parameters + i));
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
            while(TubeId < 17)
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

    case swupdate:
      {
        xMessage *msg;
        SetSSUpdateReq *p;
        long TubeId = 0;
        int i = 0;
        int cmd;
        u16 fn;
        int result = 1;
        
        if(!gdiEcho) {
          uid = (u16) atoi(*(gdi_req_func_info.parameters + i));
          i++;
        }
        //TubeId = (u16) atoi(*(gdi_req_func_info.parameters + i));
        //i++;
        cmd = (u16) atoi(*(gdi_req_func_info.parameters + i));
        i++;

        if(1 == cmd)
        {
          result = SWU_start_task();
          if(result) { 
            gdi_send_data_response("OK", newline_end);
          } else {
            gdi_send_data_response("NOK failed", newline_end);
          }
        }
        else
        {
          msg = pvPortMalloc(sizeof(xMessage)+sizeof(WriteModbusRegsReq)/*+datasize*sizeof(u16)*/);
          if(msg)
          {
            msg->ucMessageID = START_SWU;
            p = (SetSSUpdateReq *)msg->ucData;
            p->value = cmd;
            xQueueSend(SwuQueueHandle, &msg, portMAX_DELAY);
            if(result) { 
              gdi_send_data_response("OK", newline_end);
            } else {
              gdi_send_data_response("NOK Param out of range", newline_end);
            }
          }
          else
          {          
            gdi_send_data_response("NOK malloc failed", newline_end);
          }
        }
      }
      break;

		case invalid_command :
      if(gdiEcho) {
			  gdi_send_data_response("ERROR", newline_both);
			  gdi_send_data_response("Invalid Command - type at@gdi:help()", newline_end);
      }else{
			  gdi_send_data_response("NOK Invalid Command", newline_end);
      }
			break;
	}
			
}

void recieveCMD(void)
{
  static u8 index=0;
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
    input_buffer[a][index] = '\0';
    /*unlock mutex to continue in GDI task*/
    xSemaphoreGiveFromISR( GDI_RXSemaphore, &xHigherPriorityTaskWoken);
    
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    a = !a;
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
        input_buffer[a][index++] = data;
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

void gdi_init_req_func_info()
{
  gdi_req_func_info.func_type = invalid_command;
  gdi_req_func_info.number_of_parameters = 0;
  gdi_req_func_info.parameters = NULL;
}

void gdi_deinit_req_func_info()
{
extern void* xStart;
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
void gdi_init()
{
  UART_Init(uart, recieveCMD);
  UART_Init(USART3,recieveCMD); /*Only for monitoring no RX*/

}
void gdi_task(void *pvParameters)
{
   GDI_RXSemaphore = xSemaphoreCreateMutex();
   xSemaphoreTake( GDI_RXSemaphore, portMAX_DELAY );
  gdi_send_msg_response("HEATER CONTROLLER BOOTING ...");

  while(1)
  {
    /*wait for mutex*/
    xSemaphoreTake( GDI_RXSemaphore, portMAX_DELAY );
    gdi_init_req_func_info();
    gdi_parse_command(input_buffer[!a]);
    gdi_map_to_functions();
    if(gdiEcho) {
      gdi_send_data_response("", newline_end);
    }
    gdi_deinit_req_func_info();
  }
  configASSERT(0);

  vTaskDelete(NULL);
}



