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

//#define GDI_ON_USART3
//also in main.c

#define CHAR_ENTER 13
#define CHAR_BACKSPACE '\b'
char *command_prefix = "at@gdi:";

extern xQueueHandle CooleAndLidQueueHandle;

#ifdef GDI_ON_USART3
USART_TypeDef *uart = USART3;
#else
USART_TypeDef *uart = USART1;
#endif
u32 test_variable= 9876543;

#if 1
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
	print,
	cooleandlid,
	seq_set,
	seq_cmd,
	test_func,
	modbus_read_regs,
	modbus_write_regs,
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
  {"print", " Print the debug variable values", "at@gdi:print()",print },
  {"cooleandlid", " Set air cooler and lid temperatures and fan speed", "at@gdi:cooleandlid(idx, setpoint)",cooleandlid },
  {"seq", " Set tube seq temperatures and time", "at@gdi:seq(tube,[temp,time],..)",seq_set },
  {"seq_cmd", " Set seq start,stop, state", "at@gdi:seq_cmd(tube, cmd)",seq_cmd },
  {"test_func", " Test function call", "at@gdi:test_func(parameter1,parameter2)",test_func},		
  {"modbus_read_regs", " Read register values", "at@gdi:modbus_read_regs(slave,addr,datasize)",modbus_read_regs},
  {"modbus_write_regs", " Write register values", "at@gdi:modbus_write_regs(slave,addr,[data1,data2,..],datasize)",modbus_write_regs},
  { NULL, NULL, 0 }
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





/* Functions --------------------------------------------------------------*/

void gdi_send_result(u8 result)
{
	while(USART_GetFlagStatus(uart, USART_FLAG_TXE)==RESET);
	if (result == response_OK)
		UART_SendMsg(uart, "\r\nOK\r\n" , 6);
	else if (result == response_ERROR) 
		UART_SendMsg(uart, "\r\nERROR\r\n" , 9);
}


void gdi_send_data_response(char * response, u8 status)
{
	if(status == newline_start || status == newline_both)
	{
		USART_SendData(uart, '\r');
		while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
		USART_SendData(uart, '\n');
		while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
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
		USART_SendData(uart, '\n');
		while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
	}	
}


void gdi_send_msg_response(char * response)
{
	char i;
	char message[strlen(response)+5];
	strcpy(message, "\r\n");
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
            		if (idx < count);
			{
				*(gdi_req_func_info.parameters + idx) = pvPortMalloc(sizeof(token));
				strcpy(*(gdi_req_func_info.parameters + idx++), token);
            			token = strtok(0, ",");
            		}			
        	}
        	if (idx == count - 1)
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


int gdi_get_regwrite_values(u8 * buffer)
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
		buffer[j++] = (u8) atoi((*(gdi_req_func_info.parameters + start_pos))+1);
		for(i=start_pos + 1;i<end_pos;i++)
		{
			buffer[j++] = (u8) atoi(*(gdi_req_func_info.parameters + i));
		}
		token = *(gdi_req_func_info.parameters + end_pos);
		token[strlen(token) - 1] = '\0';
		buffer[j] =  (u8) atoi(token);
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
	u8 buffer[50];

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

		case print :
			gdi_print_number(test_variable,newline_both);
			gdi_send_data_response("OK", newline_end);
			break;	
    case cooleandlid:     
      {
        s16 setpoint;
        u8  fn_idx;
        xMessage *msg;
        SetCooleAndLidReq *p;
        msg = pvPortMalloc(sizeof(xMessage)+sizeof(SetCooleAndLidReq)+20);
				fn_idx   = (u8)  atoi(*(gdi_req_func_info.parameters + 0));
				setpoint = (s16) atoi(*(gdi_req_func_info.parameters + 1));
        if(3 > fn_idx) {
          msg->ucMessageID = SET_COOLE_TEMP;
        } else if(3 == fn_idx) {
          msg->ucMessageID = SET_LID_TEMP;
        } else if(4 == fn_idx) {
          msg->ucMessageID = SET_FAN;
        } else {
        
        }
        p = (SetCooleAndLidReq *)msg->ucData;
        p->value = setpoint;
        xQueueSend(CooleAndLidQueueHandle, &msg, portMAX_DELAY);
      }
			gdi_send_data_response("OK", newline_end);
      break;
	  case seq_set:
		  {
            uint16_t temp; /*Settemp in 0.1 degrees*/
            uint16_t time; /*time in secs*/
			long TubeId = 0;
            int i = 0;
			TubeId = (u16) atoi(*(gdi_req_func_info.parameters + 0 + i));
			i++;
			GDI_PRINTF("SET SEQ PARAMS for Tube[%d]: %d,",TubeId,gdi_req_func_info.number_of_parameters);
            create_seq(TubeId, gdi_req_func_info.number_of_parameters);
			while( i < gdi_req_func_info.number_of_parameters)
			{
  			temp = (u16) atoi(*(gdi_req_func_info.parameters + 0 + i));
  			time = (u16) atoi(*(gdi_req_func_info.parameters + 1 + i));
  			GDI_PRINTF("TEMP %d.%02dC @ TIME %d.%02dsecs ",temp/10,temp%10,time/10,time%10);
			
               i = i + 2;
			   if(insert_state_to_seq(TubeId,time,temp)!= TRUE)
			   {
				   GDI_PRINTF("ERROR INSERTING SEQ cause TUBE:%d not in idle state",TubeId);
				   break;
			   }
			}
			insert_state_to_seq(TubeId,0,0);/*Last id to indicate end of seq*/
			//start_tube_seq(TubeId);/*Start the seq*/
	  	 }
			//gdi_send_data_response("OK", newline_end);

		break;
		case seq_cmd:
			{
			  long TubeId = 0;
			  int i = 0;
			  TubeId = (u16) atoi(*(gdi_req_func_info.parameters + 0));
			  
			  //GDI_PRINTF("SEQ CMD[%s][%d]",(*(gdi_req_func_info.parameters + 1)),strlen((*(gdi_req_func_info.parameters + 1))));
			  if(!strncmp((*(gdi_req_func_info.parameters + 1)),"status",strlen("status")))
			  {
			    GDI_PRINTF("GET STATE on Tube:%d",TubeId);
				get_tube_state(TubeId);
			  }
			if(!strncmp((*(gdi_req_func_info.parameters + 1)),"start",strlen("start")))
            {
				GDI_PRINTF("START SEQ on Tube:%d",TubeId);
			    start_tube_seq(TubeId);/*Start the seq*/
			}
			if(!strncmp((*(gdi_req_func_info.parameters + 1)),"stop",strlen("stop")))
            {
				GDI_PRINTF("STOP SEQ on Tube:%d",TubeId);
			    stop_tube_seq(TubeId);
			}
		   }
			//  gdi_send_data_response("OK", newline_end);
		
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
					result = DebugModbusReadRegs(slave, addr, datasize, buffer);

					gdi_send_data_response("The register values read are : ", no_newline);
					for (i=0; i<datasize;i++)
						gdi_print_number(buffer[i], space_end);
					gdi_send_data_response("The return value is : ", newline_start);
					gdi_print_number(result, newline_end);
					gdi_send_data_response("OK", newline_end);
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
					for(i=0;i < datasize*2;i++)
						gdi_print_number(buffer[i], space_end);

					result = DebugModbusWriteRegs(slave,addr, buffer, datasize);

					gdi_send_data_response("The return value is : ", newline_start);
					gdi_print_number(result, newline_end);
					
					if (result == 0)
						gdi_send_data_response("Register Write Failed!", newline_end);
					else
						gdi_send_data_response("OK", newline_end);
				}
			}	
			break;

		case invalid_command :
			gdi_send_data_response("ERROR", newline_both);
			gdi_send_data_response("Invalid Command - type at@gdi:help()", newline_end);
			break;
	}
			
}

void gdi_init_req_func_info()
{
	gdi_req_func_info.func_type = invalid_command;
	gdi_req_func_info.number_of_parameters = 0;
	gdi_req_func_info.parameters = NULL;
}

void gdi_deinit_req_func_info()
{
	if (gdi_req_func_info.parameters)
    	{
        	int i;
        	for (i = 0; *(gdi_req_func_info.parameters + i); i++)
        	{
            		//gdi_send_data_response(*(gdi_req_func_info.parameters + i), newline_both);
            		vPortFree(*(gdi_req_func_info.parameters + i));
        	}
        	vPortFree(gdi_req_func_info.parameters);
    	}
}

void gdi_task(void *pvParameters)
{
	u8 data;
	char input_buffer[100];
	u8 index=0;

	while(1)
	{
		//USART_SendData(uart, 'U');
		//while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
		//vTaskDelay(100);
		
		//GPIOA->ODR ^= GPIO_Pin_9;	

		//GPIOE->ODR ^= GPIO_Pin_9;	


		while(USART_GetFlagStatus(uart, USART_FLAG_RXNE)==RESET);
		data = USART_ReceiveData(uart);

		if (CHAR_ENTER == data)
		{
			input_buffer[index] = '\0';
			gdi_init_req_func_info();
			gdi_parse_command(input_buffer);
			gdi_map_to_functions();
			gdi_send_data_response("", newline_end);
			gdi_deinit_req_func_info();
			index = 0;
		}
		else
		{
			if(CHAR_BACKSPACE == data)
			{
				if (index > 0)
					index = index - 1;
			}
			else
				input_buffer[index++] = data;

			while(USART_GetFlagStatus(uart, USART_FLAG_TXE)==RESET);
			USART_SendData(uart, data);
		}
 

	}

	vTaskDelete(NULL);
}



