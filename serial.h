
#include "stm32f10x_usart.h"

extern int USART3_intitalized;

void UART_SendMsg(USART_TypeDef *uart, u8 *buffer, int len);
void UART_Init(USART_TypeDef *uart, void (*recvCallback)());

