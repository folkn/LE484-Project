#include "helper.h"
#include "main.h"
#include <String.h>
#include <stdio.h>
#include "stm32f4xx_it.h"

void UART_TX(char msg[]){
	HAL_UART_Transmit_IT(&huart3, msg, strlen(msg));
	
}