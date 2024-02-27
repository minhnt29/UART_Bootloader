/*
 * debug_console.h
 *
 *  Created on: Feb 21, 2024
 *      Author: minh
 */

#ifndef SRC_DEBUG_CONSOLE_C_
#define SRC_DEBUG_CONSOLE_C_

#include "debug_console.h"

extern UART_HandleTypeDef huart6;
void printmsg(char *format,...)
{
	char str[80];

	/*Extract the the argument list using VA apis */
	va_list args;
	va_start(args, format);
	vsprintf(str, format,args);
	HAL_UART_Transmit(&huart6, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
	va_end(args);
}



#endif /* SRC_DEBUG_CONSOLE_C_ */
