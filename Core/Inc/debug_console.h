/*
 * debug_console.h
 *
 *  Created on: Feb 21, 2024
 *      Author: minh
 */

#ifndef INC_DEBUG_CONSOLE_H_
#define INC_DEBUG_CONSOLE_H_

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"

void printmsg(char *format,...);



#endif /* INC_DEBUG_CONSOLE_H_ */
