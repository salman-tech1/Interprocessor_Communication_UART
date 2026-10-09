/*
 * retarget.h
 *
 *  Created on: Apr 9, 2026
 *      Author: Muhmmad Salman
 */

#ifndef RETARGET_RETARGET_H_
#define RETARGET_RETARGET_H_

#include "main.h"

/*
 * @brief : initializing the uart for I/O retargetting
 *
 */
void retarget_init(UART_HandleTypeDef *uart_x ) ;


#endif /* RETARGET_RETARGET_H_ */
