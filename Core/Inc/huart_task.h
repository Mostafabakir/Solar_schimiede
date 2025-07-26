/*
 * huart_task.h
 *
 *  Created on: Jun 18, 2025
 *      Author: iliasalaur
 */

#ifndef INC_HUART_TASK_H_
#define INC_HUART_TASK_H_

#include <stdint.h>
#include <stdbool.h>

#define UART_COUNT     8

void huartTaskEntry(void* arg);

#endif /* INC_HUART_TASK_H_ */
