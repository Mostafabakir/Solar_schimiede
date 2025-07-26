/*
 * huart_task.c
 *
 *  Created on: Jun 18, 2025
 *      Author: iliasalaur
 */

#include "huart_task.h"
#include "protocol.h"

#include "stm32f4xx_hal.h"
#include "memory.h"
#include <cmsis_os2.h>

volatile bool uart_dma_ready[UART_COUNT] = { 0 };

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;

extern osMessageQueueId_t uartQueueHandle;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	for (int i = 0; i < UART_COUNT; i++) {
		UART_HandleTypeDef *current = NULL;
		switch (i) {
		case 0:
			current = &huart1;
			break;
		case 1:
			current = &huart2;
			break;
		case 2:
			current = &huart3;
			break;
		case 3:
			current = &huart4;
			break;
		case 4:
			current = &huart5;
			break;
		case 5:
			current = &huart6;
			break;
		case 6:
			current = &huart7;
			break;
		case 7:
			current = &huart8;
			break;
		}

		if (huart == current) {
			uart_dma_ready[i] = true;
			break;
		}
	}
}

void huartTaskEntry(void *arg) {
	static uint8_t uart_rx_buffers[UART_COUNT][UART_DATA_SIZE];

	UART_HandleTypeDef *huarts[UART_COUNT] = { &huart1, &huart2, &huart3,
			&huart4, &huart5, &huart6, &huart7, &huart8};

	for (int i = 0; i < UART_COUNT; i++) {
		HAL_UART_Receive_DMA(huarts[i], uart_rx_buffers[i], UART_DATA_SIZE);
	}

	// Start loop
	UartPacket packet;
	while (1) {
		bool data_processed = false;
		
		for (int i = 0; i < UART_COUNT; i++) {
			if (uart_dma_ready[i]) {
				uart_dma_ready[i] = false;
				data_processed = true;

				packet.uart_id = i + 1;
				memcpy(packet.data, uart_rx_buffers[i], UART_DATA_SIZE);
				
				// Use higher priority for message queue to ensure it's processed quickly
				osMessageQueuePut(uartQueueHandle, &packet, 0, 0);

				// Restart DMA immediately
				HAL_UART_Receive_DMA(huarts[i], uart_rx_buffers[i], UART_DATA_SIZE);
			}
		}

		// Use a shorter delay if no data was processed to be more responsive
		// but still yield to other tasks
		osDelay(data_processed ? 5 : 10);
	}
}
