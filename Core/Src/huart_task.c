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

// --- Packet reassembly logic ---
#define UART_REASSEMBLY_BUFFER_SIZE 256
typedef struct {
	uint8_t buffer[UART_REASSEMBLY_BUFFER_SIZE];
	uint16_t head;
	uint16_t tail;
} UartReassemblyBuffer;

static UartReassemblyBuffer uart_reassembly_buffers[UART_COUNT] = {0};

// Helper: get number of bytes in buffer
static uint16_t uart_reassembly_length(UartReassemblyBuffer *buf) {
	if (buf->head >= buf->tail) return buf->head - buf->tail;
	return UART_REASSEMBLY_BUFFER_SIZE - buf->tail + buf->head;
}

// Helper: pop bytes from buffer
static void uart_reassembly_pop(UartReassemblyBuffer *buf, uint16_t n) {
	buf->tail = (buf->tail + n) % UART_REASSEMBLY_BUFFER_SIZE;
}

// Helper: push bytes into buffer
static void uart_reassembly_push(UartReassemblyBuffer *buf, const uint8_t *data, uint16_t n) {
	for (uint16_t i = 0; i < n; ++i) {
		buf->buffer[buf->head] = data[i];
		buf->head = (buf->head + 1) % UART_REASSEMBLY_BUFFER_SIZE;
		// Overwrite oldest data if buffer is full
		if (buf->head == buf->tail) buf->tail = (buf->tail + 1) % UART_REASSEMBLY_BUFFER_SIZE;
	}
}

// Helper: find protocol packet in buffer
static int uart_find_protocol_packet(UartReassemblyBuffer *buf, uint16_t *start_idx, uint16_t *pkt_len) {
	// Protocol: 0x68 start, [len1][len2], 0x68, ...
	uint16_t len = uart_reassembly_length(buf);
	for (uint16_t i = 0; i + 5 < len; ++i) {
		uint16_t idx = (buf->tail + i) % UART_REASSEMBLY_BUFFER_SIZE;
		if (buf->buffer[idx] == 0x68) {
			// Check for second 0x68 and length
			uint16_t idx_len1 = (idx + 1) % UART_REASSEMBLY_BUFFER_SIZE;
			uint16_t idx_len2 = (idx + 2) % UART_REASSEMBLY_BUFFER_SIZE;
			uint16_t idx_2nd68 = (idx + 3) % UART_REASSEMBLY_BUFFER_SIZE;
			if (buf->buffer[idx_2nd68] == 0x68) {
				uint16_t plen = buf->buffer[idx_len1];
				if (plen == buf->buffer[idx_len2]) {
					// Total packet size: 4(header) + plen + 2 (checksum + stop)
					uint16_t total = 4 + plen + 2;
					if (i + total <= len) {
						*start_idx = idx;
						*pkt_len = total;
						return 1;
					}
				}
			}
		}
	}
	return 0;
}

void huartTaskEntry(void *arg) {
	static uint8_t uart_rx_buffers[UART_COUNT][UART_DATA_SIZE];

	UART_HandleTypeDef *huarts[UART_COUNT] = { &huart1, &huart2, &huart3,
			&huart4, &huart5, &huart6, &huart7, &huart8};

	for (int i = 0; i < UART_COUNT; i++) {
		HAL_UART_Receive_DMA(huarts[i], uart_rx_buffers[i], UART_DATA_SIZE);
	}

	// Start loop
	while (1) {
		bool data_processed = false;

		for (int i = 0; i < UART_COUNT; i++) {
			if (uart_dma_ready[i]) {
				uart_dma_ready[i] = false;
				data_processed = true;

				// Push new DMA data into reassembly buffer
				uart_reassembly_push(&uart_reassembly_buffers[i], uart_rx_buffers[i], UART_DATA_SIZE);

				// Try to extract all complete protocol packets
				while (1) {
					uint16_t start_idx = 0, pkt_len = 0;
					if (!uart_find_protocol_packet(&uart_reassembly_buffers[i], &start_idx, &pkt_len)) break;

					// Copy out the packet
					uint8_t packet_buf[UART_REASSEMBLY_BUFFER_SIZE];
					for (uint16_t j = 0; j < pkt_len; ++j) {
						uint16_t idx = (start_idx + j) % UART_REASSEMBLY_BUFFER_SIZE;
						packet_buf[j] = uart_reassembly_buffers[i].buffer[idx];
					}

					// Prepare UartPacket (send only up to UART_DATA_SIZE bytes of data)
					UartPacket packet = {0};
					packet.uart_id = i + 1;
					uint16_t copy_len = pkt_len > UART_DATA_SIZE ? UART_DATA_SIZE : pkt_len;
					memcpy(packet.data, packet_buf, copy_len);

					osMessageQueuePut(uartQueueHandle, &packet, 0, 0);

					// Remove the packet from buffer
					uart_reassembly_pop(&uart_reassembly_buffers[i], start_idx + pkt_len);
				}

				// Restart DMA immediately
				HAL_UART_Receive_DMA(huarts[i], uart_rx_buffers[i], UART_DATA_SIZE);
			}
		}

		osDelay(data_processed ? 5 : 10);
	}
}
