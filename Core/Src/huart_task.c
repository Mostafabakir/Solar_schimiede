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

// Circular buffer for M-Bus packet reassembly
#define MBUS_BUFFER_SIZE 1024  // Large buffer to handle multiple packets
typedef struct {
    uint8_t buffer[MBUS_BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} MbusCircularBuffer;

static MbusCircularBuffer mbus_buffers[UART_COUNT] = {0};

// Add data to circular buffer
static void mbus_buffer_push(MbusCircularBuffer *buf, const uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        buf->buffer[buf->head] = data[i];
        buf->head = (buf->head + 1) % MBUS_BUFFER_SIZE;
        
        // If buffer is full, move tail to overwrite oldest data
        if (buf->count >= MBUS_BUFFER_SIZE) {
            buf->tail = (buf->tail + 1) % MBUS_BUFFER_SIZE;
        } else {
            buf->count++;
        }
    }
}

// Find a complete M-Bus packet in the buffer
static int mbus_find_packet(MbusCircularBuffer *buf, uint16_t *start_idx, uint16_t *packet_len) {
    if (buf->count < 6) {  // Minimum M-Bus packet size (start + L + L + start + C + stop)
        return 0;
    }
    
    // Search for M-Bus packet pattern: 0x68 L L 0x68 ... 0x16
    for (uint16_t i = 0; i < buf->count - 5; i++) {
        uint16_t idx = (buf->tail + i) % MBUS_BUFFER_SIZE;
        
        // Check for start byte
        if (buf->buffer[idx] == MBUS_START_BYTE) {
            // Get L-field (length)
            uint16_t idx_len1 = (idx + 1) % MBUS_BUFFER_SIZE;
            uint16_t idx_len2 = (idx + 2) % MBUS_BUFFER_SIZE;
            uint16_t idx_2nd_start = (idx + 3) % MBUS_BUFFER_SIZE;
            
            // Verify second start byte and matching length fields
            if (buf->buffer[idx_2nd_start] == MBUS_START_BYTE && 
                buf->buffer[idx_len1] == buf->buffer[idx_len2]) {
                
                uint8_t l_field = buf->buffer[idx_len1];
                
                // Total packet length = 4 (start,L,L,start) + L + 2 (checksum,stop)
                uint16_t total_len = 4 + l_field + 2;
                
                // Check if we have enough bytes for the complete packet
                if (i + total_len <= buf->count) {
                    // Check for stop byte
                    uint16_t stop_idx = (idx + total_len - 1) % MBUS_BUFFER_SIZE;
                    if (buf->buffer[stop_idx] == MBUS_STOP_BYTE) {
                        *start_idx = idx;
                        *packet_len = total_len;
                        return 1;
                    }
                }
            }
        }
    }
    
    return 0;
}

// Remove bytes from the buffer
static void mbus_buffer_pop(MbusCircularBuffer *buf, uint16_t len) {
    if (len > buf->count) {
        len = buf->count;
    }
    
    buf->tail = (buf->tail + len) % MBUS_BUFFER_SIZE;
    buf->count -= len;
}

// Extract a complete packet from the buffer
static int mbus_extract_packet(MbusCircularBuffer *buf, uint8_t *packet, uint16_t max_len) {
    static uint16_t ongoing_packet_idx[UART_COUNT] = {0};
    static uint16_t ongoing_packet_len[UART_COUNT] = {0};
    static uint16_t ongoing_packet_pos[UART_COUNT] = {0};
    static uint8_t ongoing_uart_id[UART_COUNT] = {0};
    
    // Get UART ID from buffer pointer (hacky but works)
    uint8_t uart_id = 0;
    for (uart_id = 0; uart_id < UART_COUNT; uart_id++) {
        if (buf == &mbus_buffers[uart_id]) {
            break;
        }
    }
    
    // If we're in the middle of streaming a packet
    if (ongoing_packet_len[uart_id] > 0) {
        // Calculate how much data we can send in this chunk
        uint16_t remaining = ongoing_packet_len[uart_id] - ongoing_packet_pos[uart_id];
        uint16_t chunk_size = (remaining > max_len) ? max_len : remaining;
        
        // Copy chunk to output buffer
        for (uint16_t i = 0; i < chunk_size; i++) {
            uint16_t idx = (ongoing_packet_idx[uart_id] + ongoing_packet_pos[uart_id] + i) % MBUS_BUFFER_SIZE;
            packet[i] = buf->buffer[idx];
        }
        
        // Update position
        ongoing_packet_pos[uart_id] += chunk_size;
        
        // If we've sent the entire packet, clean up
        if (ongoing_packet_pos[uart_id] >= ongoing_packet_len[uart_id]) {
            // Remove the packet from the buffer
            mbus_buffer_pop(buf, ongoing_packet_idx[uart_id] - buf->tail + ongoing_packet_len[uart_id]);
            
            // Reset tracking variables
            ongoing_packet_len[uart_id] = 0;
            ongoing_packet_pos[uart_id] = 0;
        }
        
        // Store UART ID for reference
        ongoing_uart_id[uart_id] = uart_id + 1;
        
        return chunk_size;
    }
    
    // Look for a new packet
    uint16_t start_idx = 0;
    uint16_t packet_len = 0;
    
    if (!mbus_find_packet(buf, &start_idx, &packet_len)) {
        return 0;
    }
    
    // Start streaming this packet
    ongoing_packet_idx[uart_id] = start_idx;
    ongoing_packet_len[uart_id] = packet_len;
    ongoing_packet_pos[uart_id] = 0;
    
    // Calculate how much data we can send in this first chunk
    uint16_t chunk_size = (packet_len > max_len) ? max_len : packet_len;
    
    // Copy chunk to output buffer
    for (uint16_t i = 0; i < chunk_size; i++) {
        uint16_t idx = (start_idx + i) % MBUS_BUFFER_SIZE;
        packet[i] = buf->buffer[idx];
    }
    
    // Update position
    ongoing_packet_pos[uart_id] += chunk_size;
    
    // If we've sent the entire packet, clean up
    if (ongoing_packet_pos[uart_id] >= packet_len) {
        // Remove the packet from the buffer
        mbus_buffer_pop(buf, start_idx - buf->tail + packet_len);
        
        // Reset tracking variables
        ongoing_packet_len[uart_id] = 0;
        ongoing_packet_pos[uart_id] = 0;
    }
    
    // Store UART ID for reference
    ongoing_uart_id[uart_id] = uart_id + 1;
    
    return chunk_size;
}

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
    // Buffer for DMA reception
    static uint8_t uart_rx_buffers[UART_COUNT][16];  // Smaller DMA buffer for faster reception
    
    // Array of UART handles
    UART_HandleTypeDef *huarts[UART_COUNT] = { 
        &huart1, &huart2, &huart3, &huart4, 
        &huart5, &huart6, &huart7, &huart8
    };
    
    // Initialize DMA reception for all UARTs
    for (int i = 0; i < UART_COUNT; i++) {
        HAL_UART_Receive_DMA(huarts[i], uart_rx_buffers[i], 16);
    }
    
    // Buffer for extracted M-Bus packets
    uint8_t packet_buffer[UART_DATA_SIZE];
    
    // Main task loop
    while (1) {
        bool data_processed = false;
        
        // Check each UART for received data
        for (int i = 0; i < UART_COUNT; i++) {
            if (uart_dma_ready[i]) {
                uart_dma_ready[i] = false;
                data_processed = true;
                
                // Add received data to circular buffer
                mbus_buffer_push(&mbus_buffers[i], uart_rx_buffers[i], 16);
                
                // Restart DMA immediately
                HAL_UART_Receive_DMA(huarts[i], uart_rx_buffers[i], 16);
                
                // Extract and forward complete M-Bus packets
                int packet_len;
                while ((packet_len = mbus_extract_packet(&mbus_buffers[i], packet_buffer, UART_DATA_SIZE)) > 0) {
                    // Create UART packet
                    UartPacket uart_packet = {0};
                    uart_packet.uart_id = i + 1;  // UART ID (1-based)
                    
                    // Copy M-Bus packet data
                    memcpy(uart_packet.data, packet_buffer, packet_len);
                    
                    // Put packet in queue for transmission to ESP32
                    osMessageQueuePut(uartQueueHandle, &uart_packet, 0, 0);
                }
            }
        }
        
        // Yield to other tasks
        osDelay(data_processed ? 2 : 5);
    }
}
