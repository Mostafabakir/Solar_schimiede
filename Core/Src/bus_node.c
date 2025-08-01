#include "bus_node.h"
//#include "main.h"
#include <string.h>
#include <cmsis_os2.h>

#include <freertos_os2.h>

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;

SPI_HandleTypeDef slave; // Slave (upstream)
SPI_HandleTypeDef master; // Master (downstream)
extern uint16_t adcReadings[12];
extern int32_t externalADCReadings[11];
extern osMutexId_t adcBufReadMutexHandle;
extern osMessageQueueId_t uartQueueHandle;
extern osSemaphoreId_t spi_rxHandle;
extern osThreadId_t logTaskHandle;

GPIO_TypeDef *SPI2_SoftNSS_GPIO_Port = GPIOD;
uint16_t SPI2_SoftNSS_Pin = GPIO_PIN_8;

#define SPI2_NSS_LOW()  HAL_GPIO_WritePin(SPI2_SoftNSS_GPIO_Port, SPI2_SoftNSS_Pin, GPIO_PIN_RESET)
#define SPI2_NSS_HIGH() HAL_GPIO_WritePin(SPI2_SoftNSS_GPIO_Port, SPI2_SoftNSS_Pin, GPIO_PIN_SET)

static const uint8_t broadcast = 0xff;
static uint8_t myAddr = 0xFF; // Unassigned
static bool downstreamDetected = false;
static volatile bool spi_finished = 0;


static uint32_t getWaitTime(uint8_t prevNodes) {
	return prevNodes * MIN_WAIT_TIME_PER_NODE;
}

static void handleADCDataRequest() {
	// Create packet with ADC data
	Packet p = { 
		.header = { 
			.targetAddr = 0, 
			.srcAddr = myAddr, 
			.packetSize = PACKET_MAX_SIZE, 
			.packetType = PACKET_TYPE_REQUEST_DATA 
		}, 
		.data = { 0 } 
	};
	
	// Calculate CRC for header
	p.header.crc = p.header.packetType * 2 + p.header.targetAddr * 3 + p.header.srcAddr * 5;
	
	// Set request type
	p.requestDataPack.requestDataType = RQP_TYPE_ADC_READINGS;

	// Safely acquire mutex with timeout
	if(osMutexAcquire(adcBufReadMutexHandle, 10) == osOK) {
		// Copy ADC readings to packet
		memcpy(p.requestDataPack.data, adcReadings, ADC_READINGS_SIZE);
		osMutexRelease(adcBufReadMutexHandle);
	}

	// Set data size
	p.requestDataPack.dataSize = ADC_READINGS_SIZE;

	// Transmit with increased timeout to ensure completion
	HAL_SPI_Transmit(&slave, (uint8_t*)&p, PACKET_MAX_SIZE, 200);
}

static void handleExternalADCDataRequest() {
	// Create packet with external ADC data
	Packet p = { 
		.header = { 
			.targetAddr = 0, 
			.srcAddr = myAddr, 
			.packetSize = PACKET_MAX_SIZE, 
			.packetType = PACKET_TYPE_REQUEST_DATA 
		}, 
		.data = { 0 } 
	};
	
	// Calculate CRC for header
	p.header.crc = p.header.packetType * 2 + p.header.targetAddr * 3 + p.header.srcAddr * 5;
	
	// Set request type
	p.requestDataPack.requestDataType = RQP_TYPE_EXT_ADC_READINGS;

	// Copy external ADC readings to packet
	// No mutex needed as this is accessed only from one task
	memcpy(p.requestDataPack.data, externalADCReadings, EXT_ADC_READINGS_SIZE);

	// Set data size
	p.requestDataPack.dataSize = EXT_ADC_READINGS_SIZE;

	// Transmit with increased timeout to ensure completion
	HAL_SPI_Transmit(&slave, (uint8_t*)&p, PACKET_MAX_SIZE, 200);
}

static void handleSayHiRequest() {
	// Create packet for "Say Hi" response
	Packet hi = { 
		.header = { 
			.targetAddr = 0, 
			.srcAddr = myAddr, 
			.packetSize = PACKET_MAX_SIZE, 
			.packetType = PACKET_TYPE_REQUEST_DATA 
		}, 
		.data = { 0 } 
	};
	
	// Calculate CRC for header
	hi.header.crc = hi.header.packetType * 2 + hi.header.targetAddr * 3 + hi.header.srcAddr * 5;
	
	// Set request type
	hi.requestDataPack.requestDataType = RQP_TYPE_SAY_HI;

	// Message to send
	const char *msg = "Solar Schmiede STM Adapter v1.0";
	size_t dataLen = strlen(msg) + 1;
	
	// Copy message to packet
	memcpy(hi.requestDataPack.data, msg, dataLen);

	// Set data size
	hi.requestDataPack.dataSize = dataLen;

	// Transmit with increased timeout to ensure completion
	HAL_SPI_Transmit(&slave, (uint8_t*)&hi, PACKET_MAX_SIZE, 200);
}

static void handleUARTDataRequest() {
    UartPacket packet = { 0 };
    uint32_t msg_count = osMessageQueueGetCount(uartQueueHandle);
    
    // If no messages, return immediately with empty packet
    if (msg_count == 0) {
        Packet p = { 
            .header = { 
                .targetAddr = 0, 
                .srcAddr = myAddr, 
                .packetSize = PACKET_HEADER_SIZE + 3, // Minimal size for empty packet
                .packetType = PACKET_TYPE_REQUEST_DATA 
            }, 
            .data = { 0 } 
        };
        
        // Calculate CRC for header
        p.header.crc = p.header.packetType * 2 + p.header.targetAddr * 3 + p.header.srcAddr * 5;
        
        p.requestDataPack.requestDataType = RQP_TYPE_UART_DATA;
        p.requestDataPack.dataSize = 0; // No data
        p.requestDataPack.data[0] = 0xff; // Indicate no data
        
        HAL_SPI_Transmit(&slave, (uint8_t*)&p, p.header.packetSize, 100);
        return;
    }
    
    // Process message from queue
    if (osMessageQueueGet(uartQueueHandle, (void*)&packet, NULL, 0) == osOK) {
        // Determine actual data length - find the stop byte (0x16) if present
        uint16_t data_len = 0;
        for (data_len = 0; data_len < UART_DATA_SIZE; data_len++) {
            if (packet.data[data_len] == MBUS_STOP_BYTE && data_len > 5) {
                // Found stop byte, include it in the packet
                data_len++;
                break;
            }
        }
        
        // If no stop byte found, use the entire buffer
        if (data_len == UART_DATA_SIZE) {
            data_len = UART_DATA_SIZE;
        }
        
        // Create packet with appropriate size
        Packet p = { 
            .header = { 
                .targetAddr = 0, 
                .srcAddr = myAddr, 
                .packetSize = PACKET_HEADER_SIZE + 2 + data_len + 1, // Header + requestType + dataSize + data + uart_id
                .packetType = PACKET_TYPE_REQUEST_DATA 
            }, 
            .data = { 0 } 
        };
        
        // Calculate CRC for header
        p.header.crc = p.header.packetType * 2 + p.header.targetAddr * 3 + p.header.srcAddr * 5;
        
        p.requestDataPack.requestDataType = RQP_TYPE_UART_DATA;
        p.requestDataPack.dataSize = data_len + 1; // Data + UART ID
        
        // Copy UART ID and data
        p.requestDataPack.data[0] = packet.uart_id;
        memcpy(&p.requestDataPack.data[1], packet.data, data_len);
        
        // Transmit with higher timeout to ensure completion
        HAL_SPI_Transmit(&slave, (uint8_t*)&p, p.header.packetSize, 500);
    }
}

static bool detect_downstream(Packet *response, uint8_t prevNodes) {
	// Create identification packet
	Packet ident = { 
		.header = { 
			.targetAddr = broadcast, 
			.packetSize = IDENTIFY_WHOLEPACK_SIZE,
			.packetType = PACKET_TYPE_IDENTIFY, 
			.srcAddr = myAddr 
		} 
	};

	// Calculate CRC for header
	ident.header.crc = ident.header.packetType * 2 + 
	                   ident.header.targetAddr * 3 + 
	                   ident.header.srcAddr * 5;
	
	// Set identification packet data
	ident.identifyPack.identifiedDevices = 1;
	ident.identifyPack.searchDepth = prevNodes > 0 ? prevNodes - 1 : 0;

	// Start byte
	uint8_t byte = PACKET_START_BYTE;

	// First transmission - send start byte and identification packet
	SPI2_NSS_LOW();
	HAL_Delay(2); // Slightly longer delay to ensure stable signal
	
	// Send start byte
	if (HAL_SPI_Transmit(&master, &byte, 1, 100) != HAL_OK) {
		SPI2_NSS_HIGH();
		return false; // Abort if transmission fails
	}
	
	// Send identification packet
	if (HAL_SPI_Transmit(&master, (uint8_t*)&ident, PACKET_MAX_SIZE, 200) != HAL_OK) {
		SPI2_NSS_HIGH();
		return false; // Abort if transmission fails
	}
	
	// End first transmission
	SPI2_NSS_HIGH();

	// Wait for downstream nodes to process the packet
	// Use a longer wait time for more reliability
	uint32_t wait_time = getWaitTime(prevNodes) + 50;
	osDelay(wait_time);
	
	// Second transmission - receive response
	SPI2_NSS_LOW();
	HAL_Delay(2); // Slightly longer delay to ensure stable signal
	
	// Clear response buffer
	memset(response, 0, sizeof(Packet));
	
	// Receive response with longer timeout
	HAL_StatusTypeDef status = HAL_SPI_Receive(&master, (uint8_t*)response, 
	                                           PACKET_MAX_SIZE, wait_time * 2);
	
	// End second transmission
	SPI2_NSS_HIGH();

	// Check if we received a valid acknowledgment
	if (status == HAL_OK && response->header.packetType == PACKET_TYPE_IDENTIFY_ACK) {
		// Validate CRC
		uint8_t crc = response->header.packetType * 2 + 
		              response->header.targetAddr * 3 + 
		              response->header.srcAddr * 5;
		              
		return (crc == response->header.crc);
	}
	
	return false;
}

void BusNode_Init(void) {
	myAddr = 0xFF; // Unassigned
	downstreamDetected = false;

	master = hspi2;
	slave = hspi1;
}

static void forward_packet(const Packet *pkt) {
	// Buffer for response
	Packet response = {0};
	
	// Start byte
	uint8_t byte = PACKET_START_BYTE;
	
	// First transmission - send start byte and packet
	SPI2_NSS_LOW();
	HAL_Delay(2); // Slightly longer delay to ensure stable signal
	
	// Send start byte
	if (HAL_SPI_Transmit(&master, &byte, 1, 100) != HAL_OK) {
		SPI2_NSS_HIGH();
		return; // Abort if transmission fails
	}
	
	// Send packet
	if (HAL_SPI_Transmit(&master, (uint8_t*)pkt, PACKET_MAX_SIZE, 200) != HAL_OK) {
		SPI2_NSS_HIGH();
		return; // Abort if transmission fails
	}
	
	// End first transmission
	SPI2_NSS_HIGH();
	
	// Small delay between transmissions
	HAL_Delay(5);
	
	// Second transmission - receive response
	SPI2_NSS_LOW();
	HAL_Delay(2); // Slightly longer delay to ensure stable signal
	
	// Receive response with longer timeout
	HAL_SPI_Receive(&master, (uint8_t*)&response, PACKET_MAX_SIZE, 300);
	
	// End second transmission
	SPI2_NSS_HIGH();
	
	// Calculate CRC for response header
	response.header.crc = response.header.packetType * 2 + 
	                      response.header.targetAddr * 3 + 
	                      response.header.srcAddr * 5;
	
	// Forward response back to the slave
	HAL_SPI_Transmit(&slave, (uint8_t*)&response, PACKET_MAX_SIZE, 300);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi)
{
    if(hspi->Instance == slave.Instance)
    {
    	spi_finished = 1;
    	osThreadFlagsSet(logTaskHandle, 0x0001);
    }
}

void BusNode_RunLoop(void) {
    static uint32_t last_uart_check = 0;
    uint32_t current_tick = osKernelGetTickCount();
    
    // Check for UART data more frequently (every 2ms)
    // This ensures we prioritize forwarding M-Bus data to ESP32
    if (current_tick - last_uart_check >= 2) {
        last_uart_check = current_tick;
        
        // If there's data in the UART queue, process it immediately
        uint32_t msg_count = osMessageQueueGetCount(uartQueueHandle);
        if (msg_count > 0) {
            // Process all available UART messages (up to 5 at a time to avoid blocking)
            for (uint32_t i = 0; i < msg_count && i < 5; i++) {
                handleUARTDataRequest();
            }
            
            // Return early to prioritize UART data processing
            return;
        }
    }

    // Check for SPI commands from master
    uint8_t byte = 0;
    
    // Non-blocking check for start byte
    if (HAL_SPI_Receive(&slave, &byte, 1, 5) != HAL_OK || byte != PACKET_START_BYTE) {
        return; // No valid start byte, return and try again next time
    }
    
    // We received a start byte, now get the packet header first
    PacketHeader header = {0};
    
    if (HAL_SPI_Receive(&slave, (uint8_t*)&header, PACKET_HEADER_SIZE, 50) != HAL_OK) {
        return;
    }
    
    // Validate header
    if (header.packetSize < PACKET_HEADER_SIZE || header.packetSize > PACKET_MAX_SIZE) {
        return;
    }
    
    uint8_t crc = header.packetType * 2 + header.targetAddr * 3 + header.srcAddr * 5;
    if (crc != header.crc) {
        return;
    }
    
    // Now receive the rest of the packet based on the size in the header
    uint8_t data_size = header.packetSize - PACKET_HEADER_SIZE;
    uint8_t data_buffer[PACKET_DATA_SIZE] = {0};
    
    if (data_size > 0) {
        if (HAL_SPI_Receive(&slave, data_buffer, data_size, 100) != HAL_OK) {
            return;
        }
    }
    
    // Reconstruct the full packet
    Packet rx = {0};
    rx.header = header;
    memcpy(rx.data, data_buffer, data_size);

    // Handle ASSIGN_ADDR from master (only when unassigned)
    if (rx.header.packetType == PACKET_TYPE_ASSIGN_ADDR && myAddr == 0xFF) {
        myAddr = rx.data[0]; // Assigned address from master
        return;
    }

    // Handle IDENTIFY query
    if (rx.header.packetType == PACKET_TYPE_IDENTIFY && 
        (rx.header.targetAddr == myAddr || rx.header.targetAddr == broadcast)) {
        
        Packet response = {0};
        Packet ack = {
            .header = {
                .targetAddr = rx.header.srcAddr,
                .srcAddr = myAddr,
                .packetSize = IDENTIFY_WHOLEPACK_SIZE,
                .packetType = PACKET_TYPE_IDENTIFY_ACK
            }
        };
        
        // Calculate CRC for header
        ack.header.crc = ack.header.packetType * 2 + ack.header.targetAddr * 3 + ack.header.srcAddr * 5;

        ack.identifyPack.identifiedDevices = 1;
        ack.identifyPack.searchDepth = rx.identifyPack.searchDepth - 1;

        downstreamDetected = detect_downstream(&response, rx.identifyPack.searchDepth - 1);
        if (downstreamDetected) {
            ack.identifyPack.identifiedDevices += response.identifyPack.identifiedDevices;
        }

        HAL_SPI_Transmit(&slave, (uint8_t*)&ack, IDENTIFY_WHOLEPACK_SIZE, 200);
        return;
    }

    // If it's for us, handle it here
    if (rx.header.targetAddr == myAddr && myAddr != broadcast && 
        rx.header.packetType == PACKET_TYPE_REQUEST_DATA) {
        
        switch (rx.requestDataPack.requestDataType) {
        case RQP_TYPE_SAY_HI:
            handleSayHiRequest();
            break;

        case RQP_TYPE_ADC_READINGS:
            handleADCDataRequest();
            break;

        case RQP_TYPE_UART_DATA:
            handleUARTDataRequest();
            break;

        case RQP_TYPE_EXT_ADC_READINGS:
            handleExternalADCDataRequest();
            break;

        default:
            break;
        }
        return;
    } else if (downstreamDetected && myAddr != rx.header.targetAddr) {
        forward_packet(&rx);
    }
}
