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
		Packet p = { .header = { .targetAddr = 0, .srcAddr = myAddr, .packetSize =
		UART_WHOLEPACK_SIZE, .packetType = PACKET_TYPE_REQUEST_DATA, }, .data = { 0 } };
		p.requestDataPack.requestDataType = RQP_TYPE_UART_DATA;
		p.requestDataPack.data[0] = 0xff; // Indicate no data
		p.requestDataPack.dataSize = UART_PACKET_SIZE;
		
		HAL_SPI_Transmit(&slave, (uint8_t*) &p, PACKET_MAX_SIZE, 100);
		return;
	}
	
	// Process message from queue
	if (osMessageQueueGet(uartQueueHandle, (void*)&packet, NULL, 0) == osOK) {
		Packet p = { .header = { .targetAddr = 0, .srcAddr = myAddr, .packetSize =
		UART_WHOLEPACK_SIZE, .packetType = PACKET_TYPE_REQUEST_DATA, }, .data = { 0 } };
		p.requestDataPack.requestDataType = RQP_TYPE_UART_DATA;
		
		// Copy UART data to packet
		memcpy(p.requestDataPack.data, &packet, UART_PACKET_SIZE);
		p.requestDataPack.dataSize = UART_PACKET_SIZE;
		
		// Calculate CRC for header
		p.header.crc = p.header.packetType * 2 + p.header.targetAddr * 3 + p.header.srcAddr * 5;
		
		// Transmit with higher timeout to ensure completion
		HAL_SPI_Transmit(&slave, (uint8_t*) &p, PACKET_MAX_SIZE, 200);
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
	uint8_t byte = 0;
	static uint32_t last_uart_check = 0;
	uint32_t current_tick = osKernelGetTickCount();
	
	// Check if there's UART data to process every 5ms
	// This ensures we don't miss UART data while waiting for SPI commands
	if (current_tick - last_uart_check >= 5) {
		last_uart_check = current_tick;
		
		// If there's data in the UART queue, process it immediately
		if (osMessageQueueGetCount(uartQueueHandle) > 0) {
			handleUARTDataRequest();
		}
	}

	// Non-blocking check for start byte
	if (HAL_SPI_Receive(&slave, &byte, 1, 10) != HAL_OK || byte != PACKET_START_BYTE) {
		return; // No valid start byte, return and try again next time
	}
	
	// We received a start byte, now get the full packet
	Packet rx = {0};
	
	if (HAL_SPI_Receive(&slave, (uint8_t*)&rx, sizeof(Packet), 200) != HAL_OK) {
		return;
	}

	PacketHeader *h = &rx.header;

	// Validate packet
	if (h->packetSize < PACKET_HEADER_SIZE || h->packetSize > PACKET_MAX_SIZE) {
		return;
	}
	
	uint8_t crc = h->packetType * 2 + h->targetAddr * 3 + h->srcAddr * 5;
	if (crc != h->crc) {
		return;
	}

	// Handle ASSIGN_ADDR from master (only when unassigned)
	if (h->packetType == PACKET_TYPE_ASSIGN_ADDR && myAddr == 0xFF) {
		myAddr = rx.data[0]; // Assigned address from master
		return;
	}

	// Handle IDENTIFY query
	if (h->packetType == PACKET_TYPE_IDENTIFY && 
	    (h->targetAddr == myAddr || h->targetAddr == broadcast)) {
		
		Packet response = {0};
		Packet ack = {
			.header = {
				.targetAddr = h->srcAddr,
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

		HAL_SPI_Transmit(&slave, (uint8_t*)&ack, PACKET_MAX_SIZE, 200);
		return;
	}

	// If it's for us, handle it here
	if (h->targetAddr == myAddr && myAddr != broadcast && 
	    h->packetType == PACKET_TYPE_REQUEST_DATA) {
		
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
	} else if (downstreamDetected && myAddr != h->targetAddr) {
		forward_packet(&rx);
	}
}
