///*
// * bus_node_legacy.c
// *
// *  Created on: Jun 19, 2025
// *      Author: iliasalaur
// */
//
//
//#include "bus_node.h"
////#include "main.h"
//#include <string.h>
//#include <cmsis_os2.h>
//
//extern SPI_HandleTypeDef hspi1;
//extern SPI_HandleTypeDef hspi2;
//extern SPI_HandleTypeDef hspi3;
//
//SPI_HandleTypeDef slave; // Slave (upstream)
//SPI_HandleTypeDef master; // Master (downstream)
//extern uint16_t adcReadings[12];
//extern int32_t externalADCReadings[11];
//extern osMutexId_t adcBufReadMutexHandle;
//extern osMessageQueueId_t uartQueueHandle;
//
//GPIO_TypeDef *SPI2_SoftNSS_GPIO_Port = GPIOD;
//uint16_t SPI2_SoftNSS_Pin = GPIO_PIN_8;
//
//#define SPI2_NSS_LOW()  HAL_GPIO_WritePin(SPI2_SoftNSS_GPIO_Port, SPI2_SoftNSS_Pin, GPIO_PIN_RESET)
//#define SPI2_NSS_HIGH() HAL_GPIO_WritePin(SPI2_SoftNSS_GPIO_Port, SPI2_SoftNSS_Pin, GPIO_PIN_SET)
//
//static const uint8_t broadcast = 0xff;
//static uint8_t myAddr = 0xFF; // Unassigned
//static bool downstreamDetected = false;
//static volatile bool spi_finished = false;
//static volatile bool spi_awaiting_data = false;
//static volatile spi_prepared = false;
//static volatile Packet rx;
//static uint8_t spi_byte = 0;
//
//void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
//	if(hspi->Instance == SPI3)
//	{
//		if(spi_byte != PACKET_START_BYTE && !spi_awaiting_data)
//		{
//			HAL_SPI_Receive_DMA(hspi, &spi_byte, 1);
//			spi_awaiting_data = false;
//		}
//
//		else if(!spi_awaiting_data && !spi_finished)
//		{
//			HAL_SPI_Receive_DMA(hspi, (uint8_t*)&rx, PACKET_MAX_SIZE);
//			spi_awaiting_data = true;
//		}
//		else
//		{
//			spi_finished = 1;
//			spi_byte = 0;
//		}
//
//	}
//}
//
//void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
//	if(hspi->Instance == SPI3)
//	{
//		spi_prepared = true;
//
//	}
//}
//
//static uint32_t getWaitTime(uint8_t prevNodes) {
//	return prevNodes * MIN_WAIT_TIME_PER_NODE;
//}
//
//static void handleADCDataRequest() {
//
//	osMutexAcquire(adcBufReadMutexHandle, 100);
//	Packet p = { .header = { .targetAddr = 0, .srcAddr = myAddr, .packetSize =
//			16, .packetType = PACKET_TYPE_REQUEST_DATA, }, .data = { 0 } };
//	p.requestDataPack.requestDataType = RQP_TYPE_ADC_READINGS;
//
//	memcpy(p.requestDataPack.data, adcReadings, ADC_READINGS_SIZE);
//
//	osMutexRelease(adcBufReadMutexHandle);
//
//	p.requestDataPack.dataSize = ADC_READINGS_SIZE;
//
//	HAL_SPI_Transmit_DMA(&slave,(uint8_t*) &p, PACKET_MAX_SIZE);
//
//}
//
//static void handleExternalADCDataRequest() {
//
////	osMutexAcquire(adcBufReadMutexHandle, 100);
//	Packet p = { .header = { .targetAddr = 0, .srcAddr = myAddr, .packetSize =
//			16, .packetType = PACKET_TYPE_REQUEST_DATA, }, .data = { 0 } };
//	p.requestDataPack.requestDataType = RQP_TYPE_EXT_ADC_READINGS;
//
//	memcpy(p.requestDataPack.data, externalADCReadings, EXT_ADC_READINGS_SIZE);////
//
////	osMutexRelease(adcBufReadMutexHandle);
//
//	p.requestDataPack.dataSize = EXT_ADC_READINGS_SIZE;
//
//	HAL_SPI_Transmit_DMA(&slave,(uint8_t*) &p, PACKET_MAX_SIZE);
//
//}
//
//static void handleSayHiRequest() {
//	Packet hi = { .header = { .targetAddr = 0, .srcAddr = myAddr, .packetSize =
//			16, .packetType = PACKET_TYPE_REQUEST_DATA, }, .data = { 0 } };
//	hi.requestDataPack.requestDataType = RQP_TYPE_SAY_HI;
//
//	const char *msg = "omg it works!!!";
////	const char *msg = "hello there";
//	size_t dataLen = strlen(msg) + 1;
//	memcpy(hi.requestDataPack.data, msg, dataLen);
//
//	hi.requestDataPack.dataSize = dataLen;
//
//
//	HAL_SPI_Transmit_DMA(&slave, (uint8_t*) &hi, PACKET_MAX_SIZE);
//}
//
//static void handleUARTDataRequest() {
//	UartPacket packet = { 0 };
//
//
//	Packet p = { .header = { .targetAddr = 0, .srcAddr = myAddr, .packetSize =
//	UART_WHOLEPACK_SIZE, .packetType = PACKET_TYPE_REQUEST_DATA, }, .data =
//			{ 0 } };
//	p.requestDataPack.requestDataType = RQP_TYPE_UART_DATA;
//	p.requestDataPack.data[0] = 0xff;
//
//	if (osMessageQueueGet(uartQueueHandle, (void*)&packet, NULL, 10) == osOK)
//	{
//		memcpy(p.requestDataPack.data, &packet, UART_PACKET_SIZE);
//	}
//
//	p.requestDataPack.dataSize = UART_PACKET_SIZE;
//
//	HAL_SPI_Transmit_DMA(&slave,(uint8_t*) &p, PACKET_MAX_SIZE);
//}
//
//static bool detect_downstream(Packet *response, uint8_t prevNodes) {
//	Packet ident = { .header = { .targetAddr = broadcast, .packetSize = 8,
//			.packetType = PACKET_TYPE_IDENTIFY, .srcAddr = myAddr } };
//
//	ident.header.crc = ident.header.packetType * 2 + ident.header.targetAddr * 3
//			+ ident.header.srcAddr * 5;
//	ident.identifyPack.identifiedDevices = 1;
//	ident.identifyPack.searchDepth = prevNodes - 1;
//
//	uint8_t byte = PACKET_START_BYTE;
//
//	SPI2_NSS_LOW();
////    HAL_Delay(1);
//	HAL_SPI_Transmit(&master, &byte, 1, HAL_MAX_DELAY);
//	HAL_SPI_Transmit(&master, (uint8_t*) &ident, PACKET_MAX_SIZE,
//			HAL_MAX_DELAY);
//
////    HAL_Delay(1);
//	osDelay(getWaitTime(prevNodes));
//	int status = HAL_SPI_Receive(&master, (uint8_t*) response, PACKET_MAX_SIZE,
//			getWaitTime(prevNodes));
//
//	SPI2_NSS_HIGH();
//
//	return (response->header.packetType == PACKET_TYPE_IDENTIFY_ACK);
//}
//
//void BusNode_Init(void) {
//	myAddr = 0xFF; // Unassigned
//	downstreamDetected = 0;
//
//	master = hspi2;
//	slave = hspi3;
//
//	HAL_SPI_Receive_DMA(&slave, &spi_byte, 1);
//}
//
//static void forward_packet(const Packet *pkt) {
//	Packet response = { 0 };
//	uint8_t byte = PACKET_START_BYTE;
//	SPI2_NSS_LOW();
//	HAL_SPI_Transmit(&master, &byte, 1, 10);
//	HAL_SPI_Transmit(&master, (uint8_t*) pkt, PACKET_MAX_SIZE, 10);
//	osDelay(1);
//
//	HAL_SPI_Receive(&master, (uint8_t*) &response, PACKET_MAX_SIZE, 100);
//	HAL_SPI_Transmit_DMA(&slave,(uint8_t*) &response, PACKET_MAX_SIZE);
//	SPI2_NSS_HIGH();
//}
//
//void BusNode_Continue(void)
//{
//	while(!spi_prepared)
//	{
//		osDelay(1);
//	}
//
//	HAL_SPI_Receive_DMA(&slave, &spi_byte, 1);
//	spi_prepared = 1;//0
//}
//
//void BusNode_RunLoop(void) {
////	uint8_t byte = 0;
////	while (1) {
////		HAL_SPI_Receive(&slave, &byte, 1, 10);
////		if (byte == PACKET_START_BYTE)
////			break;
////		osDelay(1);
////	}
//
//
//	while(!spi_finished)
//		osDelay(1);
//
//	spi_finished = 0;
//	spi_awaiting_data = 0;
//
////
////    // Wait for a packet from upstream (slave)
////
////	if (HAL_SPI_Receive(&slave, &rx, PACKET_MAX_SIZE, 50) != HAL_OK)
////		return;
//
//	PacketHeader *h = &rx.header;
//
//	{
//		if (h->packetSize < PACKET_HEADER_SIZE
//				|| h->packetSize > PACKET_MAX_SIZE)
//			return;
//		uint8_t crc = h->packetType * 2 + h->targetAddr * 3 + h->srcAddr * 5;
//		if (crc != h->crc)
//			return;
//	}
//
//	// Handle ASSIGN_ADDR from master (only when unassigned)
//	if (h->packetType == PACKET_TYPE_ASSIGN_ADDR && myAddr == 0xFF) {
//		myAddr = rx.data[0]; // Assigned address from master
//
//		return;
//	}
//
//	// Handle IDENTIFY query
//	if (h->packetType == PACKET_TYPE_IDENTIFY
//			&& (h->targetAddr == myAddr || h->targetAddr == broadcast)) {
//
//		Packet response = { 0 };
//		Packet ack = { .header = { .targetAddr = h->srcAddr, .srcAddr = myAddr,
//				.packetSize = 6, .packetType = PACKET_TYPE_IDENTIFY_ACK } };
//
//		ack.identifyPack.identifiedDevices = 1;
//		ack.identifyPack.searchDepth = rx.identifyPack.searchDepth - 1;
//
//		downstreamDetected = detect_downstream(&response,
//				rx.identifyPack.searchDepth - 1);
//		if (downstreamDetected) {
//			ack.identifyPack.identifiedDevices +=
//					response.identifyPack.identifiedDevices;
//		}
//
//
//		HAL_SPI_Transmit(&slave,
//				(uint8_t*) &ack, PACKET_MAX_SIZE, 100);
//		return;
//	}
//
//	// If it's for us, handle it here
//	if (h->targetAddr == myAddr && myAddr != broadcast
//			&& h->packetType == PACKET_TYPE_REQUEST_DATA) {
//		switch (rx.requestDataPack.requestDataType) {
//		case RQP_TYPE_SAY_HI:
//			handleSayHiRequest();
//			break;
//
//		case RQP_TYPE_ADC_READINGS:
//			handleADCDataRequest();
//			break;
//
//		case RQP_TYPE_UART_DATA:
//			handleUARTDataRequest();
//			break;
//
//		case RQP_TYPE_EXT_ADC_READINGS:
//			handleExternalADCDataRequest();
//			break;
//
//		default:
//
//			break;
//		}
//		return;
//	} else if (downstreamDetected) {
//		forward_packet(&rx);
//	}
//}
