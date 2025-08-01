/*
 * protocol.h
 *
 *  Created on: Jun 15, 2025
 *      Author: iliasalaur
 */

#ifndef INC_PROTOCOL_H_
#define INC_PROTOCOL_H_

#pragma once

#include <stdint.h>

#define PACKED __attribute__((__packed__))

#define PACKET_START_BYTE 0xAA
#define PACKET_HEADER_SIZE 5
#define PACKET_MAX_SIZE 256  // Increased to handle larger M-Bus packets
#define PACKET_DATA_SIZE (PACKET_MAX_SIZE - PACKET_HEADER_SIZE)
#define MAX_BUS_NODES 10
#define MIN_WAIT_TIME_PER_NODE 50

#define RQD_PACKET_DATA_SIZE (PACKET_DATA_SIZE - 2)

#define ADC_READINGS_SIZE 24
#define EXT_ADC_READINGS_SIZE 22

// Increased to handle larger M-Bus packets
#define UART_PACKET_SIZE    251  // 250 bytes data + 1 byte UART ID
#define UART_DATA_SIZE      250  // M-Bus packets can be up to 250 bytes

// M-Bus specific definitions
#define MBUS_START_BYTE     0x68
#define MBUS_STOP_BYTE      0x16

typedef enum {
	PACKET_TYPE_ASSIGN_ADDR = 0x01,
	PACKET_TYPE_IDENTIFY = 0x02,
	PACKET_TYPE_IDENTIFY_ACK = 0x03,
	PACKET_TYPE_FORWARD = 0x04,
	PACKET_TYPE_REQUEST_DATA = 0x05
// ...extend as needed
}PACKED PacketType;

typedef enum {
	RQP_TYPE_ADC_READINGS = 0x01,
	RQP_TYPE_UART_DATA = 0x02,
	RQP_TYPE_SAY_HI = 0x03,
	RQP_TYPE_EXT_ADC_READINGS = 0x11

}PACKED RequestDataType;

typedef struct {
	uint8_t targetAddr;
	uint8_t srcAddr;
	uint8_t packetSize;
	uint8_t crc;
	uint8_t packetType;

} PACKED PacketHeader;

typedef struct {
	uint8_t identifiedDevices;
	uint8_t searchDepth;

} PACKED IdentifyPacket;

typedef struct {
	uint8_t addressToAssign;

} PACKED AddressAssignPacket;

typedef struct {
	uint8_t requestDataType;
	uint8_t dataSize;
	uint8_t data[RQD_PACKET_DATA_SIZE];

} PACKED RequestDataPacket;

typedef struct {
    uint8_t uart_id;
    uint8_t data[UART_DATA_SIZE];
} PACKED UartPacket;

typedef struct {
	PacketHeader header;
	union {
		uint8_t data[PACKET_DATA_SIZE];
		RequestDataPacket requestDataPack;
		IdentifyPacket identifyPack;
		AddressAssignPacket addrAssignPack;
	};
} PACKED Packet;

#define ADDR_ASSIGN_WHOLEPACK_SIZE (PACKET_HEADER_SIZE + sizeof(AddressAssignPacket))
#define IDENTIFY_WHOLEPACK_SIZE (PACKET_HEADER_SIZE + sizeof(IdentifyPacket))
#define UART_WHOLEPACK_SIZE (PACKET_HEADER_SIZE + sizeof(UartPacket) + sizeof(RequestDataPacket) - RQD_PACKET_DATA_SIZE)

#endif /* INC_PROTOCOL_H_ */
