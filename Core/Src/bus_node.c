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
    // Calculate actual packet size needed
    uint16_t packet_size = PACKET_HEADER_SIZE + 2 + ADC_READINGS_SIZE;
    
    // Create packet with ADC data
    Packet p = { 
        .header = { 
            .targetAddr = 0, 
            .srcAddr = myAddr, 
            .packetSize = packet_size, 
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

    // Transmit with moderate timeout
    HAL_SPI_Transmit(&slave, (uint8_t*)&p, packet_size, 100);
}

static void handleExternalADCDataRequest() {
    // Calculate actual packet size needed
    uint16_t packet_size = PACKET_HEADER_SIZE + 2 + EXT_ADC_READINGS_SIZE;
    
    // Create packet with external ADC data
    Packet p = { 
        .header = { 
            .targetAddr = 0, 
            .srcAddr = myAddr, 
            .packetSize = packet_size, 
            .packetType = PACKET_TYPE_REQUEST_DATA 
        }, 
        .data = { 0 } 
    };
    
    // Calculate CRC for header
    p.header.crc = p.header.packetType * 2 + p.header.targetAddr * 3 + p.header.srcAddr * 5;
        printf("[SPI-MON] Sent empty UART packet: size=%d\n", p.header.packetSize);
    
    // Set request type
    p.requestDataPack.requestDataType = RQP_TYPE_EXT_ADC_READINGS;

    // Copy external ADC readings to packet
    // No mutex needed as this is accessed only from one task
    memcpy(p.requestDataPack.data, externalADCReadings, EXT_ADC_READINGS_SIZE);

    // Set data size
    p.requestDataPack.dataSize = EXT_ADC_READINGS_SIZE;

    // Transmit with moderate timeout
    HAL_SPI_Transmit(&slave, (uint8_t*)&p, packet_size, 100);
}

static void handleSayHiRequest() {
    // Message to send
    const char *msg = "Solar Schmiede STM Adapter v1.0";
    size_t dataLen = strlen(msg) + 1;
    
    // Calculate actual packet size needed
    uint16_t packet_size = PACKET_HEADER_SIZE + 2 + dataLen;
    
    // Create packet for "Say Hi" response
    Packet hi = { 
        .header = { 
            .targetAddr = 0, 
            .srcAddr = myAddr, 
            .packetSize = packet_size, 
            .packetType = PACKET_TYPE_REQUEST_DATA 
        }, 
        .data = { 0 } 
    };
    
    // Calculate CRC for header
    hi.header.crc = hi.header.packetType * 2 + hi.header.targetAddr * 3 + hi.header.srcAddr * 5;
    
    // Set request type
    hi.requestDataPack.requestDataType = RQP_TYPE_SAY_HI;
    
    // Copy message to packet
    memcpy(hi.requestDataPack.data, msg, dataLen);

    // Set data size
    hi.requestDataPack.dataSize = dataLen;

    // Transmit with moderate timeout
    HAL_SPI_Transmit(&slave, (uint8_t*)&hi, packet_size, 100);
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
        // Log empty UART packet transmission
        printf("[SPI-MON] Sent empty UART packet: size=%d\n", p.header.packetSize);
        // Try to transmit with retry
        for (int retry = 0; retry < 3; retry++) {
            if (HAL_SPI_Transmit(&slave, (uint8_t*)&p, p.header.packetSize, 50) == HAL_OK) {
                break;
            }
            osDelay(1);
        }
        return;
    }
    
    // Process message from queue
    if (osMessageQueueGet(uartQueueHandle, (void*)&packet, NULL, 0) == osOK) {
        // Get actual data length
        uint16_t data_len = 0;
        uint8_t is_last_chunk = 0;
        uint8_t is_mbus_packet = 0;
        // Check if this looks like an M-Bus packet
        if (packet.data[0] == MBUS_START_BYTE && packet.data[3] == MBUS_START_BYTE) {
            is_mbus_packet = 1;
        }
        // Check for end of packet marker (0x16)
        for (data_len = 0; data_len < UART_DATA_SIZE; data_len++) {
            if (packet.data[data_len] == MBUS_STOP_BYTE && data_len > 5) {
                // Found stop byte, include it in the packet
                data_len++;
                is_last_chunk = 1;
                break;
            }
        }
        // If no stop byte found, use the entire buffer
        if (data_len == 0) {
            data_len = UART_DATA_SIZE;
        }
        // Create packet with appropriate size (don't exceed PACKET_MAX_SIZE)
        uint16_t packet_size = PACKET_HEADER_SIZE + 4 + data_len; // Header + requestType + dataSize + flags + mbus_flag + data
        if (packet_size > PACKET_MAX_SIZE) {
            packet_size = PACKET_MAX_SIZE;
            data_len = PACKET_MAX_SIZE - PACKET_HEADER_SIZE - 4;
        }
        Packet p = { 
            .header = { 
                .targetAddr = 0, 
                .srcAddr = myAddr, 
                .packetSize = packet_size,
                .packetType = PACKET_TYPE_REQUEST_DATA 
            }, 
            .data = { 0 } 
        };
        // Calculate CRC for header
        p.header.crc = p.header.packetType * 2 + p.header.targetAddr * 3 + p.header.srcAddr * 5;
        p.requestDataPack.requestDataType = RQP_TYPE_UART_DATA;
        p.requestDataPack.dataSize = data_len + 3; // Data + UART ID + flags + mbus_flag
        // Copy UART ID, flags, and data
        p.requestDataPack.data[0] = packet.uart_id;
        p.requestDataPack.data[1] = is_last_chunk; // Flag to indicate if this is the last chunk
        p.requestDataPack.data[2] = is_mbus_packet; // Flag to indicate if this is an M-Bus packet
        memcpy(&p.requestDataPack.data[3], packet.data, data_len);
        // Log SPI packet transmission
        printf("[SPI-MON] Sent UART packet: uart_id=%d, last_chunk=%d, mbus=%d, size=%d\n", packet.uart_id, is_last_chunk, is_mbus_packet, p.header.packetSize);
        // Try to transmit with retry
        HAL_StatusTypeDef result = HAL_ERROR;
        for (int retry = 0; retry < 3; retry++) {
            result = HAL_SPI_Transmit(&slave, (uint8_t*)&p, p.header.packetSize, 100);
            if (result == HAL_OK) {
                break;
            }
            osDelay(2);
        }
        // If transmission was successful, add a small delay to allow ESP32 to process the data
        if (result == HAL_OK) {
            osDelay(3);
        }
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
    HAL_Delay(1); // Shorter delay to avoid timing issues
    
    // Send start byte
    if (HAL_SPI_Transmit(&master, &byte, 1, 50) != HAL_OK) {
        SPI2_NSS_HIGH();
        return false; // Abort if transmission fails
    }
    
    // Send identification packet (use actual size, not maximum)
    if (HAL_SPI_Transmit(&master, (uint8_t*)&ident, IDENTIFY_WHOLEPACK_SIZE, 100) != HAL_OK) {
        SPI2_NSS_HIGH();
        return false; // Abort if transmission fails
    }
    
    // End first transmission
    SPI2_NSS_HIGH();

    // Wait for downstream nodes to process the packet
    uint32_t wait_time = getWaitTime(prevNodes) + 20;
    osDelay(wait_time);
    
    // Second transmission - receive response
    SPI2_NSS_LOW();
    HAL_Delay(1);
    
    // Clear response buffer
    memset(response, 0, sizeof(Packet));
    
    // First receive header to determine actual packet size
    if (HAL_SPI_Receive(&master, (uint8_t*)response, PACKET_HEADER_SIZE, 100) != HAL_OK) {
        SPI2_NSS_HIGH();
        return false;
    }
    
    // Validate header
    if (response->header.packetSize < PACKET_HEADER_SIZE || 
        response->header.packetSize > PACKET_MAX_SIZE) {
        SPI2_NSS_HIGH();
        return false;
    }
    
    // Receive rest of packet if needed
    if (response->header.packetSize > PACKET_HEADER_SIZE) {
        uint16_t data_size = response->header.packetSize - PACKET_HEADER_SIZE;
        if (HAL_SPI_Receive(&master, (uint8_t*)&response->data, data_size, 100) != HAL_OK) {
            SPI2_NSS_HIGH();
            return false;
        }
    }
    
    // End second transmission
    SPI2_NSS_HIGH();

    // Check if we received a valid acknowledgment
    if (response->header.packetType == PACKET_TYPE_IDENTIFY_ACK) {
        // Validate CRC
        uint8_t crc = response->header.packetType * 2 + 
                      response->header.targetAddr * 3 + 
                      response->header.srcAddr * 5;
                      
        return (crc == response->header.crc);
    }
    
    return false;
}

void BusNode_Init(void) {
    // Initialize variables
    myAddr = 0xFF; // Unassigned
    downstreamDetected = false;
    spi_finished = 0;

    // Set up SPI interfaces
    master = hspi2;
    slave = hspi1;
    
    // Ensure NSS pin is high initially
    SPI2_NSS_HIGH();
    
    // Reset SPI peripherals to ensure clean state
    HAL_SPI_DeInit(&slave);
    HAL_SPI_DeInit(&master);
    HAL_SPI_Init(&slave);
    HAL_SPI_Init(&master);
    
    // Small delay to allow SPI to stabilize
    HAL_Delay(10);
}

static void forward_packet(const Packet *pkt) {
    // Buffer for response
    Packet response = {0};
    
    // Start byte
    uint8_t byte = PACKET_START_BYTE;
    
    // First transmission - send start byte and packet
    SPI2_NSS_LOW();
    HAL_Delay(1); // Shorter delay to avoid timing issues
    
    // Send start byte
    if (HAL_SPI_Transmit(&master, &byte, 1, 50) != HAL_OK) {
        SPI2_NSS_HIGH();
        return; // Abort if transmission fails
    }
    
    // Send packet (use actual packet size, not maximum)
    uint16_t packet_size = pkt->header.packetSize;
    if (packet_size < PACKET_HEADER_SIZE || packet_size > PACKET_MAX_SIZE) {
        packet_size = PACKET_MAX_SIZE; // Fallback to max size if invalid
    }
    
    if (HAL_SPI_Transmit(&master, (uint8_t*)pkt, packet_size, 100) != HAL_OK) {
        SPI2_NSS_HIGH();
        return; // Abort if transmission fails
    }
    
    // End first transmission
    SPI2_NSS_HIGH();
    
    // Small delay between transmissions
    HAL_Delay(2);
    
    // Second transmission - receive response
    SPI2_NSS_LOW();
    HAL_Delay(1);
    
    // First receive header to determine actual packet size
    if (HAL_SPI_Receive(&master, (uint8_t*)&response, PACKET_HEADER_SIZE, 100) != HAL_OK) {
        SPI2_NSS_HIGH();
        return;
    }
    
    // Validate header
    if (response.header.packetSize < PACKET_HEADER_SIZE || 
        response.header.packetSize > PACKET_MAX_SIZE) {
        SPI2_NSS_HIGH();
        return;
    }
    
    // Receive rest of packet if needed
    if (response.header.packetSize > PACKET_HEADER_SIZE) {
        uint16_t data_size = response.header.packetSize - PACKET_HEADER_SIZE;
        if (HAL_SPI_Receive(&master, (uint8_t*)&response.data, data_size, 100) != HAL_OK) {
            SPI2_NSS_HIGH();
            return;
        }
    }
    
    // End second transmission
    SPI2_NSS_HIGH();
    
    // Calculate CRC for response header
    response.header.crc = response.header.packetType * 2 + 
                          response.header.targetAddr * 3 + 
                          response.header.srcAddr * 5;
    
    // Forward response back to the slave (use actual packet size)
    HAL_SPI_Transmit(&slave, (uint8_t*)&response, response.header.packetSize, 100);
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
    static uint32_t last_response_time = 0;
    static uint8_t consecutive_failures = 0;
    uint32_t current_tick = osKernelGetTickCount();
    
    // Check for UART data more frequently (every 2ms)
    // This ensures we prioritize forwarding M-Bus data to ESP32
    if (current_tick - last_uart_check >= 2) {
        last_uart_check = current_tick;
        
        // If there's data in the UART queue, process it immediately
        uint32_t msg_count = osMessageQueueGetCount(uartQueueHandle);
        if (msg_count > 0) {
            // Process all available UART messages (up to 3 at a time to avoid blocking)
            for (uint32_t i = 0; i < msg_count && i < 3; i++) {
                handleUARTDataRequest();
                // Add a small delay between UART packet transmissions
                osDelay(2);
            }
            
            // Return early to prioritize UART data processing
            return;
        }
    }

    // Check for SPI commands from master
    uint8_t byte = 0;
    
    // Non-blocking check for start byte with increased timeout
    HAL_StatusTypeDef status = HAL_SPI_Receive(&slave, &byte, 1, 10);
    
    if (status != HAL_OK) {
        // Reset SPI if we've had too many consecutive failures
        if (++consecutive_failures > 10) {
            // Re-initialize SPI
            HAL_SPI_DeInit(&slave);
            HAL_SPI_Init(&slave);
            consecutive_failures = 0;
        }
        return;
    }
    
    if (byte != PACKET_START_BYTE) {
        return; // No valid start byte, return and try again next time
    }
    
    // Reset consecutive failures counter on successful reception
    consecutive_failures = 0;
    
    // We received a start byte, now get the packet header first
    PacketHeader header = {0};
    
    if (HAL_SPI_Receive(&slave, (uint8_t*)&header, PACKET_HEADER_SIZE, 100) != HAL_OK) {
        return;
    }
    
    // Validate header with more lenient checks
    if (header.packetSize < PACKET_HEADER_SIZE) {
        header.packetSize = PACKET_HEADER_SIZE; // Fix minimum size
    } else if (header.packetSize > PACKET_MAX_SIZE) {
        header.packetSize = PACKET_MAX_SIZE; // Cap at maximum size
    }
    
    // Recalculate CRC to ensure it matches
    uint8_t expected_crc = header.packetType * 2 + header.targetAddr * 3 + header.srcAddr * 5;
    header.crc = expected_crc; // Ensure CRC is correct
    
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

    // Record the time of last successful response
    last_response_time = current_tick;

    // Handle ASSIGN_ADDR from master (only when unassigned)
    if (rx.header.packetType == PACKET_TYPE_ASSIGN_ADDR) {
        // Accept address assignment even if we already have an address
        myAddr = rx.data[0];
        
        // Send acknowledgment
        Packet ack = {
            .header = {
                .targetAddr = rx.header.srcAddr,
                .srcAddr = myAddr,
                .packetSize = PACKET_HEADER_SIZE,
                .packetType = PACKET_TYPE_ASSIGN_ADDR
            }
        };
        
        // Calculate CRC for header
        ack.header.crc = ack.header.packetType * 2 + ack.header.targetAddr * 3 + ack.header.srcAddr * 5;
        
        // Send acknowledgment with higher priority
        HAL_SPI_Transmit(&slave, (uint8_t*)&ack, PACKET_HEADER_SIZE, 100);
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
        ack.identifyPack.searchDepth = rx.identifyPack.searchDepth > 0 ? rx.identifyPack.searchDepth - 1 : 0;

        // Try to detect downstream nodes, but don't let it fail the whole operation
        if (detect_downstream(&response, ack.identifyPack.searchDepth)) {
            downstreamDetected = true;
            ack.identifyPack.identifiedDevices += response.identifyPack.identifiedDevices;
        }

        // Send response with higher priority
        HAL_SPI_Transmit(&slave, (uint8_t*)&ack, IDENTIFY_WHOLEPACK_SIZE, 100);
        return;
    }

    // If it's for us, handle it here - be more lenient with target address check
    if ((rx.header.targetAddr == myAddr || rx.header.targetAddr == broadcast) && 
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
            // For unknown request types, send a minimal response
            Packet p = { 
                .header = { 
                    .targetAddr = rx.header.srcAddr, 
                    .srcAddr = myAddr, 
                    .packetSize = PACKET_HEADER_SIZE + 2, 
                    .packetType = PACKET_TYPE_REQUEST_DATA 
                }
            };
            p.header.crc = p.header.packetType * 2 + p.header.targetAddr * 3 + p.header.srcAddr * 5;
            p.requestDataPack.requestDataType = rx.requestDataPack.requestDataType;
            p.requestDataPack.dataSize = 0;
            
            HAL_SPI_Transmit(&slave, (uint8_t*)&p, p.header.packetSize, 100);
            break;
        }
        return;
    } else if (downstreamDetected && rx.header.targetAddr != myAddr && rx.header.targetAddr != broadcast) {
        // Forward packet to downstream nodes
        forward_packet(&rx);
    } else {
        // If we don't know what to do with this packet, at least send a minimal response
        // to prevent the ESP32 from timing out
        Packet p = { 
            .header = { 
                .targetAddr = rx.header.srcAddr, 
                .srcAddr = myAddr, 
                .packetSize = PACKET_HEADER_SIZE, 
                .packetType = rx.header.packetType 
            }
        };
        p.header.crc = p.header.packetType * 2 + p.header.targetAddr * 3 + p.header.srcAddr * 5;
        
        HAL_SPI_Transmit(&slave, (uint8_t*)&p, PACKET_HEADER_SIZE, 100);
    }
}
