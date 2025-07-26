#ifndef ADS124S08_H
#define ADS124S08_H

#include "stm32f4xx_hal.h"

// === SPI Commands ===
#define ADS_CMD_RESET   0x06
#define ADS_CMD_START   0x08
#define ADS_CMD_STOP    0x0A
#define ADS_CMD_RDATA   0x12
#define ADS_CMD_RREG    0x20
#define ADS_CMD_WREG    0x40

#define ADS124S08_REG_COUNT 0x20

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *nss_port;      // GPIO Port for Software NSS
    uint16_t nss_pin;            // GPIO Pin for Software NSS
} ADS124S08;

void ADS124S08_Init(ADS124S08 *adc);
void ADS124S08_Reset(ADS124S08 *adc);
void ADS124S08_StartConversion(ADS124S08 *adc);
void ADS124S08_StopConversion(ADS124S08 *adc);
void ADS124S08_SetInputChannel(ADS124S08 *adc, uint8_t pos, uint8_t neg);


uint8_t ADS124S08_ReadRegister(ADS124S08 *adc, uint8_t reg);
void ADS124S08_WriteRegister(ADS124S08 *adc, uint8_t reg, uint8_t value);
uint32_t ADS124S08_ReadData(ADS124S08 *adc);

#endif
