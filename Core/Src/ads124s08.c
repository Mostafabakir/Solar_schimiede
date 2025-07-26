#include "ads124s08.h"

// Macro to set NSS pin low (active low)
#define ADS124S08_NSS_LOW(adc)  HAL_GPIO_WritePin((adc)->nss_port, (adc)->nss_pin, GPIO_PIN_RESET)
// Macro to set NSS pin high (inactive)
#define ADS124S08_NSS_HIGH(adc) HAL_GPIO_WritePin((adc)->nss_port, (adc)->nss_pin, GPIO_PIN_SET)

/**
  * @brief Initializes the ADS124S08 ADC struct and performs initial configuration.
  * @param adc Pointer to the ADS124S08 structure.
  * @param hspi_handle Pointer to the SPI_HandleTypeDef used for communication.
  * @param nss_port GPIO Port for the software NSS pin.
  * @param nss_pin GPIO Pin for the software NSS pin.
  * @retval None
  */
void ADS124S08_Init(ADS124S08 *adc) {
//    adc->hspi = hspi_handle;
//    adc->nss_port = nss_port;
//    adc->nss_pin = nss_pin;

    // Ensure NSS is high before starting any communication
    ADS124S08_NSS_HIGH(adc);
    HAL_Delay(10); // Short delay to ensure pin state settles

    ADS124S08_Reset(adc);
    HAL_Delay(5);  // Allow time after reset

    // SYSMON: default sample number = 8 (Register 0x09)
    // Note: The datasheet specifies SYSMON register at address 0x09.
    // The value 0x10 sets the number of samples for internal temperature sensor conversions.
    ADS124S08_WriteRegister(adc, 0x09, 0x10);
    HAL_Delay(5);

    // INPMUX: Configure PGA_GAIN (Register 0x03)
    // Assuming 0x00 for gain = 1 (PGA bypassed or gain 1, consult datasheet for specific bits)
    ADS124S08_WriteRegister(adc, 0x03, 0);
    HAL_Delay(5);

    // REF: Configure Reference (Register 0x05)
    // 0x1A typically enables internal 2.5V reference and connects negative input to AINN.
    // Ensure this matches your desired reference configuration.
    // Bits [7:6] - VREFCON (reference voltage control)
    // Bit [5]    - REF_EN (reference enable)
    // Bits [4:0] - REFP_DIR, REFN_DIR (reference positive/negative direction)
    ADS124S08_WriteRegister(adc, 0x05, 0x1A);
    HAL_Delay(5);
}

/**
  * @brief Sends the RESET command to the ADS124S08.
  * @param adc Pointer to the ADS124S08 structure.
  * @retval None
  */
void ADS124S08_Reset(ADS124S08 *adc) {
    uint8_t cmd = ADS_CMD_RESET;
    ADS124S08_NSS_LOW(adc); // Activate NSS
    HAL_Delay(5); // Delay after pulling NSS low
    HAL_SPI_Transmit(adc->hspi, &cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(5); // Delay after transmit and before deactivating NSS
    ADS124S08_NSS_HIGH(adc); // Deactivate NSS
    HAL_Delay(1); // Small delay after deactivating NSS
}

/**
  * @brief Sends the START command to the ADS124S08.
  * @param adc Pointer to the ADS124S08 structure.
  * @retval None
  */
void ADS124S08_StartConversion(ADS124S08 *adc) {
    uint8_t cmd = ADS_CMD_START;
    ADS124S08_NSS_LOW(adc); // Activate NSS
    HAL_Delay(5); // Delay after pulling NSS low
    HAL_SPI_Transmit(adc->hspi, &cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(5); // Delay after transmit and before deactivating NSS
    ADS124S08_NSS_HIGH(adc); // Deactivate NSS
}

/**
  * @brief Sends the STOP command to the ADS124S08.
  * @param adc Pointer to the ADS124S08 structure.
  * @retval None
  */
void ADS124S08_StopConversion(ADS124S08 *adc) {
    uint8_t cmd = ADS_CMD_STOP;
    ADS124S08_NSS_LOW(adc); // Activate NSS
    HAL_Delay(5); // Delay after pulling NSS low
    HAL_SPI_Transmit(adc->hspi, &cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(5); // Delay after transmit and before deactivating NSS
    ADS124S08_NSS_HIGH(adc); // Deactivate NSS
}

/**
  * @brief Reads a single register from the ADS124S08.
  * @param adc Pointer to the ADS124S08 structure.
  * @param reg Register address to read.
  * @retval Value of the read register, or 0 if address is out of bounds.
  */
uint8_t ADS124S08_ReadRegister(ADS124S08 *adc, uint8_t reg) {
    if (reg >= ADS124S08_REG_COUNT) return 0; // Basic boundary check

    // RREG command format: [RREG | Reg_Address] [Number_of_Registers_to_read - 1] [Dummy Byte]
    // To read one register, Num_Registers - 1 = 0x00
    uint8_t tx[2] = { ADS_CMD_RREG | reg, 0x00 };
    uint8_t rx = 0;

    ADS124S08_NSS_LOW(adc); // Activate NSS
    HAL_Delay(5); // Delay after pulling NSS low
    HAL_SPI_Transmit(adc->hspi, tx, 2, HAL_MAX_DELAY);  // Send command and count byte
    HAL_SPI_Receive(adc->hspi, &rx, 1, 200); // Receive the register value
    HAL_Delay(5); // Delay after transmit/receive and before deactivating NSS
    ADS124S08_NSS_HIGH(adc); // Deactivate NSS
    return rx;
}

/**
  * @brief Writes a single register in the ADS124S08.
  * @param adc Pointer to the ADS124S08 structure.
  * @param reg Register address to write.
  * @param value Value to write to the register.
  * @retval None
  */
void ADS124S08_WriteRegister(ADS124S08 *adc, uint8_t reg, uint8_t value) {
    if (reg >= ADS124S08_REG_COUNT) return; // Basic boundary check

    // WREG command format: [WREG | Reg_Address] [Number_of_Registers_to_write - 1] [Value]
    // To write one register, Num_Registers - 1 = 0x00
    uint8_t tx[3] = { ADS_CMD_WREG | reg, 0x00, value };

    ADS124S08_NSS_LOW(adc); // Activate NSS
    HAL_Delay(5); // Delay after pulling NSS low
    HAL_SPI_Transmit(adc->hspi, tx, 3, HAL_MAX_DELAY); // Send command, count byte, and value
    HAL_Delay(5); // Delay after transmit and before deactivating NSS
    ADS124S08_NSS_HIGH(adc); // Deactivate NSS
}

/**
  * @brief Sets the positive and negative input channels for the ADC (MUX register).
  * @param adc Pointer to the ADS124S08 structure.
  * @param pos Positive input channel (AIN0-AIN15, VREF, etc.).
  * @param neg Negative input channel (AIN0-AIN15, VREF, etc.).
  * @retval None
  */
void ADS124S08_SetInputChannel(ADS124S08 *adc, uint8_t pos, uint8_t neg) {
    if (pos > 15 || neg > 15) return; // Input channels are typically 0-15 (AIN0-AIN15)

    uint8_t mux_value = ((pos & 0x0F) << 4) | (neg & 0x0F); // Combine into MUX register format
    // MUX register address is 0x01 on ADS124S08
    uint8_t tx[3] = {
        ADS_CMD_WREG | 0x01,  // Write to MUX register (0x01)
        0x00,                 // Write 1 register (count - 1)
        mux_value             // The actual MUX register value
    };

    ADS124S08_NSS_LOW(adc); // Activate NSS
    HAL_Delay(5); // Delay after pulling NSS low
    HAL_SPI_Transmit(adc->hspi, tx, sizeof(tx), HAL_MAX_DELAY);
    HAL_Delay(5); // Delay after transmit and before deactivating NSS
    ADS124S08_NSS_HIGH(adc); // Deactivate NSS
}

/**
  * @brief Reads the 24-bit conversion data from the ADS124S08.
  * @param adc Pointer to the ADS124S08 structure.
  * @retval Signed 32-bit integer representing the conversion result.
  */
uint32_t ADS124S08_ReadData(ADS124S08 *adc) {
    uint8_t cmd = ADS_CMD_RDATA; // Read Data command
    uint8_t rx[3] = {0};         // Buffer for 3 data bytes

    ADS124S08_NSS_LOW(adc); // Activate NSS
    HAL_Delay(5); // Delay after pulling NSS low
//    osDelay(5);
    HAL_SPI_Transmit(adc->hspi, &cmd, 1, 100); // Send RDATA command
    HAL_SPI_Receive(adc->hspi, rx, 3, 200);    // Receive 3 data bytes
    HAL_Delay(5); // Delay after transmit/receive and before deactivating NSS
//    osDelay(5);
    ADS124S08_NSS_HIGH(adc); // Deactivate NSS

    // Reconstruct 24-bit value into a 32-bit signed integer
    uint32_t value = (rx[0] << 16) | (rx[1] << 8) | rx[2];

    // Check for negative 24-bit value and sign-extend to 32 bits
//    if (value & 0x800000) { // If the 24th bit (MSB of 24-bit data) is set
//        value |= 0xFF000000; // Sign-extend by filling upper bits with 1s
//    }

    return value;
}
