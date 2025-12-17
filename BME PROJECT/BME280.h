/**
 * @file BME280.h
 *
 * @brief Header file for the BME280 driver.
 *
 * This file contains the function definitions for the BME280 Temperature,
 * Humidity, and Pressure sensor driver using SPI interface.
 *
 * The BME280 is a combined digital humidity, pressure and temperature sensor.
 * It uses the SPI protocol for communication with the microcontroller.
 *
 * **SPI Configuration:**
 * - Mode: 0 sometimes 3, neither work

 * - Data is transmitted MSB first
 * - Register read: Set bit 7 high (0x80 | register_address)
 * - Register write: Keep bit 7 low (0x7F & register_address)
 * * Pin Connections (using SSI0):
 * - PA2: SCK (Serial Clock)
 * - PA3: CS (Chip Select - manual control via GPIO to set to 0)
 * - PA4: MISO (Master In Slave Out - SDO on BME280)
 * - PA5: MOSI (Master Out Slave In - SDI on BME280)
 * - VCC: 3.3V
 * - GND: Ground
 *
 * @note Assumes system clock of 100 MHz
 * @note This driver uses the SSI0 module
 * @note All sensor values are returned as integers (no floating point like manufactuer)
 *
 * @author Garrett Pocne
 */


#include "TM4C123GH6PM.h"

// ==================== BME280 Register Addresses ====================

// Calibration registers for temperature compensation
#define BME280_REG_DIG_T1             0x88
#define BME280_REG_DIG_T2             0x8A
#define BME280_REG_DIG_T3             0x8C

// Calibration registers for pressure compensation
#define BME280_REG_DIG_P1             0x8E
#define BME280_REG_DIG_P2             0x90
#define BME280_REG_DIG_P3             0x92
#define BME280_REG_DIG_P4             0x94
#define BME280_REG_DIG_P5             0x96
#define BME280_REG_DIG_P6             0x98
#define BME280_REG_DIG_P7             0x9A
#define BME280_REG_DIG_P8             0x9C
#define BME280_REG_DIG_P9             0x9E

// Calibration registers for humidity compensation
#define BME280_REG_DIG_H1             0xA1
#define BME280_REG_DIG_H2             0xE1
#define BME280_REG_DIG_H3             0xE3
#define BME280_REG_DIG_H4             0xE4
#define BME280_REG_DIG_H5             0xE5
#define BME280_REG_DIG_H6             0xE7

// Control and status registers
#define BME280_REG_CHIPID             0xD0
#define BME280_REG_VERSION            0xD1
#define BME280_REG_SOFTRESET          0xE0
#define BME280_REG_CTRL_HUM           0xF2
#define BME280_REG_STATUS             0xF3
#define BME280_REG_CTRL_MEAS          0xF4
#define BME280_REG_CONFIG             0xF5

// Data registers (sensor readings)
#define BME280_REG_PRESS_MSB          0xF7
#define BME280_REG_PRESS_LSB          0xF8
#define BME280_REG_PRESS_XLSB         0xF9
#define BME280_REG_TEMP_MSB           0xFA
#define BME280_REG_TEMP_LSB           0xFB
#define BME280_REG_TEMP_XLSB          0xFC
#define BME280_REG_HUM_MSB            0xFD
#define BME280_REG_HUM_LSB            0xFE

// ==================== BME280 Configuration Values ====================

// Expected chip ID value
#define BME280_CHIP_ID              0x60

// Oversampling settings (for temperature, pressure, and humidity)
#define BME280_OVERSAMPLE_SKIP      0x00
#define BME280_OVERSAMPLE_1X        0x01
#define BME280_OVERSAMPLE_2X        0x02
#define BME280_OVERSAMPLE_4X        0x03
#define BME280_OVERSAMPLE_8X        0x04
#define BME280_OVERSAMPLE_16X       0x05

// Power modes
#define BME280_MODE_SLEEP           0x00
#define BME280_MODE_FORCED          0x01
#define BME280_MODE_NORMAL          0x03

// Standby time settings (for normal mode)
#define BME280_STANDBY_0_5MS        0x00
#define BME280_STANDBY_62_5MS       0x01
#define BME280_STANDBY_125MS        0x02
#define BME280_STANDBY_250MS        0x03
#define BME280_STANDBY_500MS        0x04
#define BME280_STANDBY_1000MS       0x05 // 1 second
#define BME280_STANDBY_10MS         0x06
#define BME280_STANDBY_20MS         0x07

// IIR filter coefficient settings
#define BME280_FILTER_OFF           0x00
#define BME280_FILTER_COEFF_2       0x01
#define BME280_FILTER_COEFF_4       0x02
#define BME280_FILTER_COEFF_8       0x03
#define BME280_FILTER_COEFF_16      0x04

// ==================== Data Structures ====================

/**
 * @brief Structure to hold BME280 calibration data
 */
typedef struct {
    // Temperature calibration coefficients
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    
    // Pressure calibration coefficients
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    
    // Humidity calibration coefficients
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
    
    // Fine temperature value used in pressure and humidity compensation
    int32_t  t_fine;
} BME280_CalibData;

/**
 * @brief Structure to hold BME280 sensor data
 */
typedef struct {
    int32_t temperature;    // Temperature in degrees Celsius (e.g., 25)
    uint32_t pressure;      // Pressure in hPa/mbar (e.g., 1013)
    uint32_t humidity;      // Humidity in percent (e.g., 45)
} BME280_Data;

// ==================== Function Prototypes ====================

uint8_t BME280_Init(void);
uint8_t BME280_Read_Register(uint8_t reg_addr);
void BME280_Write_Register(uint8_t reg_addr, uint8_t data);
uint8_t BME280_Get_Chip_ID(void);
uint8_t BME280_Read_Data(BME280_Data *sensor_data);
int32_t BME280_Read_Temperature(void);
uint32_t BME280_Read_Pressure(void);
uint32_t BME280_Read_Humidity(void);
void BME280_Soft_Reset(void);

