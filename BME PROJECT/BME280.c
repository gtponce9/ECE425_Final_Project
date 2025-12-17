/**
 * @file BME280.c
 *
 * @brief Complete source code for BME280 driver (SPI, 8-bit transfers, no chip ID check)
 */

#include "BME280.h"
#include "SysTick_Delay.h"
#include "UART0.h"

// ==================== Chip Select Control ====================
#define BME280_CS_LOW()     (GPIOA->DATA &= ~0x08)
#define BME280_CS_HIGH()    (GPIOA->DATA |= 0x08)

// ==================== SSI0 Status Bit Masks ====================
#define SSI0_RX_FIFO_NOT_EMPTY_BIT_MASK 0x04
#define SSI0_TX_FIFO_NOT_FULL_BIT_MASK  0x02
#define SSI0_BUSY_BIT_MASK              0x10

// ==================== Timeout ====================
#define SPI_TIMEOUT 100000

// ==================== Global Calibration Data ====================
static BME280_CalibData calib_data;

// ==================== Private Function Prototypes ====================
static void BME280_SSI0_Init(void);
static uint8_t BME280_SPI_Transfer(uint8_t data);
static void BME280_Read_Calibration_Data(void);
static int32_t BME280_Compensate_Temperature(int32_t adc_T);
static uint32_t BME280_Compensate_Pressure(int32_t adc_P);
static uint32_t BME280_Compensate_Humidity(int32_t adc_H);

// ==================== Private Functions ====================

static void BME280_SSI0_Init(void)
{
    // Enable SSI0 and GPIOA clocks
    SYSCTL->RCGCSSI |= 0x01;
    SYSCTL->RCGCGPIO |= 0x01;
    while((SYSCTL->PRGPIO & 0x01) == 0) {};

    // Configure PA2, PA4, PA5 as SSI0, PA3 as GPIO
    GPIOA->AFSEL |= 0x3C;   // PA2, PA4, PA5
   // GPIOA->AFSEL &= ~0x08;  // PA3 as GPIO
    GPIOA->DIR |= 0x08;     // PA3 output
    GPIOA->PCTL &= ~0x00FFFF00;
    GPIOA->PCTL |= 0x00222200;
    GPIOA->DEN |= 0x3C;     // Enable digital for PA2-5
    BME280_CS_HIGH();       // CS high

    // Configure SSI0
    SSI0->CR1 &= ~0x02;     // Disable SSI0
    SSI0->CR1 &= ~0x04;     // Master mode
    SSI0->CR1 &= ~0x01;     // No loopback
    SSI0->CC  &= ~0x0F;      // System clock

    SSI0->CPSR = 100;       // SSI clock = 50MHz / 100 = 500kHz
    SSI0->CR0 &= ~0xFF00;
    SSI0->CR0 |= 0x000F;		// 8 bits
    SSI0->CR0 &= ~0x80;     // CPHA = 0
    SSI0->CR0 &= ~0x40;     // CPOL = 0
    SSI0->CR0 &= ~0x30;     // Freescale SPI format
    SSI0->CR1 |= 0x02;      // Enable SSI0
}

static uint8_t BME280_SPI_Transfer(uint8_t data)
{
    uint32_t timeout = 10000;

    // Wait until TX FIFO not full
    while((SSI0->SR & SSI0_TX_FIFO_NOT_FULL_BIT_MASK) == 0)
        if(--timeout == 0) return 0xFF;

    SSI0->DR = data;

    timeout = 10000;
    while((SSI0->SR & SSI0_RX_FIFO_NOT_EMPTY_BIT_MASK) == 0)
        if(--timeout == 0) return 0xFF;

    return (uint8_t)(SSI0->DR & 0xFF);
}

static void BME280_Read_Calibration_Data(void)
{
    uint8_t lsb, msb;

    // Temperature calibration
    lsb = BME280_Read_Register(BME280_REG_DIG_T1);
    msb = BME280_Read_Register(BME280_REG_DIG_T1 + 1);
    calib_data.dig_T1 = (msb << 8) | lsb;

    lsb = BME280_Read_Register(BME280_REG_DIG_T2);
    msb = BME280_Read_Register(BME280_REG_DIG_T2 + 1);
    calib_data.dig_T2 = (int16_t)((msb << 8) | lsb);

    lsb = BME280_Read_Register(BME280_REG_DIG_T3);
    msb = BME280_Read_Register(BME280_REG_DIG_T3 + 1);
    calib_data.dig_T3 = (int16_t)((msb << 8) | lsb);

    // Pressure calibration
    lsb = BME280_Read_Register(BME280_REG_DIG_P1);
    msb = BME280_Read_Register(BME280_REG_DIG_P1 + 1);
    calib_data.dig_P1 = (msb << 8) | lsb;

    lsb = BME280_Read_Register(BME280_REG_DIG_P2);
    msb = BME280_Read_Register(BME280_REG_DIG_P2 + 1);
    calib_data.dig_P2 = (int16_t)((msb << 8) | lsb);

    lsb = BME280_Read_Register(BME280_REG_DIG_P3);
    msb = BME280_Read_Register(BME280_REG_DIG_P3 + 1);
    calib_data.dig_P3 = (int16_t)((msb << 8) | lsb);

    lsb = BME280_Read_Register(BME280_REG_DIG_P4);
    msb = BME280_Read_Register(BME280_REG_DIG_P4 + 1);
    calib_data.dig_P4 = (int16_t)((msb << 8) | lsb);

    lsb = BME280_Read_Register(BME280_REG_DIG_P5);
    msb = BME280_Read_Register(BME280_REG_DIG_P5 + 1);
    calib_data.dig_P5 = (int16_t)((msb << 8) | lsb);

    lsb = BME280_Read_Register(BME280_REG_DIG_P6);
    msb = BME280_Read_Register(BME280_REG_DIG_P6 + 1);
    calib_data.dig_P6 = (int16_t)((msb << 8) | lsb);

    lsb = BME280_Read_Register(BME280_REG_DIG_P7);
    msb = BME280_Read_Register(BME280_REG_DIG_P7 + 1);
    calib_data.dig_P7 = (int16_t)((msb << 8) | lsb);

    lsb = BME280_Read_Register(BME280_REG_DIG_P8);
    msb = BME280_Read_Register(BME280_REG_DIG_P8 + 1);
    calib_data.dig_P8 = (int16_t)((msb << 8) | lsb);

    lsb = BME280_Read_Register(BME280_REG_DIG_P9);
    msb = BME280_Read_Register(BME280_REG_DIG_P9 + 1);
    calib_data.dig_P9 = (int16_t)((msb << 8) | lsb);

    // Humidity calibration
    calib_data.dig_H1 = BME280_Read_Register(BME280_REG_DIG_H1);

    lsb = BME280_Read_Register(BME280_REG_DIG_H2);
    msb = BME280_Read_Register(BME280_REG_DIG_H2 + 1);
    calib_data.dig_H2 = (int16_t)((msb << 8) | lsb);

    calib_data.dig_H3 = BME280_Read_Register(BME280_REG_DIG_H3);

    uint8_t h4_msb = BME280_Read_Register(BME280_REG_DIG_H4);
    uint8_t h4_h5_shared = BME280_Read_Register(BME280_REG_DIG_H4 + 1);
    uint8_t h5_msb = BME280_Read_Register(BME280_REG_DIG_H5 + 1);

    calib_data.dig_H4 = (int16_t)((h4_msb << 4) | (h4_h5_shared & 0x0F));
    calib_data.dig_H5 = (int16_t)((h5_msb << 4) | (h4_h5_shared >> 4));
    calib_data.dig_H6 = (int8_t)BME280_Read_Register(BME280_REG_DIG_H6);
}

// Compensation functions (fully expanded)
static int32_t BME280_Compensate_Temperature(int32_t adc_T)
{
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)calib_data.dig_T1 << 1))) * ((int32_t)calib_data.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)calib_data.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib_data.dig_T1))) >> 12) * ((int32_t)calib_data.dig_T3)) >> 14;
    calib_data.t_fine = var1 + var2;
    int32_t T = (calib_data.t_fine * 5 + 128) >> 8;
    return T / 100;
}

static uint32_t BME280_Compensate_Pressure(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)calib_data.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib_data.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib_data.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib_data.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib_data.dig_P3) >> 8) + ((var1 * (int64_t)calib_data.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * (int64_t)calib_data.dig_P1) >> 33;
    if(var1 == 0) return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib_data.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib_data.dig_P7) << 4);
    return (uint32_t)(p / 25600);
}

static uint32_t BME280_Compensate_Humidity(int32_t adc_H)
{
    int32_t v_x1_u32r;

    v_x1_u32r = calib_data.t_fine - 76800;
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib_data.dig_H4) << 20) -
                    (((int32_t)calib_data.dig_H5) * v_x1_u32r)) + 16384) >> 15) *
                 (((((((v_x1_u32r * calib_data.dig_H6) >> 10) *
                      (((v_x1_u32r * calib_data.dig_H3) >> 11) + 32768)) >> 10) + 2097152) *
                   calib_data.dig_H2 + 8192) >> 14));

    v_x1_u32r = v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * calib_data.dig_H1) >> 4);

    // Clamp to 0–100% range in fixed-point
    if (v_x1_u32r < 0) v_x1_u32r = 0;
    if (v_x1_u32r > 419430400) v_x1_u32r = 419430400;

    // Return humidity in 0.01% resolution
    return (uint32_t)((v_x1_u32r >> 12) / 1024); 
}


// ==================== Public Functions ====================

uint8_t BME280_Read_Register(uint8_t reg_addr)
{
    reg_addr |= 0x80; // Read operation
    BME280_CS_LOW();
    SysTick_Delay1us(10);
    BME280_SPI_Transfer(reg_addr);
    uint8_t value = BME280_SPI_Transfer(0x00);
    uint32_t timeout = SPI_TIMEOUT;
    while(SSI0->SR & SSI0_BUSY_BIT_MASK) if(--timeout == 0) break;
    SysTick_Delay1us(10);
    BME280_CS_HIGH();
    return value;
}

void BME280_Write_Register(uint8_t reg_addr, uint8_t data)
{
    reg_addr &= 0x7F; // Write operation
    BME280_CS_LOW();
    SysTick_Delay1us(10);
    BME280_SPI_Transfer(reg_addr);
    BME280_SPI_Transfer(data);
    uint32_t timeout = SPI_TIMEOUT;
    while(SSI0->SR & SSI0_BUSY_BIT_MASK) if(--timeout == 0) break;
    SysTick_Delay1us(10);
    BME280_CS_HIGH();
}

void BME280_Soft_Reset(void)
{
    BME280_Write_Register(BME280_REG_SOFTRESET, 0xB6);
    SysTick_Delay1ms(10);
}

uint8_t BME280_Init(void)
{
    BME280_SSI0_Init();
    SysTick_Delay1ms(100);
    BME280_Soft_Reset();
    BME280_Read_Calibration_Data();

    // Configure humidity
    BME280_Write_Register(BME280_REG_CTRL_HUM, BME280_OVERSAMPLE_1X);

    // Configure temperature/pressure/mode
    uint8_t ctrl_meas = (BME280_OVERSAMPLE_1X << 5) | (BME280_OVERSAMPLE_1X << 2) | BME280_MODE_NORMAL;
    BME280_Write_Register(BME280_REG_CTRL_MEAS, ctrl_meas);

    // Configure standby and filter
    uint8_t config = (BME280_STANDBY_1000MS << 5) | (BME280_FILTER_OFF << 2);
    BME280_Write_Register(BME280_REG_CONFIG, config);

    SysTick_Delay1ms(100);
    return 1;
}

uint8_t BME280_Read_Data(BME280_Data *sensor_data)
{
    uint8_t data[8];
    int32_t adc_P, adc_T, adc_H;

    data[0] = BME280_Read_Register(BME280_REG_PRESS_MSB);
    data[1] = BME280_Read_Register(BME280_REG_PRESS_LSB);
    data[2] = BME280_Read_Register(BME280_REG_PRESS_XLSB);
    data[3] = BME280_Read_Register(BME280_REG_TEMP_MSB);
    data[4] = BME280_Read_Register(BME280_REG_TEMP_LSB);
    data[5] = BME280_Read_Register(BME280_REG_TEMP_XLSB);
    data[6] = BME280_Read_Register(BME280_REG_HUM_MSB);
    data[7] = BME280_Read_Register(BME280_REG_HUM_LSB);

    adc_P = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((uint32_t)data[2] >> 4);
    adc_T = ((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | ((uint32_t)data[5] >> 4);
    adc_H = ((uint32_t)data[6] << 8) | (uint32_t)data[7];

    sensor_data->temperature = BME280_Compensate_Temperature(adc_T);
    sensor_data->pressure = BME280_Compensate_Pressure(adc_P);
    sensor_data->humidity = BME280_Compensate_Humidity(adc_H);

    return 1;
}

int32_t BME280_Read_Temperature(void)
{
    BME280_Data sensor_data;
    BME280_Read_Data(&sensor_data);
	    SysTick_Delay1us(10);

    return sensor_data.temperature;
}

uint32_t BME280_Read_Pressure(void)
{
    BME280_Data sensor_data;
    BME280_Read_Data(&sensor_data);
	    SysTick_Delay1us(10);

    return sensor_data.pressure;
}

uint32_t BME280_Read_Humidity(void)
{
    BME280_Data sensor_data;
    BME280_Read_Data(&sensor_data);
	    SysTick_Delay1us(10);

    return sensor_data.humidity;
}




