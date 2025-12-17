#include "TM4C123GH6PM.h"
#include "SysTick_Delay.h"
#include "BME280.h"
#include "UART0.h"
#include "GPIO.h"

int main(void)
{
    BME280_Data sensor_data;

    SysTick_Delay_Init();
    UART0_Init();
    RGB_LED_Init();

    UART0_Output_String("=== BME280 SPI Test ===");
    UART0_Output_Newline();

    RGB_LED_Output(RGB_LED_RED);
    SysTick_Delay1ms(500);

    UART0_Output_String("Initializing BME280...");
	  SysTick_Delay1ms(500);

    UART0_Output_Newline();
	  SysTick_Delay1ms(500);

    BME280_Init();
		SysTick_Delay1ms(500);


    UART0_Output_String("BME280 initialized!");
	  SysTick_Delay1ms(500);

    UART0_Output_Newline();
	  SysTick_Delay1ms(500);


    while(1)
    {
        if(BME280_Read_Data(&sensor_data))
        {
            UART0_Output_String("Temperature: ");
            UART0_Output_Unsigned_Decimal(sensor_data.temperature);
            UART0_Output_String(" C");
            UART0_Output_Newline();

            UART0_Output_String("Pressure   : ");
            UART0_Output_Unsigned_Decimal(sensor_data.pressure);
            UART0_Output_String(" hPa");
            UART0_Output_Newline();

            UART0_Output_String("Humidity   : ");
            UART0_Output_Unsigned_Decimal(sensor_data.humidity);
            UART0_Output_String(" %");
            UART0_Output_Newline();

            RGB_LED_Output(RGB_LED_GREEN);
        }
        else
        {
            UART0_Output_String("ERROR: Failed to read sensor data");
            UART0_Output_Newline();
            RGB_LED_Output(RGB_LED_BLUE);
        }

        SysTick_Delay1ms(1000);
    }
}




















