/*************************************************************************************
 Title	 :  MAXIM Integrated MAX31855 Library for STM32 Using HAL Libraries
 Author  :  Bardia Alikhan Afshar <bardia.a.afshar@gmail.com>
 Software:  STM32CubeIDE
 Hardware:  Any STM32 device
*************************************************************************************/
#include "MAX31855.h"
extern SPI_HandleTypeDef hspi1;
#define SAMPLEM_TIMES 21

// ------------------- Variables ----------------

volatile uint8_t Error=0;                                      // Thermocouple Connection acknowledge Flag
uint32_t sign=0;									  // Sign bit
uint8_t DATARX[4];                                    // Raw Data from MAX6675
// ------------------- Functions ----------------
void check_kmeter(void)
{
  if (Error == 1)
    LL_GPIO_SetOutputPin(LED_CTR_GPIO_Port, LED_CTR_Pin);
  else
    LL_GPIO_ResetOutputPin(LED_CTR_GPIO_Port, LED_CTR_Pin);
}

float Max31855_Read_Temp_With_Filter(void){
    float temp_data = 0;
    float value_buf[SAMPLEM_TIMES];
    unsigned int count, i, j, temp;
    uint32_t timeout_count = 0;

timeout:
    timeout_count = 0;

    for( count = 0; count < SAMPLEM_TIMES; count++ )
    {
        temp_data = Max31855_Read_Temp();

        while (Error)
        {
            temp_data = Max31855_Read_Temp();
            check_kmeter();
            timeout_count++;
        }
        if (timeout_count >= 20000)
            goto timeout;
        value_buf[count] = temp_data;
    }

    for( j = 0; j < SAMPLEM_TIMES - 1; j++ )
    {
        for( i = 0; i < SAMPLEM_TIMES - j - 1; i++ )
        {
            if( value_buf[i] > value_buf[i + 1] )
            {
                temp = value_buf[i];
                value_buf[i] = value_buf[i + 1];
                value_buf[i + 1] = temp;
            }
        }
    }
    return value_buf[( SAMPLEM_TIMES - 1 ) / 2];
}

float Max31855_Read_Temp(void){
	int Temp=0; 
    uint8_t inter_sign=0; 
                                             // Temperature Variable
    
    LL_GPIO_ResetOutputPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin);                                             
	// HAL_GPIO_WritePin(SSPORT,SSPIN,GPIO_PIN_RESET);       // Low State for SPI Communication
	HAL_SPI_Receive(&hspi1,DATARX,4,1000);                // DATA Transfer
    LL_GPIO_SetOutputPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin);
	// HAL_GPIO_WritePin(SSPORT,SSPIN,GPIO_PIN_SET);         // High State for SPI Communication
	Error=DATARX[3]&0x07;								  // Error Detection
	sign=(DATARX[0]&(0x80))>>7;							  // Sign Bit calculation

    inter_sign = (DATARX[2]&(0x80))>>7;	

	if(inter_sign == 1) {									  // Negative Temperature
		Temp = (DATARX[2] << 4) | (DATARX[3] >> 4);
		Temp &= (0xFFF);
		Temp ^= (0xFFF);
		internal_temperature = ((double)-Temp / 16);
	} else {                                                  // Positive Temperature
		Temp = (DATARX[2] << 4) | (DATARX[3] >> 4);
		internal_temperature = ((double)Temp / 16);
	}    

	if(DATARX[3] & 0x07)								  // Returns Error Number
		return(-1*(DATARX[3] & 0x07));

	else if(sign==1){									  // Negative Temperature
		Temp = (DATARX[0] << 6) | (DATARX[1] >> 2);
		Temp &= (0x3FFF);
		Temp ^= (0x3FFF);
		return((double)-Temp/4);
	}

	else												  // Positive Temperature
	{
		Temp = (DATARX[0] << 6) | (DATARX[1] >> 2);
		return((double)Temp / 4);
	}
}

