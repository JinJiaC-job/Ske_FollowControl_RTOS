#include "adc.h"
#include "stdio.h"
#include "Sensors_reading.h"

float ADC_Pressure_Value;


void pressure_SensorReading(void)
{
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,10);
//	printf(" adc start read!\r\n");
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
	{
		ADC_Pressure_Value = HAL_ADC_GetValue(&hadc1);
	}
	ADC_Pressure_Value = HAL_ADC_GetValue(&hadc1);
	printf(" LinearActuator_Pressure = %.3fV \r\n", ADC_Pressure_Value*3.3/4096);
//	printf(" ADC channel_0 value = %.3fV \r\n", ADC_Pressure_Value*3.3/4096);
//	ADC_Pressure_Value = ADC_Pressure_Value*3.3/4096;
	
}


