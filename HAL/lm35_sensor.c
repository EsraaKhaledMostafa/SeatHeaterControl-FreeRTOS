/*
 *  Module: LM35 temperature Sensor
 *  File Name: lm35.c
 *  Description: Source file for the LM35 temp sensor driver
 *  Author: Esraa Khaled
 */


#include "lm35_sensor.h"
#include "MCAL/ADC/adc.h"


uint8 LM35_getTemperature(void)
{
	uint8 temperature = 0;

	ADC_readValue();

	/* Make delay for some time until g_adcResult value is updated with the ADC interrupt */
	Delay_MS(10);

	temperature = (uint8)(((uint32)adcResult*LM35_MAX_TEMPERATURE*ADC_REF_VOLTAGE)/(ADC_MAX_VALUE*LM35_MAX_VOLT));

	return temperature;
}
