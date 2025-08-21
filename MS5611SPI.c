/*
	Based on
   MS5611-02 SPI library for ARM STM32F103xx Microcontrollers - Main source file
   05/01/2020 by Joao Pedro Vilas <joaopedrovbs@gmail.com>
   Changelog:
     2012-05-23 - initial release.
*/
/* ============================================================================================
 * MS5611SPI.c
 *
 * Created on: Feb 02, 2025
 * Author: Nathan Netzel
 *
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2025 Nathan Netzel
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the conditions of the MIT License.
 * ============================================================================================
 */

#include <MS5611SPI.h>

/* Private PROM data structure */
static struct promData promData;

/**
 * @brief  Initializes the MS5611 sensor and reads PROM calibration values
 * @note   Performs a reset and reads the PROM to verify communication
 * @param  MS5611_Handler Pointer to the MS5611_HW_InitTypeDef structure
 * @retval MS5611StateTypeDef Current state of the sensor
 */
MS5611StateTypeDef MS5611_Init(MS5611_HW_InitTypeDef *MS5611_Handler) {

	uint8_t SPITransmitData;

	enableCS_MS5611(MS5611_Handler->CS_GPIOport, MS5611_Handler->CS_GPIOpin);
	SPITransmitData = RESET_COMMAND;

	if(HAL_SPI_Transmit(MS5611_Handler->SPIhandler, &SPITransmitData, 1, 10) != HAL_OK)
		{
		disableCS_MS5611(MS5611_Handler->CS_GPIOport, MS5611_Handler->CS_GPIOpin);
			return MS5611_HAL_ERROR;
	}

	HAL_Delay(3);
	disableCS_MS5611(MS5611_Handler->CS_GPIOport, MS5611_Handler->CS_GPIOpin);

	MS5611PromRead(MS5611_Handler, &promData);

	if (promData.off == 0x00 || promData.tref == 0xff)
		return MS5611_STATE_FAILED;
	else
		return MS5611_STATE_READY;
}

/**
 * @brief  Reads calibration coefficients from PROM
 * @param  MS5611_Handler Pointer to the MS5611_HW_InitTypeDef structure
 * @param  prom Pointer to the promData structure to store calibration values
 * @retval MS5611StateTypeDef Current state of the sensor
 */
MS5611StateTypeDef MS5611PromRead(MS5611_HW_InitTypeDef *MS5611_Handler, struct promData *prom){
	uint8_t   address;
	uint16_t  *structPointer;
	uint8_t SPITransmitData;

	structPointer = (uint16_t *) prom;

	for (address = 0; address < 8; address++) {
		SPITransmitData = PROM_READ(address);
		enableCS_MS5611(MS5611_Handler->CS_GPIOport, MS5611_Handler->CS_GPIOpin);

		if(HAL_SPI_Transmit(MS5611_Handler->SPIhandler, &SPITransmitData, 1, 10) != HAL_OK)
		{
			disableCS_MS5611(MS5611_Handler->CS_GPIOport, MS5611_Handler->CS_GPIOpin);
			return MS5611_HAL_ERROR;
		}

		if(HAL_SPI_Receive(MS5611_Handler->SPIhandler, structPointer, 2, 10) != HAL_OK)
		{
			disableCS_MS5611(MS5611_Handler->CS_GPIOport, MS5611_Handler->CS_GPIOpin);
			return MS5611_HAL_ERROR;
		}

		disableCS_MS5611(MS5611_Handler->CS_GPIOport, MS5611_Handler->CS_GPIOpin);
		structPointer++;
	}

	structPointer = (uint16_t *) prom;
	for (address = 0; address < 8; address++) {
		uint8_t   *toSwap = (uint8_t *) structPointer;
		uint8_t secondByte = toSwap[0];
		toSwap[0] = toSwap[1];
		toSwap[1] = secondByte;
		structPointer++;
	}

	return MS5611_STATE_READY;
}

/**
 * @brief  Initiates an uncompensated pressure (D1) conversion
 * @param  MS5611_Handler Pointer to the MS5611_HW_InitTypeDef structure
 * @param  MS5611_Press_OSR Oversampling setting for pressure
 * @retval MS5611StateTypeDef Current state of the sensor (BUSY or ERROR)
 */
MS5611StateTypeDef MS5611_Pressure_Conversion(MS5611_HW_InitTypeDef *MS5611_Handler, uint8_t MS5611_Press_OSR){

	uint8_t SPITransmitData;
	enableCS_MS5611(MS5611_Handler->CS_GPIOport, MS5611_Handler->CS_GPIOpin);

	SPITransmitData = CONVERT_D1_COMMAND | MS5611_Press_OSR;
	if(HAL_SPI_Transmit(MS5611_Handler->SPIhandler, &SPITransmitData, 1, 10) != HAL_OK)
	{
		disableCS_MS5611(MS5611_Handler->CS_GPIOport, MS5611_Handler->CS_GPIOpin);
		return MS5611_HAL_ERROR;
	}
	disableCS_MS5611(MS5611_Handler->CS_GPIOport, MS5611_Handler->CS_GPIOpin);

	return MS5611_STATE_BUSY;
}

/**
 * @brief  Initiates an uncompensated temperature (D2) conversion
 * @param  MS5611_Handler Pointer to the MS5611_HW_InitTypeDef structure
 * @param  MS5611_Temp_OSR Oversampling setting for temperature
 * @retval MS5611StateTypeDef Current state of the sensor (BUSY or ERROR)
 */
MS5611StateTypeDef MS5611_Temperature_Conversion(MS5611_HW_InitTypeDef *MS5611_Handler, uint8_t MS5611_Temp_OSR){

	uint8_t SPITransmitData;

	enableCS_MS5611(MS5611_Handler->CS_GPIOport, MS5611_Handler->CS_GPIOpin);
	SPITransmitData = CONVERT_D2_COMMAND | MS5611_Temp_OSR;
	if(HAL_SPI_Transmit(MS5611_Handler->SPIhandler, &SPITransmitData, 1, 10) != HAL_OK)
	{
		disableCS_MS5611(MS5611_Handler->CS_GPIOport, MS5611_Handler->CS_GPIOpin);
		return MS5611_HAL_ERROR;
	}

  	disableCS_MS5611(MS5611_Handler->CS_GPIOport, MS5611_Handler->CS_GPIOpin);

  	return MS5611_STATE_BUSY;
}

/**
 * @brief  Reads the ADC result from the sensor
 * @param  MS5611_Handler Pointer to the MS5611_HW_InitTypeDef structure
 * @param  raw_data Pointer to store the 24-bit raw ADC value
 * @retval MS5611StateTypeDef Current state of the sensor (READY or ERROR)
 */
MS5611StateTypeDef MS5611_ADC_Read(MS5611_HW_InitTypeDef *MS5611_Handler, uint32_t *raw_data){

	uint8_t SPITransmitData;
	uint8_t reply[3];

	enableCS_MS5611(MS5611_Handler->CS_GPIOport, MS5611_Handler->CS_GPIOpin);

	SPITransmitData = READ_ADC_COMMAND;
	if(HAL_SPI_Transmit(MS5611_Handler->SPIhandler, &SPITransmitData, 1, 10) != HAL_OK)
	{
		disableCS_MS5611(MS5611_Handler->CS_GPIOport, MS5611_Handler->CS_GPIOpin);
		return MS5611_HAL_ERROR;
	}
	if(HAL_SPI_Receive(MS5611_Handler->SPIhandler, reply, 3, 10) != HAL_OK)
	{
		disableCS_MS5611(MS5611_Handler->CS_GPIOport, MS5611_Handler->CS_GPIOpin);
		return MS5611_HAL_ERROR;
	}

	disableCS_MS5611(MS5611_Handler->CS_GPIOport, MS5611_Handler->CS_GPIOpin);

	*raw_data = ((uint32_t) reply[0] << 16) | ((uint32_t) reply[1] << 8) | (uint32_t) reply[2];

	return MS5611_STATE_READY;
}


/**
 * @brief  Converts raw ADC data to compensated pressure and temperature
 * @param  sample Pointer to MS5611_Raw_Data_TypeDef structure
 * @param  value Pointer to MS5611_Converted_Data_TypeDef structure to store results
 * @retval None
 */
void MS5611_Data_Convert(MS5611_Raw_Data_TypeDef *sample, MS5611_Converted_Data_TypeDef *value){
	int32_t dT;
	int32_t TEMP;
	int64_t OFF;
	int64_t SENS;

	dT = sample->temperature - ((int32_t) (promData.tref << 8));

	TEMP = 2000 + (((int64_t) dT * promData.tempsens) >> 23);

	OFF = ((int64_t) promData.off << 16) + (((int64_t) promData.tco * dT) >> 7);
	SENS = ((int64_t) promData.sens << 15) + (((int64_t) promData.tcs * dT) >> 8);


	if (TEMP < 2000) {
		int32_t T2 = ((int64_t) dT * (int64_t) dT) >> 31;
		int32_t TEMPM = TEMP - 2000;
		int64_t OFF2 = (5 * (int64_t) TEMPM * (int64_t) TEMPM) >> 1;
		int64_t SENS2 = (5 * (int64_t) TEMPM * (int64_t) TEMPM) >> 2;
		if (TEMP < -1500) {
			int32_t TEMPP = TEMP + 1500;
			int32_t TEMPP2 = TEMPP * TEMPP;
			OFF2 = OFF2 + (int64_t) 7 * TEMPP2;
			SENS2 = SENS2 + (((int64_t) 11 * TEMPP2) >> 1);
		}
		TEMP -= T2;
		OFF -= OFF2;
		SENS -= SENS2;
	}

	value->pressure = ((((int64_t) sample->pressure * SENS) >> 21) - OFF) >> 15;
	value->temperature = TEMP;

}

/**
 * @brief  Enables the chip select pin for the MS5611 sensor
 * @param  CS_GPIOport Chip select GPIO port address
 * @param  CS_GPIOpin Chip select GPIO pin number
 * @retval None
 */
void enableCS_MS5611(GPIO_TypeDef *CS_GPIOport, uint16_t CS_GPIOpin){
  HAL_GPIO_WritePin(CS_GPIOport, CS_GPIOpin, GPIO_PIN_RESET);
}

/**
 * @brief  Disables the chip select pin for the MS5611 sensor
 * @param  CS_GPIOport Chip select GPIO port address
 * @param  CS_GPIOpin Chip select GPIO pin number
 * @retval None
 */
void disableCS_MS5611(GPIO_TypeDef *CS_GPIOport, uint16_t CS_GPIOpin){
  HAL_GPIO_WritePin(CS_GPIOport, CS_GPIOpin, GPIO_PIN_SET);
}

