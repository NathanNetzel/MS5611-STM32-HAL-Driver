/*
	Based on
   MS5611-02 SPI library for ARM STM32F103xx Microcontrollers - Main source file
   05/01/2020 by Joao Pedro Vilas <joaopedrovbs@gmail.com>
   Changelog:
     2012-05-23 - initial release.
*/
/* ============================================================================================
 * MS5611SPI.h
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

#ifndef _MS5611SPI_H_
#define _MS5611SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h5xx_hal.h"

// --- MS5611 SPI Commands ---
#define RESET_COMMAND                 0x1E
#define PROM_READ(address)            (0xA0 | ((address) << 1))   /**< Macro to access 8 PROM addresses */
#define CONVERT_D1_COMMAND            0x40                        /**< Start pressure conversion */
#define CONVERT_D2_COMMAND            0x50                        /**< Start temperature conversion */
#define READ_ADC_COMMAND              0x00                        /**< Read ADC result */

// --- Oversampling Ratios ---
#define MS5611_OSR_256		0x00
#define MS5611_OSR_512		0x02
#define MS5611_OSR_1024		0x04
#define MS5611_OSR_2048		0x06
#define MS5611_OSR_4096		0x08

// --- MS5611 System States ---
typedef enum MS5611States{
  MS5611_STATE_FAILED,  /**< Sensor initialization or communication failed */
  MS5611_STATE_READY,   /**< Sensor ready for use */
  MS5611_STATE_BUSY,    /**< Sensor performing conversion */
  MS5611_HAL_ERROR      /**< HAL communication error */
}MS5611StateTypeDef;

// --- PROM Data Structure ---
struct promData{
  uint16_t reserved;
  uint16_t sens;
  uint16_t off;
  uint16_t tcs;
  uint16_t tco;
  uint16_t tref;
  uint16_t tempsens;
  uint16_t crc;
};

// --- Raw Sensor Values ---
typedef struct MS5611UncompensatedValues{
  uint32_t pressure;     /**< Uncompensated pressure */
  uint32_t temperature;  /**< Uncompensated temperature */
} MS5611_Raw_Data_TypeDef;

// --- Compensated Sensor Values ---
typedef struct MS5611Readings{
  int32_t pressure;      /**< Compensated pressure */
  int32_t temperature;   /**< Compensated temperature */
} MS5611_Converted_Data_TypeDef;

// --- Hardware Initialization Structure ---
typedef struct {
	SPI_HandleTypeDef *SPIhandler;  /**< Pointer to SPI handler */
	GPIO_TypeDef *CS_GPIOport;      /**< GPIO port for chip select */
	uint16_t CS_GPIOpin;            /**< GPIO pin number for chip select */
	uint8_t SPI_Timeout;            /**< SPI timeout in milliseconds */
} MS5611_HW_InitTypeDef;

// --- Function Prototypes ---

/**
 * @brief  Initializes MS5611 Sensor
 * @param  MS5611_Handler Pointer to hardware initialization structure
 * @retval MS5611StateTypeDef Initialization status
 */
MS5611StateTypeDef MS5611_Init(MS5611_HW_InitTypeDef *);

/**
 * @brief  Reads MS5611 PROM content
 * @note   Must be called only during initialization
 * @param  MS5611_Handler Pointer to hardware initialization structure
 * @param  prom Pointer to PROM data structure
 * @retval MS5611StateTypeDef Status of read
 */
MS5611StateTypeDef MS5611PromRead(MS5611_HW_InitTypeDef *, struct promData *prom);

/**
 * @brief  Initiates an uncompensated pressure (D1) conversion
 * @param  MS5611_Handler Pointer to hardware initialization structure
 * @param  MS5611_Press_OSR Oversampling ratio for pressure conversion
 * @retval MS5611StateTypeDef Status after starting conversion
 */
MS5611StateTypeDef MS5611_Pressure_Conversion(MS5611_HW_InitTypeDef *, uint8_t);

/**
 * @brief  Initiates an uncompensated temperature (D2) conversion
 * @param  MS5611_Handler Pointer to hardware initialization structure
 * @param  MS5611_Temp_OSR Oversampling ratio for temperature conversion
 * @retval MS5611StateTypeDef Status after starting conversion
 */
MS5611StateTypeDef MS5611_Temperature_Conversion(MS5611_HW_InitTypeDef *, uint8_t);

/**
 * @brief  Reads ADC result from MS5611 sensor
 * @param  MS5611_Handler Pointer to hardware initialization structure
 * @param  raw_data Pointer to store 24-bit raw ADC result
 * @retval MS5611StateTypeDef Status after read
 */
MS5611StateTypeDef MS5611_ADC_Read(MS5611_HW_InitTypeDef *, uint32_t *);

/**
 * @brief  Converts raw sensor values to compensated values using PROM calibration
 * @param  sample Pointer to raw data structure
 * @param  value Pointer to converted data structure
 */
void MS5611_Data_Convert(MS5611_Raw_Data_TypeDef *sample, MS5611_Converted_Data_TypeDef *value);

/**
 * @brief  Enables the chip select pin for SPI communication
 * @param  CS_GPIOport GPIO port of the CS pin
 * @param  CS_GPIOpin GPIO pin number
 */
void enableCS_MS5611(GPIO_TypeDef *CS_GPIOport, uint16_t CS_GPIOpin);

/**
 * @brief  Disables the chip select pin for SPI communication
 * @param  CS_GPIOport GPIO port of the CS pin
 * @param  CS_GPIOpin GPIO pin number
 */
void disableCS_MS5611(GPIO_TypeDef *CS_GPIOport, uint16_t CS_GPIOpin);

#endif /* _MS5611SPI_H_ */
