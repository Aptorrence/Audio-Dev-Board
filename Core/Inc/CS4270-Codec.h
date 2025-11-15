/*
 * CS4270-Codec.h
 *
 *  Created on: Nov 16, 2024
 *      Author: brasi
 */

#pragma once //Gotta ask what this dose...

#include "stm32h7xx_hal.h" // include hardware stuff not sure if i need this cuz i included main

//I2c address
#define CS4270_I2C_ADDRESS				(0X48 << 1) // 0B{1001 A2 A1 A0}, A{2:0} = 0 --> 0b1001000 = 0x48

// Device ID
#define CS4270_DEVICEID					0xC0

// Register addresses
#define CS4270_REG_DEVICEID				0x01
#define CS4270_REG_POWERCONTROL			0x02

#define CS4270_REG_MODECONTROL			0x03
#define CS4270_REG_ADCDACCONTROL		0x04
#define CS4270_REG_TRANSISTIONCONTROL	0x05
#define CS4270_REG_MUTECONTROL			0x06
#define CS4270_REG_DACAVOLCONTROL		0x07
#define CS4270_REG_DACBVOLCONTROL		0x08

//register config (default, set on init())
extern uint8_t CS4270_REG_CONFIG_SETTINGS[8];


//functions
uint8_t CS4270_Init(I2C_HandleTypeDef * i2cInstance, GPIO_TypeDef * nrstPort, uint16_t nrstPin);
void CS4270_Reset();

HAL_StatusTypeDef CS4270_RegWrite(uint8_t regAddr, uint8_t regData);
HAL_StatusTypeDef CS4270_RegRead(uint8_t regAddr, uint8_t *regData);
