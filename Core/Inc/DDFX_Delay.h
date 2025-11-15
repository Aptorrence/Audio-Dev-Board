/*
 * DDFX_Delay.h
 *
 *  Created on: Nov 26, 2024
 *      Author: brasi
 */
#pragma once

//#include "stm32h7xx_hal.h" // include hardware stuff not sure if i need this cuz i included main

#include <stdint.h>

// so just above 1 sec of delay time max at
#define DDFX_DELAY_MAX_LINE_LENGTH 41200 // one second max delay at 44.1kHz frequency

typedef struct {

	//settings
	float mix;
	float feedback;

	// Delay line buffer and index
	float line[DDFX_DELAY_MAX_LINE_LENGTH];
	uint32_t lineIndex;

	// Delay Line Length
	uint32_t lineLength;

	// Output
	float out;


}DDFX_Delay ;

void	 DDFX_Delay_Init(DDFX_Delay *dly, float delayTime_ms, float mix, float feedback, float sampleRate_hz);
float	 DDFX_Delay_Update(DDFX_Delay *dly, float inp);
void 	 DDFX_Delay_SetLength(DDFX_Delay *dly, float delayTime_ms, float sampleRate_hz);

