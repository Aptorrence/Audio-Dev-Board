

#pragma once

#include <stdint.h>
#include <math.h>

typedef struct {

	/*
	 * Tremolo Mix
	 */
	float mix;

	/*
	 * Low-frequency oscilator (LFO) parameters
	 */
	float lfoDir;
	float lfoCount;
	float lfoCountLimit;

	/*
	 * Effect sample rate (Hz)
	 */
	float sampleRate_Hz;

	/*
	 * Effect output
	 */
	float out;


} DDFX_Tremolo;

void DDFX_Tremolo_Init(DDFX_Tremolo *trem, float mix, float lfoFrequency_Hz, float sampleRate_Hz);

void DDFX_Tremolo_SetMix(DDFX_Tremolo *trem, float mix);
void DDFX_Tremolo_SetLFO_Freq(DDFX_Tremolo *trem, float lfoFrequency_Hz);

float DDFX_Tremolo_Update(DDFX_Tremolo *trem, float inp);
