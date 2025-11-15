#include "DDFX_Tremolo.h"

void DDFX_Tremolo_Init(DDFX_Tremolo *trem, float mix, float lfoFrequency_Hz, float sampleRate_Hz){

	/*
	 * Store Tremolo Depth
	 */
	DDFX_Tremolo_SetMix(trem, mix);

	/*
	 * Store sample rate and compute LFO parameters
	 */
	trem->sampleRate_Hz = sampleRate_Hz;
	DDFX_Tremolo_SetLFO_Freq(trem, lfoFrequency_Hz);

	/*
	 * Clear LFO counter and set LFO 'direction'
	 */
	trem->lfoCount = 0;
	trem->lfoDir = 1;

	/*
	 * Clear output
	 */
	trem->out = 0.0f;

}

void DDFX_Tremolo_SetMix(DDFX_Tremolo *trem, float mix){

	/*
	 * Check depth bounds
	 */
	if ( mix < 0.0f ){

		mix = 0.0f;

	} else if ( mix > 1.0f ){

		mix = 1.0f;

	}

	trem->mix = mix;

}

void DDFX_Tremolo_SetLFO_Freq(DDFX_Tremolo *trem, float lfoFrequency_Hz){

	/*
	 * Check LFO frequency bounds
	 */
	if ( lfoFrequency_Hz <= 0.0f ) {

		lfoFrequency_Hz = 1.0f;

	} else if ( lfoFrequency_Hz > 0.5f * trem->sampleRate_Hz){

		lfoFrequency_Hz = 0.5f * trem->sampleRate_Hz;

	}

	/*
	 * compute counter limit based on desired FLO frequency (counterLimit = (Fs/F_lfo)/4
	 */
	trem->lfoCountLimit = 0.25f * (trem->sampleRate_Hz / lfoFrequency_Hz);

	/*
	 * Ensure LFO counter is within new counter  limit range
	 */
	if ( trem->lfoCount > trem->lfoCountLimit ) {

		trem->lfoCount = trem->lfoCountLimit;

	} else if ( trem->lfoCount < -trem->lfoCountLimit){

		trem->lfoCount = -trem->lfoCountLimit;

	}

}

float DDFX_Tremolo_Update(DDFX_Tremolo *trem, float inp){

	/*
	 *  Modulate input signal: y[n] = x[n] * ( (1-m) + m * g[n] )
	 */
	trem->out = inp * ( ( 1.0f - trem->mix) + trem->mix * (trem->lfoCount / trem->lfoCountLimit));

	/*
	 *  If we've reached a maximum or minimum of the triangle-wave, count down
	 */
	if ( trem->lfoCount >= trem->lfoCountLimit ) {

		trem->lfoDir = -1.0f;

	} else if ( trem->lfoCount <= -trem->lfoCountLimit ){

		trem->lfoDir = +1.0f;

	}

	/*
	 * Increment LFO counter
	 */
	trem->lfoCount += trem->lfoDir;

	/*
	 * Return output
	 */
	return (trem->out);
}

