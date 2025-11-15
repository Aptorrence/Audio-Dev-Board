/*
 * noisegate.c
 *
 *  Created on: Dec 8, 2024
 *      Author: Phil's Lab
 */
#include "noisegate.h"

void DDFX_NoiseGate_Init(DDFX_NoiseGate *ng, float threshold, float attackTimeMs, float releaseTimeMs, float holdTimeMs, float sampleRateHz){

	// Store settings in struct
	DDFX_NoiseGate_SetThreshold(ng, threshold);
	ng->holdTimeS = 0.001f * holdTimeMs;

	// Calculate attack and release coefficients
	DDFX_NoiseGate_SetAttackReleaseTime(ng, attackTimeMs, releaseTimeMs, sampleRateHz);

	// Store sample time
	ng->sampleTimeS = 1.0f / sampleRateHz;

	// Reset counters
	ng->attackCounter  = 0.0f;
	ng->releaseCounter = 0.0f;

	// Reset smoothed gain value
	ng->smoothedGain = 0.0f;

}


float DDFX_NoiseGate_Update(DDFX_NoiseGate *ng, float input){

	// [1] Get magnitude of input
	float inputAbs = fabsf(input);

	// [2] Gain computer (static gain characteristic)
	float gain = 1.0f;

	if (inputAbs < ng->threshold) { // Input is below noise gate threshold, does not let signal pass

		gain = 0.0f;

	}

	// [3] Gain smoothing
	if (gain <= ng->smoothedGain){ // Attack

		if (ng->attackCounter > ng->holdTimeS){

			// Attack (decrease gain -> reducing output as noise gate engages
			ng->smoothedGain = ng->attackCoeff * ng->smoothedGain + (1.0f - ng->attackCoeff) * gain;

		} else { // Haven;t reached hold time yet

			ng->attackCounter += ng->sampleTimeS;

		}
	} else if (gain > ng->smoothedGain) { // Release

		// Release (increasing gain -> increasing output as noise gate disengageous)
		ng->smoothedGain = ng->releaseCoeff * ng->smoothedGain + (1.0f - ng->releaseCoeff) * gain;

		// Reset attack counter since we're in release stage
		ng->attackCounter = 0.0f;

	}

	return (input * ng->smoothedGain);

}


void DDFX_NoiseGate_SetThreshold(DDFX_NoiseGate *ng, float threshold){

	// Limit threshold value to meaningful range
	if (threshold > 1.0f){

		threshold = 1.0f;

	}else if (threshold < 0.0f){

		threshold = 0.0f;

	}

	ng->threshold = threshold;

}

void DDFX_NoiseGate_SetAttackReleaseTime(DDFX_NoiseGate *ng, float attackTimeMs, float releaseTimeMs, float sampleRateHz){
	ng->attackCoeff  = expf(-2197.22457734f / (sampleRateHz * attackTimeMs));
	ng->releaseCoeff = expf(-2197.22457734f / (sampleRateHz * releaseTimeMs));
}
