/*
 * noisegate.h
 *
 *  Created on: Dec 8, 2024
 *      Author: Phil's Lab
 */

#pragma once
#include <math.h>

typedef struct{
    float threshold;
    float holdTimeS;

    float attackCoeff;
    float releaseCoeff;

    float attackCounter;
    float releaseCounter;

    float smoothedGain;

    float sampleTimeS;
} DDFX_NoiseGate;

void DDFX_NoiseGate_Init(DDFX_NoiseGate *ng, float threshold, float attackTimeMs, float releaseTimeMs, float holdTimeMs, float sampleRateHz);
float DDFX_NoiseGate_Update(DDFX_NoiseGate *ng, float input);
void DDFX_NoiseGate_SetThreshold(DDFX_NoiseGate *ng, float thresholddB);
void DDFX_NoiseGate_SetAttackReleaseTime(DDFX_NoiseGate *ng, float attackTimeMs, float releaseTimeMs, float sampleRateHz);
