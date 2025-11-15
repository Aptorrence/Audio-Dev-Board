/*
 * fuzz.c
 *
 *  Created on: Dec 5, 2024
 *      Author: Alejandro Esparza
 */
#include <math.h>
#include <stdint.h>
#include "fuzz.h"

/*
#define X 0.2f // Threshold for low-gain
#define Y 0.9f // Clipping value
#define M 1f // Modulating parameter
#define GRADIENT (Y/X)
*/

void fuzz_init(DDFX_FUZZ *fuzz, float threshold, float clip, float crunch){
    fuzz->threshold = threshold;
    fuzz->clip = clip;
    fuzz->crunch = crunch;
}

/*
float applyFuzz(DDFX_FUZZ *fuzz, float input) {
    float output;

    output = fuzz->crunch * input;
    output = tanh(output);

    // Saturation
    if (output > fuzz->clip) {
        output = fuzz->clip;
    } else if (output < -fuzz->clip) {
        output = -fuzz->clip;
    }

    return output;
}
*/


float applyFuzz(DDFX_FUZZ *fuzz, float input) {
    float output;
    float GRADIENT = fuzz->clip / fuzz->threshold;

    // Apply fuzz based on input amplitude
    if ((input > -fuzz->threshold) && (input < fuzz->threshold)) {
        // Low-amplitude region: linear gain
        output = GRADIENT * fuzz->crunch * input;
    } else if (input < 0) {
        // Large negative input: clip to -Y
        output = -fuzz->clip;
    } else {
        // Large positive input: clip to Y
        output = fuzz->clip;
    }

    // Saturation
    if (output > fuzz->clip) {
        output = fuzz->clip;
    } else if (output < -fuzz->clip) {
        output = -fuzz->clip;
    }

    return output;
}
