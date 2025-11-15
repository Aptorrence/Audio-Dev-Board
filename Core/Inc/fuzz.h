/*
 * fuzz.h
 *
 *  Created on: Dec 5, 2024
 *      Author: Alejandro Esparza
 */
#pragma once

typedef struct{

    // Threshold
    float threshold;

    // Clipping value
    float clip;

    // Modulating Parameter (Crunch)
    float crunch;

}DDFX_FUZZ;

float applyFuzz(DDFX_FUZZ *fuzz, float input);

void fuzz_init(DDFX_FUZZ *fuzz, float threshold, float clip, float crunch);
