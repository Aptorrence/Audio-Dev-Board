#include "DDFX_Delay.h"

void DDFX_Delay_Init(DDFX_Delay *dly, float delayTime_ms, float mix, float feedback, float sampleRate_hz){

	// Set Delay Length
	DDFX_Delay_SetLength(dly, delayTime_ms, sampleRate_hz);

	// Store delay setting
	dly->mix = mix;
	dly->feedback = feedback;

	// clear line circular buffer, reset index
	dly->lineIndex = 0;

	for (uint32_t n = 0; n < DDFX_DELAY_MAX_LINE_LENGTH; n++){
		 dly->line[n] = 0.0f;
	}

	// Clear output
	dly->out = 0.0f;

}

float DDFX_Delay_Update(DDFX_Delay *dly, float inp){

	// get current delay output
	float delayLineOutput = dly->line[dly->lineIndex];

	// compute current delay line input
	float delayLineInput = inp + dly->feedback * delayLineOutput;

	// store in line delay circular buffer
	dly->line[dly->lineIndex] = delayLineInput;

	// Increment delay line index
	dly->lineIndex++;
	if (dly->lineIndex >= dly->lineLength){

		dly->lineIndex = 0;

	}

	// mix Dry and wet signals
	dly->out = (1.0f - dly->mix) * inp + dly->mix * delayLineOutput;
	if (dly->out > 1.0f) {

		dly->out = 1.0f;

	} else if (dly->out < -1.0f){

		dly->out = -1.0f;

	}

	// return current output
	return dly->out;

}

void DDFX_Delay_SetLength(DDFX_Delay *dly, float delayTime_ms, float sampleRate_hz){

	dly->lineLength = (uint32_t) (0.001f * delayTime_ms *sampleRate_hz);

	if (dly->lineLength > DDFX_DELAY_MAX_LINE_LENGTH){

		dly->lineLength = DDFX_DELAY_MAX_LINE_LENGTH;

	}

}
