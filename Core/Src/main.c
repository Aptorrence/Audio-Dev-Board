/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "usbd_cdc_if.h"
#include "CS4270-Codec.h"
#include "DDFX_Delay.h"
#include "DDFX_Tremolo.h"
#include "noisegate.h"
#include "fuzz.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AUDIO_BUFFER_SIZE 		256
#define SAMPLEING_FREQUENCY 	41100 // 41.1k Hz sampling frequency
#define THRESHOLD 				0x7FFFFF00 // Just below maximum 32-bit value

#define ENCODER_MAX_COUNT 		UINT16_MAX // maximum encoder count before overflow
#define CONTROL_SETTING_COUNT 	9 // the number of Pot values we can read from

#define TREM_MIX_SCALE 			1.0f
#define TREM_LFO_FREQ_MIN 		0.5f
#define TREM_LFO_FREQ_SCALE		10.0f


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockB1;
DMA_HandleTypeDef hdma_sai1_a;
DMA_HandleTypeDef hdma_sai1_b;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

// Lookup table with 249 values from 0 to 1 (mapping 7 to 255)
static const float adcLUT[256] = {
	0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f,
    0.0000f, 0.0040f, 0.0081f, 0.0121f, 0.0161f, 0.0202f, 0.0242f, 0.0282f,
    0.0323f, 0.0363f, 0.0403f, 0.0444f, 0.0484f, 0.0524f, 0.0565f, 0.0605f,
    0.0645f, 0.0685f, 0.0726f, 0.0766f, 0.0806f, 0.0847f, 0.0887f, 0.0927f,
    0.0968f, 0.1008f, 0.1048f, 0.1089f, 0.1129f, 0.1169f, 0.1210f, 0.1250f,
    0.1290f, 0.1331f, 0.1371f, 0.1411f, 0.1452f, 0.1492f, 0.1532f, 0.1573f,
    0.1613f, 0.1653f, 0.1694f, 0.1734f, 0.1774f, 0.1815f, 0.1855f, 0.1895f,
    0.1936f, 0.1976f, 0.2016f, 0.2056f, 0.2097f, 0.2137f, 0.2177f, 0.2218f,
    0.2258f, 0.2298f, 0.2339f, 0.2379f, 0.2419f, 0.2460f, 0.2500f, 0.2540f,
    0.2581f, 0.2621f, 0.2661f, 0.2702f, 0.2742f, 0.2782f, 0.2823f, 0.2863f,
    0.2903f, 0.2944f, 0.2984f, 0.3024f, 0.3065f, 0.3105f, 0.3145f, 0.3185f,
    0.3226f, 0.3266f, 0.3306f, 0.3347f, 0.3387f, 0.3427f, 0.3468f, 0.3508f,
    0.3548f, 0.3589f, 0.3629f, 0.3669f, 0.3710f, 0.3750f, 0.3790f, 0.3831f,
    0.3871f, 0.3911f, 0.3952f, 0.3992f, 0.4032f, 0.4073f, 0.4113f, 0.4153f,
    0.4194f, 0.4234f, 0.4274f, 0.4315f, 0.4355f, 0.4395f, 0.4435f, 0.4476f,
    0.4516f, 0.4556f, 0.4597f, 0.4637f, 0.4677f, 0.4718f, 0.4758f, 0.4798f,
    0.4839f, 0.4879f, 0.4919f, 0.4960f, 0.5000f, 0.5040f, 0.5081f, 0.5121f,
    0.5161f, 0.5202f, 0.5242f, 0.5282f, 0.5323f, 0.5363f, 0.5403f, 0.5444f,
    0.5484f, 0.5524f, 0.5565f, 0.5605f, 0.5645f, 0.5685f, 0.5726f, 0.5766f,
    0.5806f, 0.5847f, 0.5887f, 0.5927f, 0.5968f, 0.6008f, 0.6048f, 0.6089f,
    0.6129f, 0.6169f, 0.6210f, 0.6250f, 0.6290f, 0.6331f, 0.6371f, 0.6411f,
    0.6452f, 0.6492f, 0.6532f, 0.6573f, 0.6613f, 0.6653f, 0.6694f, 0.6734f,
    0.6774f, 0.6815f, 0.6855f, 0.6895f, 0.6935f, 0.6976f, 0.7016f, 0.7056f,
    0.7097f, 0.7137f, 0.7177f, 0.7218f, 0.7258f, 0.7298f, 0.7339f, 0.7379f,
    0.7419f, 0.7460f, 0.7500f, 0.7540f, 0.7581f, 0.7621f, 0.7661f, 0.7702f,
    0.7742f, 0.7782f, 0.7823f, 0.7863f, 0.7903f, 0.7944f, 0.7984f, 0.8024f,
    0.8065f, 0.8105f, 0.8145f, 0.8185f, 0.8226f, 0.8266f, 0.8306f, 0.8347f,
    0.8387f, 0.8427f, 0.8468f, 0.8508f, 0.8548f, 0.8589f, 0.8629f, 0.8669f,
    0.8710f, 0.8750f, 0.8790f, 0.8831f, 0.8871f, 0.8911f, 0.8952f, 0.8992f,
    0.9032f, 0.9073f, 0.9113f, 0.9153f, 0.9194f, 0.9234f, 0.9274f, 0.9315f,
    0.9355f, 0.9395f, 0.9435f, 0.9476f, 0.9516f, 0.9556f, 0.9597f, 0.9637f,
    0.9677f, 0.9718f, 0.9758f, 0.9798f, 0.9839f, 0.9879f, 0.9919f, 0.9960f,
    1.0000f
};

/* Buffer Declares*/
int32_t adcBuf[AUDIO_BUFFER_SIZE];
int32_t dacBuf[AUDIO_BUFFER_SIZE];

static volatile int32_t *inBufPtr;
static volatile int32_t *outBufPtr = &dacBuf[0];

uint8_t dataReadyFlag; // flag for checking if the buffers are half full ready to process
uint8_t controlSettingLocked; // lock control if it is ready to process or hasn't been 1/50 of a second

/* User Controls Declares */
uint8_t 		potValue[9] 	= {0};
volatile float	volume 			= 0.00f;
volatile float	perameter1[9]	= {0.00f}; // the 8 potentiometers reading and exp Ped
volatile float	expPed 			= 0.00f;
uint16_t 		encoder_count	= 0; // reads the register of the encoder count
int16_t 		rotEncoder 		= 0; // is the useful non overflow value
uint8_t 		Foot_Sw_1		= 0;
uint8_t			Foot_Sw_2 		= 0;

/* FX Processing Declares */
DDFX_Delay dly;
DDFX_Tremolo trem;
DDFX_NoiseGate ng;
DDFX_FUZZ fuzz;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI3_Init(void);
static void MX_SAI1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Error_LED() {
	  HAL_GPIO_WritePin(GPIOD, LED_Pin, GPIO_PIN_SET);
}


void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
	// Handle half transfer complete
	inBufPtr  = &adcBuf[0];
	outBufPtr = &dacBuf[0];

	dataReadyFlag = 1;
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
    // Handle transfer complete
	inBufPtr  = &adcBuf[AUDIO_BUFFER_SIZE/2];
	outBufPtr = &dacBuf[AUDIO_BUFFER_SIZE/2];

	dataReadyFlag = 1;
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
    // Handle half transfer complete
	inBufPtr  = &adcBuf[0];
	outBufPtr = &dacBuf[0];

	dataReadyFlag = 1;
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
    // Handle transfer complete
	inBufPtr  = &adcBuf[AUDIO_BUFFER_SIZE/2];
	outBufPtr = &dacBuf[AUDIO_BUFFER_SIZE/2];

	dataReadyFlag = 1;
}

uint16_t Read_Encoder(){

	  //Handle overflow
		uint16_t start_encoder_count = TIM1->CNT;
	    int16_t delta_count = TIM1->CNT; ;
	    /*
	     * Checks if the differnece is more or less than half the encoder count
	     * if it is then it subtracks or adds the encoder count to keep it just counting normal
	     */
	    if (delta_count <= -ENCODER_MAX_COUNT / 2) {

	    	delta_count += ENCODER_MAX_COUNT + 1;

	    }

	    else if (delta_count >= ENCODER_MAX_COUNT / 2) {

	    	delta_count -= ENCODER_MAX_COUNT + 1;

	    }

	    encoder_count = start_encoder_count; // sets the encoder_count to the prev value for next time

	    return delta_count;

}


void Process_HalfBuffer(){

		controlSettingLocked = 1;

		// Input samples
		static float leftIn 	= 0.0f;
		static float rightIn	= 0.0f;

		// Output samples
		static float leftOut	= 0.0f;
		static float rightOut	= 0.0f;

		// Loop through half of audio buffer (double buffering), convert int->float,
		for (uint16_t sampleIndex = 0; sampleIndex < (AUDIO_BUFFER_SIZE/2) - 1; sampleIndex += 2){

			/*
			 *  Convert current input samples (24-bits) to floats (two I2S data lines,
			 */

			// Extract 24-bits via bit mask
			inBufPtr[sampleIndex]		&= 0xFFFFFF;
			inBufPtr[sampleIndex +1]	&= 0xFFFFFF;

			// Check if number is negative (sign bit)
			if (inBufPtr[sampleIndex] & 0x800000) {
				inBufPtr[sampleIndex] |= ~0xFFFFFF;
			}

			if (inBufPtr[sampleIndex +1] & 0x80000){
				inBufPtr[sampleIndex +1] |= ~0xFFFFFF;
			}

			// Normalise to float (-1.0 + 1.0)
			leftIn = (float) inBufPtr[sampleIndex]		/(float) (0x7FFFFF);
			rightIn = (float) inBufPtr[sampleIndex + 1]	/(float) (0x7FFFFF);

			leftIn  = DDFX_NoiseGate_Update(&ng, leftIn);
			rightIn = DDFX_NoiseGate_Update(&ng, rightIn);

			leftIn =DDFX_Delay_Update(&dly, leftIn); // volume control multiplied by input


			if (Foot_Sw_1 == 1 && Foot_Sw_2 == 0){

				leftOut  =	volume * applyFuzz(&fuzz, leftIn);
				//leftOut  =	volume * DDFX_Delay_Update(&dly, leftIn); // volume control multiplied by input
				rightOut =  volume * rightIn; //volume control multiplied by input

			} else if(Foot_Sw_1 == 1 && Foot_Sw_2 == 1) {

				leftOut  =	volume * applyFuzz(&fuzz, leftIn);
				leftOut  = volume * DDFX_Tremolo_Update(&trem, leftOut);
				//leftOut  = volume * DDFX_Tremolo_Update(&trem, leftIn);
				//leftOut  = DDFX_Delay_Update(&dly, leftOut); // volume control multiplied by input
				rightOut = volume * rightIn; //volume control multiplied by input

			} else if(Foot_Sw_1 == 0 && Foot_Sw_2 == 1){

				leftOut  = volume * DDFX_Tremolo_Update(&trem, leftIn);
				rightOut = volume * rightIn; //volume control multiplied by input

			} else {

				leftOut = volume  * leftIn; //volume control multiplied by input
				rightOut = volume * rightIn; //volume control multiplied by input
			}
			/*
			 *  Convert floats to 32-bit output samples
			 */

			int32_t scaledLeft = (int32_t)(leftOut * 8388607.0f);

			    // Ensure the result fits within 24 bits
			    scaledLeft &= 0x00FFFFFF;

			int32_t scaledRight = (int32_t)(rightOut * 8388607.0f);

				// Ensure the result fits within 24 bits
				scaledRight &= 0x00FFFFFF;

			outBufPtr[sampleIndex] =  scaledLeft;
			outBufPtr[sampleIndex +1] = scaledRight;

		}

		dataReadyFlag = 0;

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_SPI3_Init();
  MX_SAI1_Init();
  MX_TIM6_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_StatusTypeDef halStatus;
  /*
   * Initialise codec
   */
  uint8_t codecInitStatus = CS4270_Init(&hi2c1, CODEC_NRST_GPIO_Port, CODEC_NRST_Pin);
  if ( codecInitStatus !=0 ) {Error_LED();};


  /* Initialise SAIs (to start clocks, so that codec PLL locks) */
  halStatus = HAL_SAI_Init(&hsai_BlockA1);
  if ( halStatus != HAL_OK ) {Error_LED(); }

  halStatus = HAL_SAI_Init(&hsai_BlockB1);
  if ( halStatus != HAL_OK ) {Error_LED(); }
  /*
   *  Start SAI DAC transmission
   */
  halStatus = HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t*)dacBuf, AUDIO_BUFFER_SIZE );
  if ( halStatus != HAL_OK ) {Error_LED(); }
  /*
   *  Start SAI ADC transmission
   */
  halStatus = HAL_SAI_Receive_DMA(&hsai_BlockB1, (uint8_t*)adcBuf, AUDIO_BUFFER_SIZE );
   if ( halStatus != HAL_OK ) {Error_LED(); }

   /*
    *  Start encoder counters
    */
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

   /*
    *  Start PWM timers and set PWM to 0 duty cycle
    */
//   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 4294967295);
//   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
//
//   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 65535);
//   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

   /*
    *  Start Internal ADC DMA streaming and set it to be continuos
    */

   HAL_ADC_Start_DMA(&hadc1, potValue, 9);

   /*
    *  Initilise the Effects
    */
   // DDFX_Delay_Init(DDFX_Delay *dly, float delayTime_ms, float mix, float feedback, float sampleRate_hz);
   DDFX_Delay_Init(&dly, 500.0f, 0.5f, 0.5f, SAMPLEING_FREQUENCY);

   // void DDFX_Tremolo_Init(DDFX_Tremolo *trem, float mix, float lfoFrequency_Hz, float sampleRate_Hz);
   DDFX_Tremolo_Init(&trem, 0.5f, 20.0f, SAMPLEING_FREQUENCY);

   // Noise gate Init
   DDFX_NoiseGate_Init(&ng, 0.02f, 100.0f, 10.0f, 10.0f, SAMPLEING_FREQUENCY);

   // Fuzz Init
   fuzz_init(&fuzz, 0.5f, 0.5f, 0.5f);

   // turn on Volume LED
   HAL_GPIO_WritePin(GPIOB, PLED_0_Pin, GPIO_PIN_SET);


   /*
    *  Start 50hz internal interupt
    */
   halStatus = HAL_TIM_Base_Start_IT(&htim6) ;
   if ( halStatus != HAL_OK ) {Error_LED(); }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(dataReadyFlag){

		  Process_HalfBuffer();

	  }else if(!controlSettingLocked) {

		  uint8_t oldPotValue[9];

		  for (uint8_t controlSettingNum = 0; controlSettingNum < CONTROL_SETTING_COUNT; controlSettingNum++){

			  if (oldPotValue[controlSettingNum]!= potValue[controlSettingNum]){

				  switch(controlSettingNum) {

				  	  // Volume
				  	  case 0:

				  		  volume = adcLUT[potValue[0]];

				  		  break;

					  // Delay: Gain of delay or fuzz
					  case 1:

						  if (encoder_count > 0){
							  dly.mix = adcLUT[potValue[1]];

						  } else{
							  fuzz.clip = adcLUT[potValue[1]];
						  }
//						  fuzz.clip = adcLUT[potValue[1]];
//
//						  dly.mix = adcLUT[potValue[1]];

						  break;

					  // Delay: Set how long Delay is
					  case 2:
						  if (encoder_count > 0){
							  DDFX_Delay_SetLength(&dly, 50.0f + 950.0f *adcLUT[potValue[2]], SAMPLEING_FREQUENCY);

						  } else{
							  fuzz.crunch = adcLUT[potValue[2]];
						  }
//						  fuzz.crunch = adcLUT[potValue[2]];
//						  DDFX_Delay_SetLength(&dly, 50.0f + 950.0f *adcLUT[potValue[2]], SAMPLEING_FREQUENCY);

						  break;

					  // Delay: Set feedback amount & set fuzz clip
					  case 3:

						  if (encoder_count > 0){
							  dly.feedback = adcLUT[potValue[3]];

						  } else{
							  fuzz.threshold = adcLUT[potValue[3]];
						  }
//						  fuzz.threshold = adcLUT[potValue[3]];
//						  dly.feedback = adcLUT[potValue[3]];

						  break;

					  // Tremolo: Set mix
					  case 6:

						  float mix = TREM_MIX_SCALE * adcLUT[potValue[6]];
						  DDFX_Tremolo_SetMix(&trem, mix);

						  break;

					  case 7:

						  float lfoFreq_Hz = TREM_LFO_FREQ_MIN + TREM_LFO_FREQ_SCALE * adcLUT[potValue[7]];
						  DDFX_Tremolo_SetLFO_Freq(&trem, lfoFreq_Hz);

						  break;


					  default :

						  break;

				  }

			  }
			  oldPotValue[controlSettingNum] = potValue[controlSettingNum];
		  }

		    rotEncoder = Read_Encoder(); // sets the rotEncoder value and a signed value to see if we have counted up or down

		    /*
		     * Read Foot Switches and turn on associated LED's
		     */
		    Foot_Sw_1 = HAL_GPIO_ReadPin(FT_1_GPIO_Port, FT_1_Pin);

		    if (Foot_Sw_1 == 1){

		    	HAL_GPIO_WritePin(GPIOB, PLED_1_Pin, GPIO_PIN_SET);
		    	HAL_GPIO_WritePin(GPIOA,PWLED_3_Pin|PWLED_2_Pin, GPIO_PIN_SET);

		    }else {

		    	HAL_GPIO_WritePin(GPIOB, PLED_1_Pin, GPIO_PIN_RESET);
		    	HAL_GPIO_WritePin(GPIOA,PWLED_3_Pin|PWLED_2_Pin, GPIO_PIN_RESET);

		    }

		    Foot_Sw_2 = HAL_GPIO_ReadPin(FT_2_GPIO_Port, FT_2_Pin);

		    if (Foot_Sw_2 == 1){

		    	HAL_GPIO_WritePin(GPIOC, PLED_7_Pin|PLED_6_Pin, GPIO_PIN_SET);

		    }else {

		    	HAL_GPIO_WritePin(GPIOC, PLED_7_Pin|PLED_6_Pin, GPIO_PIN_RESET);

		    }

		    if (encoder_count > 0){

		    	HAL_GPIO_WritePin(GPIOB, PLED_1_Pin, GPIO_PIN_SET);
		    	HAL_GPIO_WritePin(GPIOA,PWLED_3_Pin|PWLED_2_Pin, GPIO_PIN_SET);
		    	HAL_GPIO_WritePin(GPIOC, PLED_7_Pin|PLED_6_Pin, GPIO_PIN_RESET);



		    }


	  }


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 4;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_SPI3
                              |RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.PLL3.PLL3M = 4;
  PeriphClkInitStruct.PLL3.PLL3N = 32;
  PeriphClkInitStruct.PLL3.PLL3P = 17;
  PeriphClkInitStruct.PLL3.PLL3Q = 4;
  PeriphClkInitStruct.PLL3.PLL3R = 16;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSE;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL3;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL3;
  PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_PLL3;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 9;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_18;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x40000A0B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_44K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_24BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  hsai_BlockB1.Instance = SAI1_Block_B;
  hsai_BlockB1.Init.AudioMode = SAI_MODESLAVE_RX;
  hsai_BlockB1.Init.Synchro = SAI_SYNCHRONOUS;
  hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockB1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_24BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 99;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 47999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CODEC_NRST_Pin|PLED_7_Pin|PLED_6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PLED_5_Pin|PLED_4_Pin|PWLED_3_Pin|PWLED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PLED_0_Pin|PLED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_Pin|SPI3_DC_Pin|SPI3_RES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CODEC_NRST_Pin PLED_7_Pin PLED_6_Pin */
  GPIO_InitStruct.Pin = CODEC_NRST_Pin|PLED_7_Pin|PLED_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PLED_5_Pin PLED_4_Pin PWLED_3_Pin PWLED_2_Pin */
  GPIO_InitStruct.Pin = PLED_5_Pin|PLED_4_Pin|PWLED_3_Pin|PWLED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PLED_0_Pin PLED_1_Pin */
  GPIO_InitStruct.Pin = PLED_0_Pin|PLED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin SPI3_DC_Pin SPI3_RES_Pin */
  GPIO_InitStruct.Pin = LED_Pin|SPI3_DC_Pin|SPI3_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_BT_Pin */
  GPIO_InitStruct.Pin = EN_BT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EN_BT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FT_2_Pin */
  GPIO_InitStruct.Pin = FT_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(FT_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FT_1_Pin */
  GPIO_InitStruct.Pin = FT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(FT_1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){

	/*
	 * 50 times a second it jumps into this interupt to update values from user inputs
	 * including the 8 pots and the rotary encoder
	 */
	controlSettingLocked = 0;



}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
