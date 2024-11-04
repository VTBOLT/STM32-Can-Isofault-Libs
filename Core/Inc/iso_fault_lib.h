#pragma once

/* Includes */
#include "main.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_nucleo.h"
#include <stdbool.h>
#include <stdlib.h>

/* Constants */
#define EQN_B 94.964f // y-intercept in eq to find duty cycle from resistance
#define EQN_M -0.072f // slope in eq to find duty cycle from resistance

/* Function declarations */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void IsoFault_Init(TIM_HandleTypeDef *htim);
void get_signal_measurements(float *freq, float *duty);
bool IsoFault_IsFault();
float IsoFault_GetResistance_kOhms();