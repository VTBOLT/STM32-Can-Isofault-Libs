#include "iso_fault_lib.h"

#define TIMCLOCK 170000000 // 170 MHz
#define PRESCALAR 680      // 100

/* Stupid global variables */
// Global variables to store measurement results
volatile float signal_frequency = 0.0f;
volatile float duty_cycle = 0.0f;
// Variables for input capture
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t IC_Val3 = 0;
uint32_t Difference = 0;
int Is_First_Captured = 0;

/* Measure Frequency */
float frequency = 0;
uint32_t usWidth = 0;

/**
 * @brief Timer callback function for input capture
 *
 * @param htim Pointer to timer handler
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
            if (Is_First_Captured == 0) // if the first rising edge is not captured
            {
                IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
                if (HAL_GPIO_ReadPin(IsoFaultBoard_Status_GPIO_Port, IsoFaultBoard_Status_Pin) != GPIO_PIN_SET) {
                    Is_First_Captured = 1;
                    return;
                }
                Is_First_Captured = 0; // set the first captured as true
            }

            else if (Is_First_Captured == 1) {                            // First falling edge
                IC_Val3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read second value
                Is_First_Captured = 2;
            }

            else if (Is_First_Captured == 3) {
                Is_First_Captured = 0; // set the first captured as true
            }

            else // If the first rising edge is captured, now we will capture the second edge
            {
                IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read second value

                if (IC_Val2 > IC_Val1) {
                    Difference = IC_Val2 - IC_Val1;
                }

                else if (IC_Val1 > IC_Val2) {
                    Difference = (0xffffffff - IC_Val1) + IC_Val2;
                }

                float refClock = TIMCLOCK / (PRESCALAR);
                float mFactor = 1000.0 / refClock;

                frequency = refClock / Difference;
                usWidth = (IC_Val2 - IC_Val3) * mFactor;
                signal_frequency = frequency;
                duty_cycle = (float)usWidth * frequency / 1000.0f;

                __HAL_TIM_SET_COUNTER(htim, 0); // reset the counter
                Is_First_Captured = 3;          // set it back to false
            }
        }
    }
}

/**
 * @brief Initialize the input capture for the isolation fault detection
 *
 * @param htim Pointer to timer handler
 */
void IsoFault_Init(TIM_HandleTypeDef *htim) {
    // Start input capture
    HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_1);
}

/**
 * @brief Get the signal measurements for the isolation fault detection
 *
 * @param freq Frequency of the signal
 * @param duty Duty cycle of the signal
 */
void get_signal_measurements(float *freq, float *duty) {
    *freq = signal_frequency;
    *duty = duty_cycle;
}

/**
 * @brief Check if and isolation fault is detected at all
 *
 * @return true if there is a fault, false otherwise
 */
bool IsoFault_IsFault() {
    return HAL_GPIO_ReadPin(IsoFaultBoard_IsFault_GPIO_Port, IsoFaultBoard_IsFault_Pin) != GPIO_PIN_SET;
}

/**
 * @brief Get the resistance of the current isolation fault
 *
 * @return Fault resistance in kOhms. Returns -1 if there is another
 * issue with the board
 */
float IsoFault_GetResistance_kOhms() {
    float frequency = 0.0;
    float dutyCycle = 0.0f;

    get_signal_measurements(&frequency, &dutyCycle);

    if (frequency < 9 || frequency > 11) {
        return -1;
    }

    // D = mR + b
    // R = (D - b) / m
    return ((duty_cycle * 100) - EQN_B) / EQN_M;
}
