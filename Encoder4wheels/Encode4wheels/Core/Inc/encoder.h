/*
 * encoder.h
 *
 *  Created on: Sep 9, 2024
 *      Author: Tung Tze Yang
 *
 *  Written for MMU Cybertron
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

typedef struct
{
	volatile int32_t counter;
	int RPM;
}encoder;

encoder encoder1, encoder2, encoder3, encoder4;
// based on the datasheet the counter will be increase steps by 4 for each pulse received

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {
        // Handle encoder 1 (connected to TIM2)
        encoder1.counter = (int16_t) __HAL_TIM_GET_COUNTER(htim);
        //encoder1.counter = TIM1 -> CNT;
    }
     if (htim->Instance == TIM2) {
        // Handle encoder 2 (connected to TIM3)
        encoder2.counter = (int16_t) __HAL_TIM_GET_COUNTER(htim);
    }
     if (htim->Instance == TIM3) {
		// Handle encoder 3 (connected to TIM3)
		encoder3.counter = (int16_t) __HAL_TIM_GET_COUNTER(htim);

	}
     if (htim->Instance == TIM4) {
		// Handle encoder 4 (connected to TIM3)
		encoder4.counter = (int16_t) __HAL_TIM_GET_COUNTER(htim);
	}
    else{
    	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
    }
    // can continue with another encoder, and so on....
}


#endif /* INC_ENCODER_H_ */
