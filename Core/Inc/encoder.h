#ifndef __ENCODER_H
#define __ENCODER_H

/* defines */
#define X_ENCODER_TIM_HANDLE htim1
#define Y_ENCODER_TIM_HANDLE htim3
#define Z_ENCODER_TIM_HANDLE htim4

/* external variables */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* function prototypes */
void encoderTask(void *pvParameters);
void encoderInterruptHandler(TIM_HandleTypeDef *htim);
uint32_t encoderGetCounter();
void encoderResetCounter(TIM_HandleTypeDef *htim);
int32_t encoderReadDegree();
float encoderReadRPM();

#endif // __ENCODER_H