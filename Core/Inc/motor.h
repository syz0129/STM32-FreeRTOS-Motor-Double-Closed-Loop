#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

/* 电机驱动引脚,按实际接线改这里 */
#define MOTOR_PWM_TIM           (&htim1)        // PWM用TIM1_CH1,CubeMX里已配好
#define MOTOR_PWM_CHANNEL       TIM_CHANNEL_1

/* 方向脚接驱动模块的IN1/IN2 */
#define MOTOR_IN1_GPIO_Port     GPIOB
#define MOTOR_IN1_Pin           GPIO_PIN_12
#define MOTOR_IN2_GPIO_Port     GPIOB
#define MOTOR_IN2_Pin           GPIO_PIN_13

/* TB6612的STBY脚: 模块上已直接拉高的话置0不用管 */
#define MOTOR_USE_STBY          (0)
#define MOTOR_STBY_GPIO_Port    GPIOB
#define MOTOR_STBY_Pin          GPIO_PIN_14

/* 占空比最多给90%,留点余量 */
#define MOTOR_MAX_DUTY          (900)

extern TIM_HandleTypeDef htim1;

void Motor_Init(void);
void Motor_SetDuty(int32_t duty);   // -1000~1000,正=正转,负=反转,0=刹车
void Motor_Stop(void);

#endif /* __MOTOR_H */
