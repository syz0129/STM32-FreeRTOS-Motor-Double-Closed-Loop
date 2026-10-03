#include "motor.h"

void Motor_Init(void)
{
    /* TIM1的PWM初始化CubeMX生成在tim.c里了,这里只管启动输出 */
    HAL_TIM_PWM_Start(MOTOR_PWM_TIM, MOTOR_PWM_CHANNEL);

#if (MOTOR_USE_STBY == 1)
    /* 用TB6612且STBY没短接高电平的话,这里把PB14拉起来 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = MOTOR_STBY_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOTOR_STBY_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_SET);
#endif

    /* 上电先刹车,别一开机电机就转起来 */
    Motor_Stop();
}

void Motor_SetDuty(int32_t duty)
{
    uint32_t arr, cmp;

    /* 限幅 */
    if (duty >  MOTOR_MAX_DUTY) duty =  MOTOR_MAX_DUTY;
    if (duty < -MOTOR_MAX_DUTY) duty = -MOTOR_MAX_DUTY;

    if (duty == 0) {
        Motor_Stop();   // 0就是刹车
        return;
    }

    /* 按符号切方向: IN1高IN2低正转,反过来反转 */
    if (duty > 0) {
        HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_SET);
        duty = -duty;
    }

    /* 占空比是千分之,换算成比较值: duty/1000*ARR */
    arr = __HAL_TIM_GET_AUTORELOAD(MOTOR_PWM_TIM);
    cmp = (uint32_t)duty * arr / 1000UL;
    __HAL_TIM_SET_COMPARE(MOTOR_PWM_TIM, MOTOR_PWM_CHANNEL, cmp);
}

void Motor_Stop(void)
{
    /* IN1/IN2同时拉低=刹车(TB6612/L298N都是这个逻辑) */
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(MOTOR_PWM_TIM, MOTOR_PWM_CHANNEL, 0);
}
