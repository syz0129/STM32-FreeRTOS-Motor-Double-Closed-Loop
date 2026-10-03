#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

/* 编码器配置,换编码器/换电机改这里 */
#define ENCODER_TIM             (&htim2)    // TIM2编码器模式,CubeMX里配的TI1+TI2
#define ENCODER_PPR             (13)        // 编码器线数: 每转13个脉冲
#define ENCODER_MULTIPLE        (4)         // 4倍频
#define ENCODER_CNT_PER_REV     (ENCODER_PPR * ENCODER_MULTIPLE)  // 电机每转的计数
#define ENCODER_DIR             (+1)        // 方向接反了改-1

extern TIM_HandleTypeDef htim2;

void    Encoder_Init(void);
void    Encoder_Update(int32_t dt_ms);      // 电机控制任务每个周期调一次
int32_t Encoder_GetPosition(void);
float   Encoder_GetRpm(void);
void    Encoder_ZeroPosition(void);

#endif /* __ENCODER_H */
