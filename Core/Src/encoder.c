#include "encoder.h"
#include "app_config.h"

static uint16_t s_lastCnt = 0;   // 上一次读到的计数器值
static int32_t  s_position = 0;  // 累积位置
static float    s_rpm = 0.0f;    // 滤波后的转速
static uint8_t  s_first = 1;

void Encoder_Init(void)
{
    /* 编码器模式的初始化CubeMX生成在tim.c里,这里只管启动 */
    HAL_TIM_Encoder_Start(ENCODER_TIM, TIM_CHANNEL_ALL);
    s_position = 0;
    s_lastCnt  = 0;
    s_rpm      = 0.0f;
    s_first    = 1;
}

void Encoder_ZeroPosition(void)
{
    /* 位置清零: 直接把当前点当原点 */
    s_position = 0;
    s_lastCnt  = (uint16_t)__HAL_TIM_GET_COUNTER(ENCODER_TIM);
}

/* 电机控制任务每5ms调一次,同时更新位置和转速 */
void Encoder_Update(int32_t dt_ms)
{
    uint16_t cnt = (uint16_t)__HAL_TIM_GET_COUNTER(ENCODER_TIM);

    if (s_first) {
        s_lastCnt = cnt;
        s_first = 0;
        return;
    }

    /* 位置累加: 差值按int16解释,这样16位计数器翻转(65535<->0)时
     * 也能算出正确增量。前提是5ms内增量不超过32768,实际远达不到 */
    int32_t delta = (int16_t)((uint16_t)(cnt - s_lastCnt));
    s_lastCnt = cnt;

    delta *= ENCODER_DIR;
    s_position += delta;

    /* M法测速: rpm = 周期内脉冲数 / 每转脉冲数 / 周期(分钟) */
    float rpmRaw = (float)delta * 60000.0f /
                   ((float)ENCODER_CNT_PER_REV * (float)dt_ms);

    /* 一阶低通,不然低速时M法量化误差大,转速跳得没法看 */
    s_rpm += SPEED_LPF_ALPHA * (rpmRaw - s_rpm);
}

int32_t Encoder_GetPosition(void)
{
    return s_position;
}

float Encoder_GetRpm(void)
{
    return s_rpm;
}
