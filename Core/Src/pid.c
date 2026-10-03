#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd, float outMax, float outMin)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->outMax = outMax;
    pid->outMin = outMin;

    /* 积分限幅: 积分项单独作用也不能超过输出量程,这就是抗积分饱和。
     * 不加这个的话,定位时误差长时间朝一个方向,积分越积越大,
     * 到了目标点也刹不住,来回荡(超调) */
    pid->integralMax = (pid->Ki > 0.0f) ? (pid->outMax / pid->Ki) : 0.0f;
    if (pid->integralMax < 0.0f) {
        pid->integralMax = -pid->integralMax;
    }

    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->out = 0.0f;
}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->out = 0.0f;
}

float PID_Update(PID_t *pid, float setpoint, float feedback, float dt)
{
    float error, derivative, out;

    if (dt <= 0.0f) {
        dt = 0.001f;    // 防除零
    }

    error = setpoint - feedback;

    /* 积分累积: 消除稳态误差,位置环就靠它把最后的定位误差吃干净 */
    pid->integral += error * dt;
    if (pid->integral >  pid->integralMax) pid->integral =  pid->integralMax;
    if (pid->integral < -pid->integralMax) pid->integral = -pid->integralMax;

    /* 微分: 误差变化率,加阻尼抑制超调 */
    derivative = (error - pid->prevError) / dt;
    pid->prevError = error;

    out = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    /* 输出限幅 */
    if (out > pid->outMax) out = pid->outMax;
    if (out < pid->outMin) out = pid->outMin;

    pid->out = out;
    return out;
}
