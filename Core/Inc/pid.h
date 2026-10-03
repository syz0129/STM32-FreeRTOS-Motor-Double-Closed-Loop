#ifndef __PID_H
#define __PID_H

/* 位置式PID,位置环和速度环共用这一套 */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;     // 积分累积
    float prevError;    // 上次误差,算微分用
    float outMax;       // 输出限幅
    float outMin;
    float integralMax;  // 积分限幅,防饱和
    float out;          // 本次输出
} PID_t;

void  PID_Init(PID_t *pid, float kp, float ki, float kd, float outMax, float outMin);
/* 位置式: out = Kp*err + Ki*∫err dt + Kd*derr/dt,输出带限幅 */
float PID_Update(PID_t *pid, float setpoint, float feedback, float dt);
/* 清积分/微分状态,切模式或停车时调,不然电机容易抖一下 */
void  PID_Reset(PID_t *pid);

#endif /* __PID_H */
