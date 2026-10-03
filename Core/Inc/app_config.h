#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

/* 全局配置都放这里,改硬件/调参数只动这个文件 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* ---- 任务 ---- */
/* 电机控制任务优先级最高(6),保证5ms控制周期不被显示和串口任务打断;
 * USART和OLED任务优先级低一点,一个收指令一个刷屏 */
#define MOTOR_TASK_PRIORITY     6
#define USART_TASK_PRIORITY     4
#define OLED_TASK_PRIORITY      3

/* 任务栈(单位:字,1字=4字节),256字=1KB。
 * 之前USART任务给128字时栈溢出过(库函数吃栈),干脆统一给256,
 * 串口发K指令能看各任务还剩多少栈 */
#define MOTOR_TASK_STACK_SIZE   256
#define USART_TASK_STACK_SIZE   256
#define OLED_TASK_STACK_SIZE    256

/* 队列深度 */
#define CMD_QUEUE_LEN           8    // 控制指令队列
#define LINE_QUEUE_LEN          6    // 串口指令行队列
#define STATUS_QUEUE_LEN        8    // 工况数据队列

/* ---- 控制参数 ---- */
#define MOTOR_CTRL_PERIOD_MS    5         // 控制周期5ms(200Hz)
#define MOTOR_CTRL_PERIOD_S     (MOTOR_CTRL_PERIOD_MS / 1000.0f)
#define POS_LOOP_DIV            2         // 位置环每2个周期跑1次(100Hz)
#define SPEED_LPF_ALPHA         0.3f      // 测速低通滤波系数,越小越平滑
#define MOTOR_SPEED_MAX_RPM     300.0f    // 位置环输出的转速上限(rpm)

/* ---- 共享类型 ---- */
/* 串口解析出来的控制指令,经队列发给电机控制任务 */
typedef enum {
    CMD_SET_POS = 0,    // G: 位置模式定位
    CMD_SET_SPEED,      // V: 速度模式调速
    CMD_STOP,           // S: 停止
    CMD_TUNE_SPEED_PID, // TV: 在线整定速度环PID
    CMD_TUNE_POS_PID,   // TP: 在线整定位置环PID
    CMD_ZERO_POS,       // Z: 当前位置清零
} CmdType_t;

typedef struct {
    CmdType_t type;
    int32_t   value;       // 位置目标(编码器计数)或转速目标(rpm)
    float     p1, p2, p3;  // TV/TP整定指令带的kp/ki/kd
} MotorCmd_t;

typedef enum {
    MODE_STOP = 0,
    MODE_SPEED,
    MODE_POSITION,
} MotorMode_t;

/* 系统状态快照: 电机任务写,串口任务读,跨任务访问要进临界区 */
typedef struct {
    int32_t     position;
    int16_t     rpm;
    MotorMode_t mode;
    int32_t     target;
} SysState_t;

/* ---- 全局对象 ---- */
extern QueueHandle_t g_cmdQueue;      // 串口任务   -> 电机控制任务
extern QueueHandle_t g_lineQueue;     // 串口中断   -> 串口任务
extern QueueHandle_t g_statusQueue;   // 电机控制任务 -> OLED任务
extern TaskHandle_t g_motorTask, g_usartTask, g_oledTask;
extern volatile SysState_t g_sysState;

#endif /* __APP_CONFIG_H */
