/*
 * 主文件: 系统初始化 + 3个FreeRTOS任务
 *
 * 任务结构(括号里是优先级,数字越大越高):
 *   MotorCtrl(6)  最高,5ms周期,位置-速度双闭环PID都在这算
 *   UsartCmd (4)  收串口指令,解析后经队列发给MotorCtrl
 *   OledDisp (3)  等队列里的工况数据显示
 * 队列:  lineQueue(串口中断->串口任务) cmdQueue(串口任务->电机任务)
 *        statusQueue(电机任务->OLED任务)
 *
 * 说明: 这个main.c替换了CubeMX生成的main.c。新版CubeMX把外设初始化
 * 生成在tim.c/gpio.c/i2c.c/usart.c这些独立文件里,main.c里直接调
 * MX_TIM1_Init之类的函数就行(声明在tim.h/gpio.h/i2c.h)。
 * CubeMX的usart.c被本工程的usart.c覆盖了,所以串口初始化挪到本文件。
 * 重新生成代码会覆盖main.c和usart.c,生成后要重新拷回来。
 */
#include "main.h"
#include "tim.h"        /* htim1/htim2和MX_TIM1_Init/MX_TIM2_Init的声明 */
#include "gpio.h"       /* MX_GPIO_Init声明 */
#include "i2c.h"        /* hi2c1和MX_I2C1_Init声明 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "app_config.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "oled.h"
#include "usart.h"

/* ========== 硬件配置 ========== */
#define LED_GPIO_Port     GPIOC      // 板载LED,运行指示灯
#define LED_Pin           GPIO_PIN_13

/* 默认PID参数,实际要按电机调,也可以串口TV/TP在线改 */
#define PID_SPEED_DEF_KP    5.0f
#define PID_SPEED_DEF_KI    15.0f
#define PID_SPEED_DEF_KD    0.05f
#define PID_POS_DEF_KP      1.8f
#define PID_POS_DEF_KI      0.5f
#define PID_POS_DEF_KD      0.1f

/* ========== 全局变量 ========== */
/* 三条队列,任务间的数据交互全走消息队列 */
QueueHandle_t g_cmdQueue;      // 串口任务 -> 电机控制任务
QueueHandle_t g_lineQueue;     // 串口中断 -> 串口任务
QueueHandle_t g_statusQueue;   // 电机控制任务 -> OLED任务
TaskHandle_t  g_motorTask, g_usartTask, g_oledTask;

/* 系统状态快照: 电机任务写,串口任务读,跨任务访问要进临界区 */
volatile SysState_t g_sysState;

/* 两个环的PID实例,只有电机控制任务碰它们,不用加锁 */
static PID_t s_pidSpeed;       // 内环: 速度环,输出PWM占空比
static PID_t s_pidPos;         // 外环: 位置环,输出期望转速

/* htim1/htim2定义在tim.c里,hi2c1在i2c.c里,htim3在时基文件里。
 * huart1放这里: CubeMX的usart.c被本工程usart.c覆盖了 */
UART_HandleTypeDef huart1;

/* ========== 初始化函数 ========== */
/* 外设初始化直接调CubeMX生成的函数:
 * MX_GPIO_Init(gpio.c) MX_TIM1_Init/MX_TIM2_Init(tim.c) MX_I2C1_Init(i2c.c)。
 * 引脚和外设配得不一样就重新生成代码,本文件不用改 */

/* 时钟: HSE 8MHz x9 = 72MHz。板子晶振不是8M的话,用CubeMX生成的
 * SystemClock_Config替换这个函数 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* 开HSE,PLL 9倍频 */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /* 72MHz主频,APB1二分频(上面的定时器倍频后还是72MHz) */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/* 串口初始化本来CubeMX生成在usart.c,那文件被覆盖了,挪到这里。
 * 优先级配5: 接收回调里调了xQueueSendFromISR,优先级数值必须>=5
 * (见FreeRTOSConfig.h)。引脚PA9/PA10在usart.c的HAL_UART_MspInit里配 */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* TIM3时基中断回调。CubeMX把时基初始化生成在stm32f1xx_hal_timebase_tim.c
 * (HAL_InitTick),但那个文件里没有这个回调,得自己写,不然HAL_GetTick
 * 不走,HAL_Delay和I2C超时全废。注意: 这个函数不能加__weak,
 * 加了会和HAL库自带的弱定义重复,链接直接报错 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        HAL_IncTick();
    }
}

/* ========== 电机控制任务(最高优先级) ========== */

/* 当前模式与目标值,只有电机控制任务访问 */
static MotorMode_t s_mode = MODE_STOP;       // 上电默认停,防误动作
static int32_t  s_posTarget = 0;             // 位置模式目标(编码器计数)
static float    s_speedTarget = 0.0f;        // 速度模式目标(或位置环输出)
static uint32_t s_tick = 0;                  // 控制周期计数

/* 切模式先清PID,不然积分残留,切过去那一下电机会抖 */
static void Control_SetMode(MotorMode_t mode)
{
    if (s_mode == mode) {
        return;
    }
    s_mode = mode;
    PID_Reset(&s_pidSpeed);
    PID_Reset(&s_pidPos);
}

/* 消费指令队列(非阻塞): 指令都在队列里,只有电机任务改控制参数,
 * 队列天然串行化,这里不用加锁 */
static void Control_HandleCommands(void)
{
    MotorCmd_t cmd;

    while (xQueueReceive(g_cmdQueue, &cmd, 0) == pdPASS) {
        switch (cmd.type) {
        case CMD_SET_POS:
            s_posTarget = cmd.value;
            Control_SetMode(MODE_POSITION);
            break;
        case CMD_SET_SPEED:
            s_speedTarget = (float)cmd.value;
            Control_SetMode(MODE_SPEED);
            break;
        case CMD_STOP:
            Control_SetMode(MODE_STOP);
            break;
        case CMD_ZERO_POS:
            Encoder_ZeroPosition();          // 当前位置当原点
            s_posTarget = 0;
            break;
        case CMD_TUNE_SPEED_PID:
            PID_Init(&s_pidSpeed, cmd.p1, cmd.p2, cmd.p3,
                     MOTOR_MAX_DUTY, -MOTOR_MAX_DUTY);
            break;
        case CMD_TUNE_POS_PID:
            PID_Init(&s_pidPos, cmd.p1, cmd.p2, cmd.p3,
                     MOTOR_SPEED_MAX_RPM, -MOTOR_SPEED_MAX_RPM);
            break;
        default:
            break;
        }
    }
}

/* 电机控制任务: 优先级最高(6),5ms一个周期,用vTaskDelayUntil保证
 * 周期不被显示/串口任务影响。
 * 级联双闭环:
 *   外环 位置环(100Hz,每2个周期跑1次): 位置误差 -> PID -> 期望转速(限幅)
 *   内环 速度环(200Hz): 转速误差 -> PID -> PWM占空比 -> 电机 */
void MotorCtrlTask(void *arg)
{
    (void)arg;
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(MOTOR_CTRL_PERIOD_MS);

    for (;;) {
        /* 精确周期: 以上次唤醒时刻为基准延时,周期不漂移 */
        vTaskDelayUntil(&lastWake, period);

        /* 1.先处理积压的指令 */
        Control_HandleCommands();

        /* 2.读编码器: 更新位置和滤波转速 */
        Encoder_Update(MOTOR_CTRL_PERIOD_MS);
        int32_t pos = Encoder_GetPosition();
        float rpm = Encoder_GetRpm();

        /* 3.双闭环PID */
        float duty = 0.0f;
        switch (s_mode) {
        case MODE_SPEED:
            /* 速度模式只跑内环 */
            duty = PID_Update(&s_pidSpeed, s_speedTarget, rpm, MOTOR_CTRL_PERIOD_S);
            break;

        case MODE_POSITION:
            /* 位置模式: 外环位置PID输出期望转速(限幅后给内环) */
            if ((s_tick % POS_LOOP_DIV) == 0) {
                float spdRef = PID_Update(&s_pidPos, (float)s_posTarget, (float)pos,
                                          MOTOR_CTRL_PERIOD_S * POS_LOOP_DIV);
                if (spdRef >  MOTOR_SPEED_MAX_RPM) spdRef =  MOTOR_SPEED_MAX_RPM;
                if (spdRef < -MOTOR_SPEED_MAX_RPM) spdRef = -MOTOR_SPEED_MAX_RPM;
                s_speedTarget = spdRef;
            }
            /* 内环: 速度PID输出占空比 */
            duty = PID_Update(&s_pidSpeed, s_speedTarget, rpm, MOTOR_CTRL_PERIOD_S);
            break;

        case MODE_STOP:
        default:
            /* 停止: 清积分,输出0(刹车) */
            PID_Reset(&s_pidSpeed);
            PID_Reset(&s_pidPos);
            duty = 0.0f;
            break;
        }
        Motor_SetDuty((int32_t)duty);

        /* 4.更新状态快照。这里是共享数据,串口任务要读,
         * 写入时关一下中断,防止读一半被抢占(临界区保护) */
        taskENTER_CRITICAL();
        g_sysState.position = pos;
        g_sysState.rpm      = (int16_t)rpm;
        g_sysState.mode     = s_mode;
        g_sysState.target   = (s_mode == MODE_POSITION) ? s_posTarget : (int32_t)s_speedTarget;
        taskEXIT_CRITICAL();

        /* 5.每10个周期(50ms)给OLED任务推一次工况 */
        s_tick++;
        if ((s_tick % 10) == 0) {
            SysState_t st;
            st.position = pos;
            st.rpm      = (int16_t)rpm;
            st.mode     = s_mode;
            st.target   = (s_mode == MODE_POSITION) ? s_posTarget : (int32_t)s_speedTarget;
            xQueueSend(g_statusQueue, &st, 0);
        }

        /* 6.运行灯: 每秒翻一次 */
        if ((s_tick % 200) == 0) {
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        }
    }
}

/* ========== 主函数 ========== */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    /* 外设初始化(函数在CubeMX生成的tim.c/gpio.c/i2c.c里) */
    MX_GPIO_Init();
    MX_TIM1_Init();             // PWM
    MX_TIM2_Init();             // 编码器正交解码
    MX_I2C1_Init();             // OLED
    MX_USART1_UART_Init();      // 串口(本文件内实现)

    /* 模块驱动初始化 */
    Motor_Init();               // 启动PWM,电机初始刹车
    Encoder_Init();             // 启动正交解码
    OLED_Init();
    USART_Start();              // 启动串口中断接收

    /* 开机画面 */
    OLED_ShowString(0, 0, "STM32 MOTOR SERVO");
    OLED_ShowString(0, 2, "FREERTOS PID CTRL");
    OLED_ShowString(0, 4, "UART 115200 8N1");

    /* PID初始化: 位置环输出限幅=最大转速,速度环输出限幅=最大占空比 */
    PID_Init(&s_pidSpeed, PID_SPEED_DEF_KP, PID_SPEED_DEF_KI, PID_SPEED_DEF_KD,
             MOTOR_MAX_DUTY, -MOTOR_MAX_DUTY);
    PID_Init(&s_pidPos, PID_POS_DEF_KP, PID_POS_DEF_KI, PID_POS_DEF_KD,
             MOTOR_SPEED_MAX_RPM, -MOTOR_SPEED_MAX_RPM);

    /* 建队列 */
    g_cmdQueue    = xQueueCreate(CMD_QUEUE_LEN,    sizeof(MotorCmd_t));
    g_lineQueue   = xQueueCreate(LINE_QUEUE_LEN,   USART_LINE_LEN + 1);
    g_statusQueue = xQueueCreate(STATUS_QUEUE_LEN, sizeof(SysState_t));
    if (g_cmdQueue == NULL || g_lineQueue == NULL || g_statusQueue == NULL) {
        /* 建队列失败一般是堆不够,查configTOTAL_HEAP_SIZE */
        Error_Handler();
    }

    /* 建任务 */
    xTaskCreate(MotorCtrlTask, "MotorCtrl", MOTOR_TASK_STACK_SIZE, NULL,
                MOTOR_TASK_PRIORITY, &g_motorTask);
    xTaskCreate(UsartCmdTask, "UsartCmd", USART_TASK_STACK_SIZE, NULL,
                USART_TASK_PRIORITY, &g_usartTask);
    xTaskCreate(OledDispTask, "OledDisp", OLED_TASK_STACK_SIZE, NULL,
                OLED_TASK_PRIORITY, &g_oledTask);

    /* 启动调度器,之后不会返回 */
    vTaskStartScheduler();

    /* 调度器启动失败才会走到这 */
    Error_Handler();
    while (1) {
    }
}

/* ========== 错误处理与FreeRTOS钩子 ========== */

/* 栈溢出钩子: configCHECK_FOR_STACK_OVERFLOW=2时内核检测到溢出会调这,
 * 调试时在这打断点,看pcTaskName就知道是哪个任务,调大它的栈 */
__weak void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}

/* 堆申请失败钩子: 一般是configTOTAL_HEAP_SIZE给小了 */
__weak void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}

/* 定时器任务的内存回调: 开了静态分配+软件定时器后,内核建定时器任务
 * 会找这个函数。CubeMX的freertos.c只给了空闲任务的,没给定时器的,
 * 这里补上(弱定义,哪天CubeMX自己生成了就自动用它的) */
__weak void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                           StackType_t **ppxTimerTaskStackBuffer,
                                           uint32_t *pulTimerTaskStackSize)
{
    static StaticTask_t xTimerTaskTCBBuffer;
    static StackType_t xTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

    *ppxTimerTaskTCBBuffer   = &xTimerTaskTCBBuffer;
    *ppxTimerTaskStackBuffer = &xTimerTaskStack[0];
    *pulTimerTaskStackSize   = configTIMER_TASK_STACK_DEPTH;
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}
