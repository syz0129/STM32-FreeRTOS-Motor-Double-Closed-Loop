/*
 * FreeRTOS V10.x配置,在CubeMX生成的模板上改了几处:
 *  - 优先级上限8,电机任务用6(最高)
 *  - 堆给到10K,C8T6总共20K RAM,还要给外设留空间
 *  - 开了栈溢出检测(方法2),配合main.c里的钩子函数
 *  - MAX_SYSCALL_INTERRUPT_PRIORITY配5,串口中断里调了队列发送,优先级必须>=5
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>

extern uint32_t SystemCoreClock;

/* 基础调度配置 */
#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_TICKLESS_IDLE                 0
#define configCPU_CLOCK_HZ                      ( SystemCoreClock )        /* 72MHz */
#define configTICK_RATE_HZ                      ( (TickType_t)1000 )       /* 1kHz节拍 */
#define configMAX_PRIORITIES                    ( 8 )
#define configMINIMAL_STACK_SIZE                ( (unsigned short)128 )
#define configMAX_TASK_NAME_LEN                 ( 16 )
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1

/* 队列/互斥/信号量/任务通知 */
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           0
#define configQUEUE_REGISTRY_SIZE               8
#define configUSE_QUEUE_SETS                    0
#define configUSE_TASK_NOTIFICATIONS            1

/* 内存管理 */
#define configSUPPORT_STATIC_ALLOCATION         1
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configTOTAL_HEAP_SIZE                   ( (size_t)10240 )
#define configAPPLICATION_ALLOCATED_HEAP        0
#define configUSE_MALLOC_FAILED_HOOK            1

/* 栈溢出检测 */
#define configCHECK_FOR_STACK_OVERFLOW          2

/* 调试/统计 */
#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_TRACE_FACILITY                1
#define configUSE_STATS_FORMATTING_FUNCTIONS    0

/* 协程(没用上) */
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         2

/* 软件定时器(本工程没用软件定时器,但内核会建个定时器服务任务) */
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               4
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            128
#define configTIMER_SERVICE_TASK_NAME           "Tmr Svc"

/* F1中断优先级0~15,数值越小优先级越高。凡是中断里调FreeRTOS API的
 * (本工程只有串口接收中断用了xQueueSendFromISR),其中断优先级数值
 * 必须>=5,不然会打断内核临界区,系统跑飞。串口中断优先级在
 * main.c的MX_USART1_UART_Init里配的是5,满足 */
#ifdef __NVIC_PRIO_BITS
    #define configPRIO_BITS       __NVIC_PRIO_BITS
#else
    #define configPRIO_BITS       4
#endif

/* 可以被"设置优先级"函数使用的最低中断优先级 */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY          15
/* 可以在中断中使用FreeRTOS中断安全API的最高优先级(数值) */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY     5
/* 内核移植层自身使用的中断优先级 */
#define configKERNEL_INTERRUPT_PRIORITY      ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* 断言: 参数非法时死循环,方便调试器定位 */
#define configASSERT( x ) if ((x) == 0) { taskDISABLE_INTERRUPTS(); for( ;; ); }

/* 可选钩子 */
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configUSE_DAEMON_TASK_STARTUP_HOOK       0
#define configUSE_POSIX_ERRNO                    0
#define configUSE_NEWLIB_REENTRANT               0

/* 按需开启的API,减小内核体积 */
#define INCLUDE_vTaskPrioritySet                 1
#define INCLUDE_uxTaskPriorityGet                1
#define INCLUDE_vTaskDelete                      1
#define INCLUDE_vTaskSuspend                     1
#define INCLUDE_vTaskDelayUntil                  1
#define INCLUDE_vTaskDelay                       1
#define INCLUDE_xTaskGetSchedulerState           1
#define INCLUDE_xTaskGetCurrentTaskHandle        1
#define INCLUDE_uxTaskGetStackHighWaterMark      1
#define INCLUDE_xTaskGetIdleTaskHandle           0
#define INCLUDE_eTaskGetState                    0
#define INCLUDE_xTimerPendFunctionCall           0
#define INCLUDE_xQueueGetMutexHolder             0
#define INCLUDE_pcTaskGetTaskName                0

#endif /* FREERTOS_CONFIG_H */
