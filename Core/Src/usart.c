/*
 * 串口部分: 中断逐字节收 -> 凑成一行丢进队列 -> 串口任务解析分发
 * 控制类指令(G/V/S/Z/TV/TP)经队列转发给电机控制任务,
 * 查询类(R/K)直接回复
 */
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ========== 中断接收 ========== */

static uint8_t s_rxByte;
static char    s_lineBuf[USART_LINE_LEN + 1];
static uint8_t s_lineLen = 0;

/* CubeMX生成的usart.c被这个文件覆盖了,所以串口引脚初始化挪到这里,
 * HAL_UART_Init内部会自动回调它: PA9=TX复用推挽, PA10=RX浮空输入 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (huart->Instance == USART1) {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* PA9 ------> USART1_TX */
        GPIO_InitStruct.Pin   = GPIO_PIN_9;
        GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* PA10 ------> USART1_RX */
        GPIO_InitStruct.Pin   = GPIO_PIN_10;
        GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

void USART_Start(void)
{
    s_lineLen = 0;
    /* 每次只接收1字节,收完再启动下一次 */
    HAL_UART_Receive_IT(USART_HANDLE, &s_rxByte, 1);
}

/* 接收完成回调(中断里跑): 这里调了xQueueSendFromISR,所以串口中断
 * 优先级必须是5(FreeRTOS的规矩,见FreeRTOSConfig.h的说明) */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart->Instance == USART_HANDLE->Instance) {
        if (s_rxByte == '\r' || s_rxByte == '\n') {
            /* 收到回车: 一行结束,丢给串口任务去解析 */
            if (s_lineLen > 0) {
                s_lineBuf[s_lineLen] = '\0';
                xQueueSendFromISR(g_lineQueue, s_lineBuf, &xHigherPriorityTaskWoken);
                s_lineLen = 0;
            }
        } else if (s_lineLen < USART_LINE_LEN) {
            s_lineBuf[s_lineLen++] = (char)s_rxByte;   // 超长直接丢,防溢出
        }
        /* 重新启动接收,不然中断链就断了 */
        HAL_UART_Receive_IT(USART_HANDLE, &s_rxByte, 1);
        /* 接收方(串口任务)优先级更高的话,立刻切一次任务 */
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* 串口出错(比如溢出ORE)后重启接收,防止接收链断掉 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART_HANDLE->Instance) {
        HAL_UART_Receive_IT(USART_HANDLE, &s_rxByte, 1);
    }
}

void USART_TransmitStr(const char *str)
{
    HAL_UART_Transmit(USART_HANDLE, (uint8_t *)str, (uint16_t)strlen(str), 100);
}

/* ========== 指令解析 ========== */

/* 手写的小数解析(保留2位小数)。Keil的微库不支持scanf %f,
 * 用了也白搭,干脆自己来 */
static uint8_t ParseFixed100(const char *s, float *out)
{
    int32_t sign = 1, intPart = 0, fracPart = 0, fracDigits = 0;

    if (*s == '-')      { sign = -1; s++; }
    else if (*s == '+') { s++; }
    if (*s < '0' || *s > '9') {
        return 0;   /* 非法输入 */
    }
    while (*s >= '0' && *s <= '9') {
        intPart = intPart * 10 + (*s - '0');
        s++;
    }
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9' && fracDigits < 2) {
            fracPart = fracPart * 10 + (*s - '0');
            fracDigits++;
            s++;
        }
    }
    if (fracDigits == 1) {
        fracPart *= 10;
    }
    *out = (float)sign * ((float)intPart + (float)fracPart / 100.0f);
    return 1;
}

/* 查询当前工况: 状态是共享数据,整体拷贝时要关中断,
 * 不然读到一半电机任务改了数据,前后就对不上了 */
static void CmdReplyStatus(void)
{
    SysState_t st;
    char buf[64];

    taskENTER_CRITICAL();
    st = g_sysState;
    taskEXIT_CRITICAL();

    const char *modeStr = (st.mode == MODE_POSITION) ? "POSITION" :
                          (st.mode == MODE_SPEED)    ? "SPEED"    : "STOP";
    snprintf(buf, sizeof(buf), "POS:%d SPD:%d RPM MODE:%s TGT:%d\r\n",
             (int)st.position, st.rpm, modeStr, (int)st.target);
    USART_TransmitStr(buf);
}

/* 查各任务还剩多少栈(字),排查栈溢出很好用 */
static void CmdReplyStack(void)
{
    char buf[96];

    snprintf(buf, sizeof(buf),
             "STACK WaterMark(words) Motor:%u Usart:%u Oled:%u\r\n",
             (unsigned)uxTaskGetStackHighWaterMark(g_motorTask),
             (unsigned)uxTaskGetStackHighWaterMark(g_usartTask),
             (unsigned)uxTaskGetStackHighWaterMark(g_oledTask));
    USART_TransmitStr(buf);
}

static void CmdReplyHelp(void)
{
    /* 字符串用英文——ARMCC在中文Windows把UTF-8按GBK读,中文会乱码
     * (编译报#870那个警告),中文注释没事,只有字符串不行 */
    USART_TransmitStr(
        "===== Motor Servo Commands =====\r\n"
        "G <cnt>     position mode, target = encoder counts\r\n"
        "V <rpm>     speed mode, target rpm\r\n"
        "S           stop (brake)\r\n"
        "Z           zero current position\r\n"
        "R           query running status\r\n"
        "TV kp ki kd tune speed-loop PID online\r\n"
        "TP kp ki kd tune position-loop PID online\r\n"
        "K           query task stack watermark\r\n"
        "H           help\r\n");
}

/* 解析一条指令。控制类转发给电机控制任务(队列天然串行化,
 * 所有控制参数只会在电机任务里改,不用加锁);
 * 查询类直接回复 */
static void ParseAndDispatch(char *line)
{
    MotorCmd_t cmd;
    char *op = strtok(line, " \t");

    if (op == NULL) {
        return;
    }

    if (strcmp(op, "G") == 0) {
        char *arg = strtok(NULL, " \t");
        if (arg != NULL) {
            cmd.type  = CMD_SET_POS;
            cmd.value = atoi(arg);
            xQueueSend(g_cmdQueue, &cmd, 10);
            USART_TransmitStr("OK: POSITION MODE\r\n");
        } else {
            USART_TransmitStr("ERR: usage G <cnt>\r\n");
        }
    } else if (strcmp(op, "V") == 0) {
        char *arg = strtok(NULL, " \t");
        if (arg != NULL) {
            cmd.type  = CMD_SET_SPEED;
            cmd.value = atoi(arg);
            xQueueSend(g_cmdQueue, &cmd, 10);
            USART_TransmitStr("OK: SPEED MODE\r\n");
        } else {
            USART_TransmitStr("ERR: usage V <rpm>\r\n");
        }
    } else if (strcmp(op, "S") == 0) {
        cmd.type = CMD_STOP;
        xQueueSend(g_cmdQueue, &cmd, 10);
        USART_TransmitStr("OK: STOP\r\n");
    } else if (strcmp(op, "Z") == 0) {
        cmd.type = CMD_ZERO_POS;
        xQueueSend(g_cmdQueue, &cmd, 10);
        USART_TransmitStr("OK: position zeroed\r\n");
    } else if (strcmp(op, "R") == 0) {
        CmdReplyStatus();
    } else if (strcmp(op, "K") == 0) {
        CmdReplyStack();
    } else if (strcmp(op, "H") == 0) {
        CmdReplyHelp();
    } else if (strcmp(op, "TV") == 0) {
        float kp, ki, kd;
        char *a1 = strtok(NULL, " \t");
        char *a2 = strtok(NULL, " \t");
        char *a3 = strtok(NULL, " \t");
        if (a1 && a2 && a3 &&
            ParseFixed100(a1, &kp) && ParseFixed100(a2, &ki) && ParseFixed100(a3, &kd)) {
            cmd.type = CMD_TUNE_SPEED_PID;
            cmd.p1 = kp; cmd.p2 = ki; cmd.p3 = kd;
            xQueueSend(g_cmdQueue, &cmd, 10);
            USART_TransmitStr("OK: speed PID updated\r\n");
        } else {
            USART_TransmitStr("ERR: usage TV <kp> <ki> <kd>\r\n");
        }
    } else if (strcmp(op, "TP") == 0) {
        float kp, ki, kd;
        char *a1 = strtok(NULL, " \t");
        char *a2 = strtok(NULL, " \t");
        char *a3 = strtok(NULL, " \t");
        if (a1 && a2 && a3 &&
            ParseFixed100(a1, &kp) && ParseFixed100(a2, &ki) && ParseFixed100(a3, &kd)) {
            cmd.type = CMD_TUNE_POS_PID;
            cmd.p1 = kp; cmd.p2 = ki; cmd.p3 = kd;
            xQueueSend(g_cmdQueue, &cmd, 10);
            USART_TransmitStr("OK: position PID updated\r\n");
        } else {
            USART_TransmitStr("ERR: usage TP <kp> <ki> <kd>\r\n");
        }
    } else {
        USART_TransmitStr("ERR: unknown command, H for help\r\n");
    }
}

/* 串口任务: 阻塞等队列里的一行指令,来了就解析 */
void UsartCmdTask(void *arg)
{
    (void)arg;
    char line[USART_LINE_LEN + 1];

    for (;;) {
        if (xQueueReceive(g_lineQueue, line, portMAX_DELAY) == pdPASS) {
            ParseAndDispatch(line);
        }
    }
}
