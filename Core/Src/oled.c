#include "oled.h"
#include <stdio.h>
#include <string.h>

/* SSD1306的I2C底层: 控制字节0x00后面跟命令,0x40后面跟数据 */

static void OLED_WriteCmd(uint8_t cmd)
{
    HAL_I2C_Mem_Write(OLED_I2C, OLED_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 100);
}

static void OLED_WriteData(const uint8_t *data, uint16_t len)
{
    HAL_I2C_Mem_Write(OLED_I2C, OLED_ADDR, 0x40, I2C_MEMADD_SIZE_8BIT,
                      (uint8_t *)data, len, 100);
}

static void OLED_SetCursor(uint8_t x, uint8_t page)
{
    OLED_WriteCmd(0xB0 | (page & 0x07));         // 页地址
    OLED_WriteCmd(0x00 | (x & 0x0F));            // 列地址低4位
    OLED_WriteCmd(0x10 | ((x >> 4) & 0x0F));     // 列地址高4位
}

void OLED_Init(void)
{
    /* 上电初始化序列,照SSD1306手册来的 */
    static const uint8_t initCmds[] = {
        0xAE,             // 关显示
        0xD5, 0x80,       // 时钟分频
        0xA8, 0x3F,       // 复用率64行
        0xD3, 0x00,       // 显示偏移0
        0x40,             // 起始行0
        0x8D, 0x14,       // 开内部电荷泵
        0x20, 0x02,       // 页寻址模式
        0xA1,             // 段重映射
        0xC8,             // COM扫描方向
        0xDA, 0x12,       // COM引脚
        0x81, 0xCF,       // 对比度
        0xD9, 0xF1,       // 预充电周期
        0xDB, 0x40,       // VCOMH电压
        0xA4,             // 显示内容来自GDDRAM
        0xA6,             // 正常显示
        0xAF,             // 开显示
    };
    uint8_t i;

    for (i = 0; i < sizeof(initCmds); i++) {
        OLED_WriteCmd(initCmds[i]);
    }
    OLED_Clear();
}

void OLED_Clear(void)
{
    static const uint8_t zeros[128] = {0};
    uint8_t page;

    for (page = 0; page < OLED_PAGES; page++) {
        OLED_SetCursor(0, page);
        OLED_WriteData(zeros, 128);
    }
}

/* ASCII字模,6x8(5x7点+1列间距),只有0-9 A-Z和几个符号,够显示用了 */
static const uint8_t Font6x8[][5] = {
    /* 0-9 */
    {0x3E,0x51,0x49,0x45,0x3E}, /* 0 */
    {0x00,0x42,0x7F,0x40,0x00}, /* 1 */
    {0x42,0x61,0x51,0x49,0x46}, /* 2 */
    {0x21,0x41,0x45,0x4B,0x31}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10}, /* 4 */
    {0x27,0x45,0x45,0x45,0x39}, /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30}, /* 6 */
    {0x01,0x71,0x09,0x05,0x03}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36}, /* 8 */
    {0x06,0x49,0x49,0x29,0x1E}, /* 9 */
    /* A-Z */
    {0x7E,0x11,0x11,0x11,0x7E}, /* A */
    {0x7F,0x49,0x49,0x49,0x36}, /* B */
    {0x3E,0x41,0x41,0x41,0x22}, /* C */
    {0x7F,0x41,0x41,0x22,0x1C}, /* D */
    {0x7F,0x49,0x49,0x49,0x41}, /* E */
    {0x7F,0x09,0x09,0x09,0x01}, /* F */
    {0x3E,0x41,0x49,0x49,0x7A}, /* G */
    {0x7F,0x08,0x08,0x08,0x7F}, /* H */
    {0x00,0x41,0x7F,0x41,0x00}, /* I */
    {0x20,0x40,0x41,0x3F,0x01}, /* J */
    {0x7F,0x08,0x14,0x22,0x41}, /* K */
    {0x7F,0x40,0x40,0x40,0x40}, /* L */
    {0x7F,0x02,0x0C,0x02,0x7F}, /* M */
    {0x7F,0x04,0x08,0x10,0x7F}, /* N */
    {0x3E,0x41,0x41,0x41,0x3E}, /* O */
    {0x7F,0x09,0x09,0x09,0x06}, /* P */
    {0x3E,0x41,0x51,0x21,0x5E}, /* Q */
    {0x7F,0x09,0x19,0x29,0x46}, /* R */
    {0x46,0x49,0x49,0x49,0x31}, /* S */
    {0x01,0x01,0x7F,0x01,0x01}, /* T */
    {0x3F,0x40,0x40,0x40,0x3F}, /* U */
    {0x1F,0x20,0x40,0x20,0x1F}, /* V */
    {0x3F,0x40,0x38,0x40,0x3F}, /* W */
    {0x63,0x14,0x08,0x14,0x63}, /* X */
    {0x07,0x08,0x70,0x08,0x07}, /* Y */
    {0x61,0x51,0x49,0x45,0x43}, /* Z */
    /* 常用符号 */
    {0x00,0x00,0x00,0x00,0x00}, /* 空格 */
    {0x00,0x36,0x36,0x00,0x00}, /* : */
    {0x00,0x60,0x60,0x00,0x00}, /* . */
    {0x08,0x08,0x08,0x08,0x08}, /* - */
    {0x20,0x10,0x08,0x04,0x02}, /* / */
    {0x23,0x13,0x08,0x64,0x62}, /* % */
    {0x00,0x41,0x22,0x14,0x08}, /* > */
    {0x08,0x14,0x22,0x41,0x00}, /* < */
    {0x08,0x08,0x3E,0x08,0x08}, /* + */
    {0x14,0x14,0x14,0x14,0x14}, /* = */
    {0x00,0x00,0x5F,0x00,0x00}, /* ! */
};

static const uint8_t *OLED_GetGlyph(char ch)
{
    static const uint8_t spaceGlyph[5] = {0, 0, 0, 0, 0};

    if (ch >= '0' && ch <= '9') return Font6x8[ch - '0'];
    if (ch >= 'A' && ch <= 'Z') return Font6x8[ch - 'A' + 10];
    switch (ch) {
        case ' ': return Font6x8[36];
        case ':': return Font6x8[37];
        case '.': return Font6x8[38];
        case '-': return Font6x8[39];
        case '/': return Font6x8[40];
        case '%': return Font6x8[41];
        case '>': return Font6x8[42];
        case '<': return Font6x8[43];
        case '+': return Font6x8[44];
        case '=': return Font6x8[45];
        case '!': return Font6x8[46];
        default:  return spaceGlyph;
    }
}

void OLED_ShowChar(uint8_t x, uint8_t page, char ch)
{
    static const uint8_t zeroByte = 0x00;
    const uint8_t *glyph = OLED_GetGlyph(ch);

    OLED_SetCursor(x, page);
    OLED_WriteData(glyph, 5);          // 5列字形
    OLED_WriteData(&zeroByte, 1);      // 1列字间距,加起来6像素宽
}

void OLED_ShowString(uint8_t x, uint8_t page, const char *str)
{
    while (*str) {
        if (x > OLED_COLS - 6) {       // 超宽换到下一显示行
            x = 0;
            page += 2;
        }
        if (page >= OLED_PAGES) return;
        OLED_ShowChar(x, page, *str++);
        x += 6;
    }
}

/* 行缓冲补空格到21个字符宽,把上一帧的字盖掉(21x6=126<128) */
static void LinePad(char *line)
{
    int len = (int)strlen(line);
    while (len < 21) {
        line[len++] = ' ';
    }
    line[21] = '\0';
}

/* OLED显示任务: 阻塞等工况队列,刷新转速/位置/模式/目标 */
void OledDispTask(void *arg)
{
    (void)arg;
    SysState_t st;
    char line[22];

    for (;;) {
        /* 等电机控制任务经队列推数据过来 */
        if (xQueueReceive(g_statusQueue, &st, portMAX_DELAY) != pdPASS) {
            continue;
        }
        /* 队列可能积了几条,只渲染最新的 */
        SysState_t tmp;
        while (xQueueReceive(g_statusQueue, &tmp, 0) == pdPASS) {
            st = tmp;
        }

        /* 第1行: 转速 */
        snprintf(line, sizeof(line), "SPD:%5d RPM", st.rpm);
        LinePad(line);
        OLED_ShowString(0, 0, line);

        /* 第2行: 位置 */
        snprintf(line, sizeof(line), "POS:%7d C", (int)st.position);
        LinePad(line);
        OLED_ShowString(0, 2, line);

        /* 第3行: 运行模式 */
        const char *modeStr = (st.mode == MODE_POSITION) ? "POSITION" :
                              (st.mode == MODE_SPEED)    ? "SPEED"    : "STOP";
        snprintf(line, sizeof(line), "MOD:%s", modeStr);
        LinePad(line);
        OLED_ShowString(0, 4, line);

        /* 第4行: 目标值 */
        snprintf(line, sizeof(line), "TGT:%7d", (int)st.target);
        LinePad(line);
        OLED_ShowString(0, 6, line);
    }
}
