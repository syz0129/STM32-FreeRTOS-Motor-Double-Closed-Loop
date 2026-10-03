#ifndef __OLED_H
#define __OLED_H

#include "main.h"
#include "app_config.h"

/* OLED: SSD1306 128x64, I2C接口 */
#define OLED_I2C       (&hi2c1)          // CubeMX里配的I2C1,PB6/PB7
#define OLED_ADDR      (0x3C << 1)       // 7位地址0x3C,左移1位=写地址
#define OLED_COLS      128
#define OLED_PAGES     8                 // 每页8行,8页=64行

extern I2C_HandleTypeDef hi2c1;

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t x, uint8_t page, char ch);    // 字符宽6像素
void OLED_ShowString(uint8_t x, uint8_t page, const char *str);

/* OLED显示任务: 等队列里的工况数据,刷新转速/位置/模式/目标 */
void OledDispTask(void *arg);

#endif /* __OLED_H */
