#pragma once

#include <Arduino.h>

extern uint8_t RC_num;
extern int     send_icon, esp_now_signal; // 声明来自 senData.cpp 的全局变量
extern bool    oled_display_flag;         // 声明来自 button.h 的全局变量

void    oled_init();
void    oled_task(void* pvParameters);
uint8_t get_MAC_address();
