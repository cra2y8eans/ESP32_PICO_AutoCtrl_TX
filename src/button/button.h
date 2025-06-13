#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>

// 按钮状态枚举
enum ButtonState {
  BUTTON_L_SHORT_PRESS,  // 左按钮短按
  BUTTON_L_LONG_PRESS,   // 左按钮长按
  BUTTON_L_REPEAT_PRESS, // 左按钮重复按下

  BUTTON_R_SHORT_PRESS,  // 右按钮短按
  BUTTON_R_LONG_PRESS,   // 右按钮长按
  BUTTON_R_REPEAT_PRESS, // 右按钮重复按下
};

extern QueueHandle_t ButtonEventQueue;
extern bool          buzzerFlag;
extern bool          oled_display_flag;

// 按钮初始化和任务函数
void button_init();
void button_task(void* pvParameters);
