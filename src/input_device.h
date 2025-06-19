#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>

// 按钮状态枚举
enum ButtonState {
  BUTTON_L_1_SHORT_PRESS,  // 左按钮短按
  BUTTON_L_1_LONG_PRESS,   // 左按钮长按

  BUTTON_L_2_SHORT_PRESS,  // 左按钮短按
  BUTTON_L_2_LONG_PRESS,   // 左按钮长按

  BUTTON_R_1_SHORT_PRESS,  // 右按钮短按
  BUTTON_R_1_LONG_PRESS,   // 右按钮长按

  BUTTON_R_2_SHORT_PRESS,  // 右按钮短按
  BUTTON_R_2_LONG_PRESS,   // 右按钮长按
};

extern QueueHandle_t ButtonToOledQueue;
extern QueueHandle_t ButtonToBuzzerQueue;
extern QueueHandle_t SwitchEventQueue;

// 按钮初始化和任务函数
void input_device_init();
void input_device_task(void* pvParameters);
