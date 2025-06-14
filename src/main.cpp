/**
 * ESP32_PICO 手抛飞机自稳遥控器  分支： multifunctional
 *
 *
 * 将遥控器功能分为OLED显示、按钮判断、蜂鸣器、ESP NOW通讯等功能模块，通过freeRTOS任务运行
 * 将解锁操作整合到OLED任务中，将钮子开关判断、电池电量接收和读取整合到ESP NOW任务中
 * 各功能模块通过队列通信，实现模块间解耦
 * RC MAC地址获取、蜂鸣器和OLED标志位通过extern修饰的全局变量实现
 */


#include <Arduino.h>
#include "sendData.h"
#include "oled.h"

#define ADC_RESOLUTION 12

void setup() {
  Serial.begin(115200);
  analogReadResolution(ADC_RESOLUTION);
  oled_init();
  sendData_init();
  // buzzer_init();
  // button_init();
  vTaskDelete(NULL);
}
void loop() {
}
