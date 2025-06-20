#pragma once

#include <Arduino.h>

extern QueueHandle_t BatteryToBuzzerQueue;   // 按钮到OLED的消息队列

void battery_init(); // 电池初始化函数
void batteryReadingTask(void* pvParameters); // 电池读取任务函数
// void lowBatteryAlarmTask(void* pvParameters); // 低电量报警任务函数