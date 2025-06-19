#pragma once

#include <Arduino.h>

typedef struct {
  float pad[2]      = {}; // 0、电压    1、百分比
  float aircraft[2] = {}; // 0、电压    1、百分比
} Battery_t;
extern Battery_t batteryStatus; // 电池状态结构体

typedef struct {
  int8_t adcValue[4]     = {}; // 存储四个摇杆的ADC值
  int8_t switchStatus[3] = {}; // 存储摇杆状态
} sendData_t;
extern sendData_t sendData; // 发送数据结构体

extern bool buzzerFlag;        // 在button.cpp中定义
extern bool oled_display_flag; // 在button.cpp中定义

extern uint8_t RC_num;         // 在button.cpp中定义
