#pragma once

#include <Arduino.h>


typedef struct {
  float pad[2] = {}; // 0、电压    1、百分比
} Battery_t;
extern Battery_t batteryStatus; // 电池状态结构体，在数据发送中定义

typedef struct {
  int    adcValue[4]     = {}; // 0、差速        1、油门          2、副翼         3、升降舵
  int8_t switchStatus[3] = {}; // 0、发送开关    1、自稳开关      2、襟翼开关
} sendData_t;
extern sendData_t sendData; // 发送数据结构体，在sendData.cpp中定义

typedef struct {
  float batteryValue[2] = {}; // 0、电压           1、百分比
} Aircraft;
extern Aircraft aircraft; // 飞机数据结构体

typedef struct {
  int     adcValue[4]     = {}; // 0、差速         1、油门          2、副翼         3、升降舵
  int     icon[3]         = {}; // 0、发送         1、蜂鸣器        2、连接状态
  float   batteryValue[4] = {}; // 0、遥控器电压   1、遥控器电量    2、飞机电压      3、飞机电量
  uint8_t page, num;            // 页面
} OLED_t;
extern OLED_t oled; // OLED显示数据结构体，在oled.cpp中定义

extern bool buzzerFlag;        // 在button.cpp中定义
extern bool oled_display_flag; // 在button.cpp中定义
extern bool esp_connected;     // 在sendData.cpp中定义

extern uint8_t RC_num;         // 在button.cpp中定义
extern uint8_t buzzerMode;     // 在button.cpp中定义
extern int     esp_now_signal; // 在sendData.cpp中定义

void buzzer(uint8_t mode);
