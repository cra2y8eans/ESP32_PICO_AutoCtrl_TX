#ifndef _SEND_DATA_H_
#define _SEND_DATA_H_

#include <Arduino.h>

typedef struct {
  int   joystick_cur_val[4] = {}; // 0、油门        1、差速         2、副翼         3、升降舵
  int   joystick_adj_val[3] = {}; // 0、差速        1、副翼         2、升降舵
  int   send_icon, esp_now_signal;
  float airCraftPercentage, airCraftBatteryVoltage, padPercentage, padBatteryVoltage;
} Pad;

typedef struct {
  float batteryValue[2] = {}; // 0、电压           1、电量
} Aircraft;

typedef struct {
  int     joystick_cur_val[4] = {}; // 0、油门        1、差速         2、副翼         3、升降舵
  int     joystick_adj_val[3] = {}; // 0、差速        1、副翼         2、升降舵
  float   diffrential_coe;
  uint8_t switch_status[5] = {}; // 0、方向开关    1、副翼开关     2、升降舵开关   3、差速（自稳）开关      4、襟翼开关
} SendData;

extern QueueHandle_t PadDataQueue;

void sendData_init();
void sendDataTask(void* pvParameters);

#endif