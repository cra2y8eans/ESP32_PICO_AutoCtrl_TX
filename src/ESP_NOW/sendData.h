#ifndef _SEND_DATA_H_
#define _SEND_DATA_H_

#include <Arduino.h>

typedef struct {
  int   button_status[3]    = {}; // 0、自稳开关    1、襟翼开关     2、微调开关
  int   joystick_cur_val[4] = {}; // 0、油门        1、差速         2、副翼         3、升降舵
  int   send_icon;
  float diffrential_coe,
      padPercentage,
      airCraftPercentage,
      padBatteryVoltage,
      airCraftBatteryVoltage;
} Pad;

extern QueueHandle_t padDataQueueOLED;

void sendData_init();

#endif