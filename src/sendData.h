#ifndef _SEND_DATA_H_
#define _SEND_DATA_H_

#include <Arduino.h>

extern int send_icon, esp_now_signal;
extern QueueHandle_t PadDataQueue;
extern bool esp_connected;

typedef struct {
  int     joystick_cur_val[4] = {}; // 0、油门        1、差速         2、副翼         3、升降舵
  // float   diffrential_coe;
  uint8_t switch_status[3] = {}; // 0、发送开关    1、差速（自稳）开关      2、襟翼开关
} PadData;

typedef struct {
  float batteryValue[2] = {}; // 0、电压           1、电量
} Aircraft;

// typedef struct {
//   int   joystick_cur_val[4] = {}; // 0、油门        1、差速         2、副翼         3、升降舵
//   int   send_icon, esp_now_signal;
//   float airCraftPercentage, airCraftBatteryVoltage, padPercentage, padBatteryVoltage;
// } Pad;



// extern QueueHandle_t PadDataQueue;

void sendData_init();
void mainTask(void* pvParameters);

#endif