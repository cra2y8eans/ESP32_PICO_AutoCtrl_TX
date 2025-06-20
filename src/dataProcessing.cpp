#include "dataProcessing.h"
#include "common.h"
#include "input_device.h"
#include <Arduino.h>

#define ADC_MIN 0                         // ADC最小值
#define ADC_MAX = pow(2, ADC_RESOLUTION); // ADC最大值
#define ADC_OUT_MIN -255                  // 摇杆输出ADC最小值
#define ADC_OUT_MAX 255                   // 摇杆输出ADC最大值

#define SPEAKER_ON 59239
#define SPEAKER_OFF 59215
#define SEND_ON 0xE898
#define SEND_OFF 0xf140
#define ESP_NOW_CONNECTED 0xe870
#define ESP_NOW_DISCONNECTED 0xe791

#define TOTAL_PAGES 2 // OLED总页数

#define SERVO_MAX_ANGLE 120 // 舵机最大角度

OLED_t      oled;
ButtonState btnState;

void AssignValues() {
  oled.icon[0]         = sendData.switchStatus[0] ? SEND_ON : SEND_OFF;                                        // 发送开关图标
  oled.icon[1]         = buzzerFlag ? SPEAKER_ON : SPEAKER_OFF;                                                // 蜂鸣器图标
  oled.icon[2]         = esp_connected ? ESP_NOW_CONNECTED : ESP_NOW_DISCONNECTED;                             // 连接状态图标                                          // 连接状态图标
  oled.adcValue[1]     = map(sendData.adcValue[0], ADC_OUT_MIN, ADC_OUT_MAX, ADC_MIN, 255);                    // 油门
  oled.adcValue[2]     = map(sendData.adcValue[1], ADC_OUT_MIN, ADC_OUT_MAX, ADC_MIN, SERVO_MAX_ANGLE);        // 副翼
  oled.adcValue[3]     = map(sendData.adcValue[2], ADC_OUT_MIN, ADC_OUT_MAX, ADC_MIN, (SERVO_MAX_ANGLE - 20)); // 升降舵
  oled.batteryValue[0] = batteryStatus.pad[0];                                                                 // 遥控器电压
  oled.batteryValue[1] = batteryStatus.pad[1];                                                                 // 遥控器电量
  oled.batteryValue[2] = aircraft.batteryValue[0];                                                             // 飞机电压
  oled.batteryValue[3] = aircraft.batteryValue[1];                                                             // 飞机电量
  if (xQueueReceive(ButtonToOledQueue, &btnState, 0) == pdTRUE) {
    switch (btnState) {
    case BUTTON_L_1_SHORT_PRESS:
      oled.num -= 1;
      oled.page = oled.num % TOTAL_PAGES;
      break;
      break;
    case BUTTON_R_1_SHORT_PRESS:
      oled.num += 1;
      oled.page = oled.num % TOTAL_PAGES;
      break;
    default:
      break;
    }
  }
}

void dataProcessingTask(void* pvParameters) {
  TickType_t       xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency    = pdMS_TO_TICKS(20); // 50Hz
  oled.num                       = TOTAL_PAGES;       // 总页数
  oled.page                      = 0;                 // 页面初始化
  while (1) {
    AssignValues();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void dataProcessingInit() {
  xTaskCreatePinnedToCore(dataProcessingTask, "dataProcessingTask", 1024 * 4, NULL, 1, NULL, 1);
}