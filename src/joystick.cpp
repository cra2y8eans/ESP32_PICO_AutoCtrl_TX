#include "joystick.h"
#include "Arduino.h"
#include "common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "input_device.h"
#include "my_analog_hat.h"

// #define DEBUG
#define STICK_L_HORI 34 // 左摇杆水平
#define STICK_L_VERT 39 // 左摇杆垂直
#define STICK_R_HORI 32 // 右摇杆水平
#define STICK_R_VERT 35 // 右摇杆垂直

void getADCvalue(void* pvParameters) {
  uint8_t          switchLastStatus[3] = { 0 }; // 存储四个摇杆的ADC值
  TickType_t       xLastWakeTime       = xTaskGetTickCount();
  const TickType_t xPeriod             = pdMS_TO_TICKS(10); // 频率 100Hz → 周期为 1/100 = 0.01 秒 = 10 毫秒
  while (1) {
    if (xQueueReceive(SwitchEventQueue, switchLastStatus, 10) == pdPASS) {
      sendData.switchStatus[0] = switchLastStatus[0]; // 发送开关
      sendData.switchStatus[1] = switchLastStatus[1]; // 自稳开关
      sendData.switchStatus[2] = switchLastStatus[2]; // 襟翼开关
    }
    if (sendData.switchStatus[0]) {
      sendData.adcValue[0] = getAnalogHat(diffrential); // 左摇杆水平
      sendData.adcValue[1] = getAnalogHat(throttle);    // 左摇杆垂直
      sendData.adcValue[2] = getAnalogHat(aileron);     // 右摇杆水平
      sendData.adcValue[3] = getAnalogHat(elevator);    // 右摇杆垂直
    } else {
      sendData.adcValue[0] = 0; // 关闭发送按钮或关机断联
      sendData.adcValue[1] = -255;
      sendData.adcValue[2] = 0;
      sendData.adcValue[3] = 0;
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
#ifdef DEBUG
    static int count = 0;
    if (++count >= 50) {
      count = 0;
      Serial.printf("ADC value from joystick: LH:%d, LV:%d, RH:%d, RV:%d\n",
          sendData.adcValue[0], sendData.adcValue[1], sendData.adcValue[2], sendData.adcValue[3]);
      Serial.printf("Switches status from joystick: SEND:%d AUTO:%d FLAP:%d\n",
          sendData.switchStatus[0], sendData.switchStatus[1], sendData.switchStatus[2]);
    }
#endif
  }
}

void joystick_init() {
  xTaskCreatePinnedToCore(getADCvalue, "getADCvalue", 1024 * 4, NULL, 1, NULL, 1);
}