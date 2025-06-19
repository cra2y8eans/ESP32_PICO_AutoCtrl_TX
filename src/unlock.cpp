#pragma once
#include "buzzer.h"
#include "common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "input_device.h"
#include "my_analog_hat.h"
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define OLED_I2C_ADDR 0x3C // oled屏幕I2C地址
#define LOCK 0xe72e
#define UNLOCK 0xe785
#define ADC_MIN 0                         // ADC最小值
#define ADC_MAX = pow(2, ADC_RESOLUTION); // ADC最大值
#define ADC_OUT_MIN -255                  // 摇杆输出ADC最小值
#define ADC_OUT_MAX 255                   // 摇杆输出ADC最大值

#define QUEUE_MESSAGE_WAIT 10

// 构造oled对象
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2_unlock(
    /*旋转角度*/ U8G2_R0,
    /*重启引脚*/ U8X8_PIN_NONE,
    /*SCL引脚*/ SCL_PIN,
    /*SDA引脚*/ SDA_PIN);

uint8_t RC_num = 0; // 接收机编号

void unlock() {

  u8g2_unlock.begin();
  u8g2_unlock.enableUTF8Print();
  bool    paringMax  = false;
  bool    paringMin  = false;
  bool    RC_confirm = false;
  int     lock;
  uint8_t progress   = 0;
  String  RC_version = "";

  buzzerStatuas buzzerMode;

  while (paringMax == false) {
    int reading = getAnalogHat(throttle);
    lock        = LOCK;
    u8g2_unlock.clearBuffer();
    u8g2_unlock.setFont(pad_35);
    u8g2_unlock.drawGlyph(44, 38, lock);
    u8g2_unlock.setFont(u8g2_font_wqy12_t_gb2312b);
    u8g2_unlock.drawUTF8(18, 56, "请将油门推到最大");
    u8g2_unlock.sendBuffer();
    if (reading > ADC_OUT_MAX - 10) {
      paringMax  = true;
      buzzerMode = BUZZER_SHORT;
      xQueueSend(ButtonToBuzzerQueue, &buzzerMode, QUEUE_MESSAGE_WAIT / portTICK_PERIOD_MS);
    }
  }
  delay(500);
  while (paringMax == true && RC_confirm == false && paringMin == false) {
    int reading = getAnalogHat(aileron);
    u8g2_unlock.clearBuffer();
    u8g2_unlock.setFont(u8g2_font_wqy12_t_gb2312b);
    u8g2_unlock.drawUTF8(30, 20, "请选择接收机");
    u8g2_unlock.drawUTF8(10, 50, "v 1.01");
    u8g2_unlock.drawUTF8(80, 50, "v 1.02");
    u8g2_unlock.sendBuffer();
    if (reading > ADC_OUT_MAX - 50) {
      RC_num     = 2;
      RC_confirm = true;
      RC_version = "1.02";
      buzzerMode = BUZZER_SHORT;
      xQueueSend(ButtonToBuzzerQueue, &buzzerMode, QUEUE_MESSAGE_WAIT / portTICK_PERIOD_MS);
    }
    if (reading < ADC_OUT_MIN + 50) {
      RC_num     = 1;
      RC_confirm = true;
      RC_version = "1.01";
      buzzerMode = BUZZER_SHORT;
      xQueueSend(ButtonToBuzzerQueue, &buzzerMode, QUEUE_MESSAGE_WAIT / portTICK_PERIOD_MS);
    }
  }
  while (paringMax == true && RC_confirm == true && paringMin == false) {
    int reading = getAnalogHat(throttle);
    lock        = UNLOCK;
    u8g2_unlock.clearBuffer();
    u8g2_unlock.setFont(pad_35);
    u8g2_unlock.drawGlyph(44, 38, lock);
    u8g2_unlock.setFont(u8g2_font_wqy12_t_gb2312b);
    u8g2_unlock.drawUTF8(18, 56, "再将油门推到最小");
    u8g2_unlock.sendBuffer();
    if (reading < ADC_OUT_MIN + 2) {
      paringMin  = true;
      buzzerMode = BUZZER_LONG;
      xQueueSend(ButtonToBuzzerQueue, &buzzerMode, QUEUE_MESSAGE_WAIT / portTICK_PERIOD_MS);
    }
    while (paringMax == true && paringMin == true && progress < 100) {
      progress += 2;
      u8g2_unlock.clearBuffer();
      u8g2_unlock.setFont(u8g2_font_wqy14_t_gb2312b);
      u8g2_unlock.drawUTF8(33, 18, "解锁中...");
      u8g2_unlock.drawFrame(9, 28, 110, 16); // x、y、w、h
      u8g2_unlock.drawBox(14, 33, progress, 6);
      u8g2_unlock.setCursor(40, 63);
      u8g2_unlock.printf("RC %s", RC_version);
      u8g2_unlock.sendBuffer();
    }
  }
}
