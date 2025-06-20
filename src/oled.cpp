#include "oled.h"
#include "common.h"
#include <U8g2lib.h>
#include <Wire.h>
#include <arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// #define DEBUG

#define SDA_PIN 21
#define SCL_PIN 22
#define OLED_I2C_ADDR 0x3C // oled屏幕I2C地址

#define ICON_AIRCRAFT 0xe709
#define ICON_HANDHELD 0xe7fc
#define SEND_FAILED 0xe71b // 双圈

// 构造oled对象
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(
    /*旋转角度*/ U8G2_R0,
    /*重启引脚*/ U8X8_PIN_NONE,
    /*SCL引脚*/ SCL_PIN,
    /*SDA引脚*/ SDA_PIN);

void oled_task(void* pvParameters) {
  u8g2.begin();
  u8g2.enableUTF8Print();
  if (oled_display_flag == true) {
    switch (oled.page) {
    case 0:
      // 设备状态
      u8g2.clearBuffer();
      u8g2.setFont(aircraft_14);
      u8g2.drawGlyph(59, 14, oled.icon[1]);  // 扬声器图标
      u8g2.drawGlyph(2, 63, ICON_HANDHELD);  // 手柄图标
      u8g2.drawGlyph(86, 62, ICON_AIRCRAFT); // 飞机图标
      // 手柄电量
      u8g2.setCursor(24, 61);
      u8g2.setFont(u8g2_font_7x14B_tf);
      u8g2.printf("%.0f%%", oled.batteryValue[1]);
      // 信号
      u8g2.setFont(aircraft_pad_icon_14);
      u8g2.drawGlyph(2, 12, esp_now_signal); // 信号图标
      u8g2.drawGlyph(110, 13, oled.icon[0]); // 发送开关图标
      // 飞机电量
      u8g2.setCursor(106, 61);
      u8g2.setFont(u8g2_font_7x14B_tf);
      u8g2.printf("%.0f%%", oled.batteryValue[4]);
      // 副翼
      u8g2.setCursor(6, 38);
      u8g2.setFont(u8g2_font_7x14B_tf);
      u8g2.printf("%02d°", oled.adcValue[2]);
      // 升降舵
      u8g2.setCursor(102, 38);
      u8g2.setFont(u8g2_font_7x14B_tf);
      u8g2.printf("%02d°", oled.adcValue[3]);
      // 油门
      u8g2.setFont(u8g2_font_logisoso22_tr);
      u8g2.setCursor(42, 42);
      u8g2.printf("%03d", oled.adcValue[1]);
      u8g2.sendBuffer();
      break;
    case 1:
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_wqy12_t_gb2312b);
      u8g2.drawUTF8(5, 15, "电量");
      u8g2.setCursor(5, 35);
      u8g2.printf("遥控器: %.2fv", oled.batteryValue[0]);
      u8g2.setCursor(5, 55);
      u8g2.printf("接收机: %.2fv", oled.batteryValue[2]);
      u8g2.sendBuffer();
      break;
    default:
      break;
    }
  } else {
    u8g2.clearBuffer();
    u8g2.sendBuffer();
  }
}

void oled_init() {
  xTaskCreatePinnedToCore(oled_task, "oled_task", 1024 * 4, NULL, 1, NULL, 1);
#ifdef DEBUG
  Serial.println(oled_task == NULL ? "OLED任务创建失败" : "OLED任务创建成功");
#endif
}