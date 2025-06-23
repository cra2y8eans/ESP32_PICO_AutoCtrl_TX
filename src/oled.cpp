#include "oled.h"
#include "buzzer.h"
#include "common.h"
#include "input_device.h"
#include "my_analog_hat.h"
#include <U8g2lib.h>
#include <Wire.h>
#include <arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// #define DEBUG

#define SDA_PIN 21
#define SCL_PIN 22
#define OLED_I2C_ADDR 0x3C // oled屏幕I2C地址

#define ADC_MIN 0                         // ADC最小值
#define ADC_MAX = pow(2, ADC_RESOLUTION); // ADC最大值
#define ADC_OUT_MIN -255                  // 摇杆输出ADC最小值
#define ADC_OUT_MAX 255                   // 摇杆输出ADC最大值

#define LOCK 0xe72e
#define UNLOCK 0xe785
#define ICON_AIRCRAFT 0xe709
#define ICON_HANDHELD 0xe7fc
#define SEND_FAILED 0xe71b // 双圈
#define SPEAKER_ON 59239
#define SPEAKER_OFF 59215
#define SEND_ON 0xE898
#define SEND_OFF 0xf140
#define ESP_NOW_CONNECTED 0xe870
#define ESP_NOW_DISCONNECTED 0xe791

#define TOTAL_PAGES 2 // OLED总页数
#define QUEUE_MESSAGE_WAIT 10
#define SERVO_MAX_ANGLE 120 // 舵机最大角度

// 构造oled对象
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(
    /*旋转角度*/ U8G2_R0,
    /*重启引脚*/ U8X8_PIN_NONE,
    /*SCL引脚*/ SCL_PIN,
    /*SDA引脚*/ SDA_PIN);

uint8_t RC_num = 0; // 接收机编号

OLED_t oled; // OLED显示数据结构体

void AssignValues() {
  ButtonState btnState;
  oled.icon[0]         = sendData.switchStatus[0] ? SEND_ON : SEND_OFF;                                        // 发送开关图标
  oled.icon[1]         = buzzerFlag ? SPEAKER_ON : SPEAKER_OFF;                                                // 蜂鸣器图标
  oled.icon[2]         = esp_connected ? ESP_NOW_CONNECTED : ESP_NOW_DISCONNECTED;                             // 连接状态图标                                          // 连接状态图标
  oled.adcValue[1]     = map(sendData.adcValue[1], ADC_OUT_MIN, ADC_OUT_MAX, ADC_MIN, 255);                    // 油门
  oled.adcValue[2]     = map(sendData.adcValue[2], ADC_OUT_MIN, ADC_OUT_MAX, ADC_MIN, SERVO_MAX_ANGLE);        // 副翼
  oled.adcValue[3]     = map(sendData.adcValue[3], ADC_OUT_MIN, ADC_OUT_MAX, ADC_MIN, (SERVO_MAX_ANGLE - 20)); // 升降舵
  oled.batteryValue[0] = batteryStatus.pad[0];                                                                 // 遥控器电压
  oled.batteryValue[1] = batteryStatus.pad[1];                                                                 // 遥控器电量
  oled.batteryValue[2] = aircraft.batteryValue[0];                                                             // 飞机电压
  oled.batteryValue[3] = aircraft.batteryValue[1];                                                             // 飞机电量
  if (xQueueReceive(ButtonToOledQueue, &btnState, 10) == pdTRUE) {
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

void unlock() {
  bool    paringMax  = false;
  bool    paringMin  = false;
  bool    RC_confirm = false;
  int     lock;
  uint8_t progress   = 0;
  String  RC_version = "";

  // buzzerStatuas buzzerMode;

  while (paringMax == false) {
    int reading = getAnalogHat(throttle);
    lock        = LOCK;
    u8g2.clearBuffer();
    u8g2.setFont(pad_35);
    u8g2.drawGlyph(44, 38, lock);
    u8g2.setFont(u8g2_font_wqy12_t_gb2312b);
    u8g2.drawUTF8(18, 56, "请将油门推到最大");
    u8g2.sendBuffer();
    if (reading > ADC_OUT_MAX - 10) {
      paringMax = true;
      buzzer(1); // 蜂鸣器短响
      // buzzerMode = BUZZER_SHORT;
      // if (ButtonToBuzzerQueue != NULL) {
      //   xQueueSend(ButtonToBuzzerQueue, &buzzerMode, QUEUE_MESSAGE_WAIT / portTICK_PERIOD_MS);
      // }
    }
  }
  delay(500);
  while (paringMax == true && RC_confirm == false && paringMin == false) {
    int reading = getAnalogHat(aileron);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_wqy12_t_gb2312b);
    u8g2.drawUTF8(30, 20, "请选择接收机");
    u8g2.drawUTF8(10, 50, "v 1.01");
    u8g2.drawUTF8(80, 50, "v 1.02");
    u8g2.sendBuffer();
    if (reading > ADC_OUT_MAX - 50) {
      RC_num     = 2;
      RC_confirm = true;
      RC_version = "1.02";
      buzzer(1);
      // buzzerMode = BUZZER_SHORT;
      // if (ButtonToBuzzerQueue != NULL) {
      //   xQueueSend(ButtonToBuzzerQueue, &buzzerMode, QUEUE_MESSAGE_WAIT / portTICK_PERIOD_MS);
      // }
    }
    if (reading < ADC_OUT_MIN + 50) {
      RC_num     = 1;
      RC_confirm = true;
      RC_version = "1.01";
      buzzer(1);
      // buzzerMode = BUZZER_SHORT;
      // if (ButtonToBuzzerQueue != NULL) {
      //   xQueueSend(ButtonToBuzzerQueue, &buzzerMode, QUEUE_MESSAGE_WAIT / portTICK_PERIOD_MS);
      // }
    }
  }
  delay(500);
  while (paringMax == true && RC_confirm == true && paringMin == false) {
    int reading = getAnalogHat(throttle);
    lock        = UNLOCK;
    u8g2.clearBuffer();
    u8g2.setFont(pad_35);
    u8g2.drawGlyph(44, 38, lock);
    u8g2.setFont(u8g2_font_wqy12_t_gb2312b);
    u8g2.drawUTF8(18, 56, "再将油门推到最小");
    u8g2.sendBuffer();
    if (reading < ADC_OUT_MIN + 2) {
      paringMin = true;
      buzzer(2); // 蜂鸣器长响
      // buzzerMode = BUZZER_LONG;
      // if (ButtonToBuzzerQueue != NULL) {
      //   xQueueSend(ButtonToBuzzerQueue, &buzzerMode, QUEUE_MESSAGE_WAIT / portTICK_PERIOD_MS);
      // }
    }
    while (paringMax == true && paringMin == true && progress < 100) {
      progress += 2;
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_wqy14_t_gb2312b);
      u8g2.drawUTF8(33, 18, "解锁中...");
      u8g2.drawFrame(9, 28, 110, 16); // x、y、w、h
      u8g2.drawBox(14, 33, progress, 6);
      u8g2.setCursor(40, 63);
      u8g2.printf("RC %s", RC_version);
      u8g2.sendBuffer();
    }
  }
}

void oled_task(void* pvParameters) {
  u8g2.begin();
  u8g2.enableUTF8Print();
  unlock(); // 解锁遥控器
  while (1) {
    AssignValues();
    vTaskDelay(100 / portTICK_PERIOD_MS);
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
}

void oled_init() {
  xTaskCreatePinnedToCore(oled_task, "oled_task", 1024 * 10, NULL, 1, NULL, 1);
#ifdef DEBUG
  Serial.println(oled_task == NULL ? "OLED任务创建失败" : "OLED任务创建成功");
#endif
}