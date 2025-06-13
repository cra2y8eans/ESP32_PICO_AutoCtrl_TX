#include "oled.h"
#include "ESP_NOW/sendData.h"
#include "button/button.h"
#include "buzzer/buzzer.h"
#include "filter/my_analog_hat.h"
#include <U8g2lib.h>
#include <Wire.h>
#include <arduino.h>
#include <freertos/FreeRTOS.h>

#define DEBUG

#define SDA_PIN 21
#define SCL_PIN 22
#define OLED_I2C_ADDR 0x3C // oled屏幕I2C地址

#define SPEAKER_ON 59239
#define SPEAKER_OFF 59215
#define LOCK 0xe72e
#define UNLOCK 0xe785
#define SEND_ON 0xE898
#define SEND_OFF 0xf140
#define ICON_AIRCRAFT 0xe709
#define ICON_HANDHELD 0xe7fc
#define SEND_FAILED 0xe71b

#define ADC_MIN 0                         // ADC最小值
#define ADC_MAX = pow(2, ADC_RESOLUTION); // ADC最大值
#define ADC_OUT_MIN -255                  // 摇杆输出ADC最小值
#define ADC_OUT_MAX 255                   // 摇杆输出ADC最大值

#define SERVO_MAX_ANGLE 120 // 舵机最大角度

#define QUEUE_MESSAGE_WAIT 20

// 构造oled对象
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(
    /*旋转角度*/ U8G2_R0,
    /*重启引脚*/ U8X8_PIN_NONE,
    /*SCL引脚*/ SCL_PIN,
    /*SDA引脚*/ SDA_PIN);

uint8_t num        = 2; // 总页数
uint8_t page       = 0; // 正在显示的页面
uint8_t progress   = 0;
uint8_t RC_num     = 0; // 接收机编号
String  RC_version = "";

int speaker = SPEAKER_ON; // 扬声器图标
int esp_now_signal, lock;

static Pad data;

void oled_init() {
  setupAnalogHat();
  u8g2.begin();
  u8g2.enableUTF8Print();
  xTaskCreatePinnedToCore(oled_task, "oled_task", 1024 * 2, NULL, 1, NULL, 1);
#ifdef DEBUG
  Serial.println(oled_task == NULL ? "OLED任务创建失败" : "OLED任务创建成功");
#endif
}

void unlock() {
  bool          paringMax  = false;
  bool          paringMin  = false;
  bool          RC_confirm = false;
  buzzerStatuas buzzerMode;

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
      paringMax  = true;
      buzzerMode = BUZZER_SHORT;
      xQueueSend(BuzzerEventQueue, &buzzerMode, QUEUE_MESSAGE_WAIT / portTICK_PERIOD_MS);
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
      buzzerMode = BUZZER_SHORT;
      xQueueSend(BuzzerEventQueue, &buzzerMode, QUEUE_MESSAGE_WAIT / portTICK_PERIOD_MS);
    }
    if (reading < ADC_OUT_MIN + 50) {
      RC_num     = 1;
      RC_confirm = true;
      RC_version = "1.01";
      buzzerMode = BUZZER_SHORT;
      xQueueSend(BuzzerEventQueue, &buzzerMode, QUEUE_MESSAGE_WAIT / portTICK_PERIOD_MS);
    }
  }
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
      paringMin  = true;
      buzzerMode = BUZZER_LONG;
      xQueueSend(BuzzerEventQueue, &buzzerMode, QUEUE_MESSAGE_WAIT / portTICK_PERIOD_MS);
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

uint8_t get_MAC_address() {
  return RC_num;
#ifdef DEBUG
  Serial.println("MAC address have been returned: " + String(get_MAC_address()));
#endif
}

void oledButtonEvent() {
  ButtonState button_state;
  xQueueReceive(ButtonEventQueue, &button_state, QUEUE_MESSAGE_WAIT / portTICK_PERIOD_MS);
  oled_display_flag = button_state == BUTTON_R_LONG_PRESS ? !oled_display_flag : oled_display_flag;
  if (oled_display_flag == true) {
    switch (button_state) {
    case BUTTON_L_SHORT_PRESS:
      num  = num + 1;
      page = num % 2;
      break;
    case BUTTON_R_SHORT_PRESS:
      if (page > 0) {
        num = num - 1;
      } else {
        num = 0;
      }
      page = num % 2;
      break;
    default:
      break;
    }
  }
}

void displayContent(){
    xQueueReceive(PadDataQueue, &data, QUEUE_MESSAGE_WAIT / portTICK_PERIOD_MS);

}

void oleddisplay() {
  if (oled_display_flag == true) {
    int throttle = map(data.joystick_cur_val[0], ADC_OUT_MIN, ADC_OUT_MAX, ADC_MIN, 255);
    int aileron  = map(data.joystick_cur_val[2], ADC_OUT_MIN, ADC_OUT_MAX, ADC_MIN, SERVO_MAX_ANGLE);
    int elevator = map(data.joystick_cur_val[3], ADC_OUT_MIN, ADC_OUT_MAX, ADC_MIN, (SERVO_MAX_ANGLE - 20));
    switch (page) {
    case 0:
      // 设备状态
      u8g2.clearBuffer();
      u8g2.setFont(aircraft_14);
      u8g2.drawGlyph(59, 14, speaker);       // 扬声器图标
      u8g2.drawGlyph(2, 63, ICON_HANDHELD);  // 手柄图标
      u8g2.drawGlyph(86, 62, ICON_AIRCRAFT); // 飞机图标
      // 手柄电量
      u8g2.setCursor(24, 61);
      u8g2.setFont(u8g2_font_7x14B_tf);
      u8g2.printf("%.0f%%", data.padPercentage);
      // 信号
      u8g2.setFont(aircraft_pad_icon_14);
      u8g2.drawGlyph(2, 12, esp_now_signal);   // 信号图标
      u8g2.drawGlyph(110, 13, data.send_icon); // 发送开关图标
      // 飞机电量
      u8g2.setCursor(106, 61);
      u8g2.setFont(u8g2_font_7x14B_tf);
      u8g2.printf("%.0f%%", data.airCraftPercentage);
      // 副翼
      u8g2.setCursor(6, 38);
      u8g2.setFont(u8g2_font_7x14B_tf);
      u8g2.printf("%02d°", aileron);
      // 升降舵
      u8g2.setCursor(102, 38);
      u8g2.setFont(u8g2_font_7x14B_tf);
      u8g2.printf("%02d°", elevator);
      // 油门
      u8g2.setFont(u8g2_font_logisoso22_tr);
      u8g2.setCursor(42, 42);
      u8g2.printf("%03d", throttle);
      u8g2.sendBuffer();
      break;
    case 1:
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_wqy12_t_gb2312b);
      u8g2.drawUTF8(5, 15, "电量");
      u8g2.setCursor(5, 35);
      u8g2.printf("遥控器: %.2fv", data.padBatteryVoltage);
      u8g2.setCursor(5, 55);
      u8g2.printf("接收机: %.2fv", data.airCraftBatteryVoltage);
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

void oled_task(void* pvParameters) {
  unlock();
  while (1) {
    oledButtonEvent();
    displayContent();
    oleddisplay();
  }
}