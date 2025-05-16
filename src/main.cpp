
/******************************************************************************************************

ESP32_PICO 手抛飞机自稳遥控器

          把遥控器发送的数据统一起来，实现一个遥控器连接操控多个设备的功能。
          使用none的低通滤波校正摇杆虚位和死区问题.

*******************************************************************************************************/

#include "batteryReading.hpp"
#include "my_analog_hat.h"
#include <Arduino.h>
#include <Ticker.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>

/*------------------------------------------------- ESP NOW -------------------------------------------------*/

// uint8_t RC_coreless_c3mini[]  = { 0x9c, 0x9e, 0x6e, 0x86, 0x2b, 0x48 }; // 有刷c3mini（舵机）
uint8_t RC_autoControl_pico[] = { 0xf0, 0x24, 0xf9, 0x8f, 0xb3, 0x9c }; // PICO_1 自稳
uint8_t RC_brushless_1_0_1[]  = { 0x48, 0xca, 0x43, 0xed, 0xc4, 0x80 }; // 无刷v1.01
uint8_t RC_brushless_1_0_2[]  = { 0x48, 0xca, 0x43, 0xed, 0xc4, 0x58 }; // 无刷v1.02
uint8_t RC_coreless_c3mini[]  = { 0x9c, 0x9e, 0x6e, 0x84, 0xf2, 0x1c }; // 有刷c3mini（差速）
uint8_t airCraftAddress[6]    = {};
// 创建ESP NOW通讯实例
esp_now_peer_info_t peerInfo;

struct Pad {
  int   button_status[3]    = {}; // 0、自稳开关    1、襟翼开关     2、微调开关
  int   joystick_cur_val[4] = {}; // 0、油门        1、差速         2、副翼         3、升降舵
  float diffrential_coe;
};
Pad pad;

struct Aircraft {
  float batteryValue[2] = {}; // 0、电压           1、电量
};
Aircraft aircraft;

bool esp_connected;

/*------------------------------------------------- oled -------------------------------------------------*/

#define SDA_PIN 21
#define SCL_PIN 22
#define OLED_I2C_ADDR 0x3C // oled屏幕I2C地址

// 构造oled对象
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(
    /*旋转角度*/ U8G2_R0,
    /*重启引脚*/ U8X8_PIN_NONE,
    /*SCL引脚*/ SCL_PIN,
    /*SDA引脚*/ SDA_PIN);

volatile bool oled_display_flag = true;

uint8_t num      = 4; // 总页数
uint8_t page     = 0; // 正在显示的页面
uint8_t progress = 0;
// String RSSI_status = "";

int speaker        = 59239;  // 扬声器图标
int esp_now_signal = 0xe870; // 连接图标
int send_icon      = 0xe71b; // 发送开关图标
int lock           = 0xe72e;

/*------------------------------------------------- 蜂鸣器 -------------------------------------------------*/

#define BUZZER_PIN 23 // 蜂鸣器
volatile bool buzzer_flag    = true;
unsigned long previousBuzzer = 0; // 蜂鸣器时间判断

Ticker buzzerMode;

/*------------------------------------------------- 开机锁 -------------------------------------------------*/

// volatile bool paringMax  = false; // 油门推到最大标志位
// volatile bool paringMin  = false; // 油门推到最小标志位
// volatile bool RC_confirm = false; // 接收机选择标志位
uint8_t RC_num     = 0; // 接收机编号
String  RC_version = "";

/*-------------------------------------------------- 按钮 --------------------------------------------------*/

#define BUTTON_L_1 13
#define BUTTON_L_2 26
#define BUTTON_R_1 16
#define BUTTON_R_2 17

uint8_t       button_pin;
uint8_t       buttonArray[]      = { BUTTON_L_1, BUTTON_L_2, BUTTON_R_1, BUTTON_R_2 };
const int     debounceDelay      = 50;    // 去抖延迟
const int     longPressDuration  = 1000;  // 长按持续时间（毫秒）
int           buttonState        = HIGH;  // 初始按键状态
int           lastButtonState    = HIGH;  // 上一次的按键状态
unsigned long lastDebounceTime   = 0;     // 上一次去抖的时间
unsigned long pressStartTime     = 0;     // 按键按下的起始时间
bool          longPressTriggered = false; // 是否已经触发了长按

/*-------------------------------------------------- 电量 --------------------------------------------------*/

#define BATTERY_PIN 36                    // 电池电量读取引脚
#define BATTERY_MAX_VALUE 4.2             // 电池最大电量
#define BATTERY_MIN_VALUE 3.2             // 电池最小电量
#define BATTERY_MIN_PERCENTAGE 20         // 电池最低百分比
#define PAD_BATTERY_READING_INTERVAL 2000 // 采样间隔
#define R1 10000
#define R2 9950

unsigned long previousPadBattery = 0; // 电量读取时间判断

float
    // 遥控端
    padBatteryVoltage, // 遥控器电池电量
    padPercentage,     // 遥控器电量百分比
    // 飞机端
    airCraftBatteryVoltage, // 飞行器电池电量 单位v
    airCraftPercentage;     // 飞行器电量百分比

BatReading battery; // 电池电量读取类的初始化

/*----------------------------------------------- 微调&襟翼&油门开关 ------------------------------------------------*/

#define BUTTON_THROTTLE 25   // 油门开关
#define BUTTON_FINETUNING 18 // 微调开关
#define BUTTON_FLAP 19       // 襟翼开关

String finetuning_btn_status = "";
String flap_btn_status       = "";

/*------------------------------------------------ 摇杆滤波 ------------------------------------------------*/

#define STICK_THROTTLE 39    // 油门
#define STICK_DIFFRENTIAL 34 // 差速
#define STICK_ELEVATOR 35    // 升降舵
#define STICK_AILERON 32     // 副翼
#define LIMIT_FILTER 10      // 限幅滤波阈值，建议取值范围3~10，值越小，操控越需要柔和
#define AVERAGE_FILTER 50    // 均值滤波，N次取样平均，建议取值范围20~80
#define SERVO_MAX_ANGLE 120  // 舵机最大角度
#define ADC_RESOLUTION 12    // ADC精度
#define ADC_MIN 0            // ADC最小值
#define ADC_OUT_MIN -255     // ADC最小值
#define ADC_OUT_MAX 255      // ADC最小值

int   ADC_MAX         = pow(2, ADC_RESOLUTION);            // ADC最大值
float diffrential_coe = 0.35, diffrential_adj_step = 0.01; // 转向系数和微调步长

// // 摇杆初始读数
// int left_y_mid;  // 差速中间值
// int left_y_min;  // 油门最小值
// int left_x_mid;  // 油门中间值
// int right_x_mid; // 升降舵中间值
// int right_y_mid; // 副翼中间值

// int throttle_base_val;   // 油门基础值
// int diffrential_reading; // 差速摇杆读数
// int motor_l_diffrential; // 左电机转向差速值
// int motor_r_diffrential; // 右电机转向差速值

/*------------------------------------------------- 自定义函数 -------------------------------------------------*/

// 数据发出去之后的回调函数
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  // 如果发送成功
  if (status == ESP_NOW_SEND_SUCCESS) {
    esp_connected  = true;
    esp_now_signal = 0xe870;
  } else {
    esp_now_signal = 0xe791;
    esp_connected  = false;
  }
}

// 收到消息后的回调
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  memcpy(&aircraft, incomingData, sizeof(aircraft));
}

// ESP NOW 初始化及连接
void esp_now_connect() {
  WiFi.mode(WIFI_STA); // 设置wifi为STA模式
  WiFi.begin();
  esp_now_init();                       // 初始化ESP NOW
  esp_now_register_send_cb(OnDataSent); // 注册发送成功的回调函数
  esp_now_register_recv_cb(OnDataRecv); // 注册接受数据后的回调函数

  // 注册通信频道
  memcpy(peerInfo.peer_addr, airCraftAddress, 6); // 设置配对设备的MAC地址并储存，参数为拷贝地址、拷贝对象、数据长度
  peerInfo.channel = 1;                           // 设置通信频道
  esp_now_add_peer(&peerInfo);                    // 添加通信对象
}

// 计时器函数
void toggle() {
  digitalWrite(BUZZER_PIN, LOW);
}

// 蜂鸣器
void buzzer(int mode) {
  if (buzzer_flag == true) {
    switch (mode) {
    case 0: // 长鸣
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerMode.once_ms(1000, toggle);
      break;
    case 1: // 短鸣
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerMode.once_ms(200, toggle);
      break;
    default:
      break;
    }
  }
}

// 限幅滤波，防止尖端突变
int limit_filter(int pin) {
  static int last_val = analogRead(pin); // 静态变量，只初始化一次，全程序中保存在内存
  int        val      = analogRead(pin);
  if (abs(val - last_val) > LIMIT_FILTER) {
    val = last_val;
  }
  last_val = analogRead(pin);
  return val;
}

// 均值滤波，抑制噪声
int avg_filter(int pin) {
  int val, sum = 0;
  for (int count = 0; count < AVERAGE_FILTER; count++) {
    sum += analogRead(pin);
  }
  val = sum / AVERAGE_FILTER;
  return val;
}

// 限幅滤波+均值滤波
int limit_avg_filter(int pin) {
  int val, sum = 0;
  for (int count = 0; count < AVERAGE_FILTER; count++) {
    sum += limit_filter(pin);
  }
  val = sum / AVERAGE_FILTER;
  return val;
}

// 遥控解锁
void unlock() {
  bool paringMax  = false;
  bool paringMin  = false;
  bool RC_confirm = false;

  while (paringMax == false) {
    int reading = getAnalogHat(throttle);
    u8g2.clearBuffer();
    u8g2.setFont(pad_35);
    u8g2.drawGlyph(44, 38, lock);
    u8g2.setFont(u8g2_font_wqy12_t_gb2312b);
    u8g2.drawUTF8(18, 56, "请将油门推到最大");
    u8g2.sendBuffer();
    if (reading > ADC_OUT_MAX - 10) {
      paringMax = true;
      buzzer(1);
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
      RC_num = 2;
      memcpy(airCraftAddress, RC_brushless_1_0_2, sizeof(RC_brushless_1_0_2));
      buzzer(1);
      RC_confirm = true;
      RC_version = "1.02";
    }
    if (reading < ADC_OUT_MIN + 50) {
      RC_num = 1;
      memcpy(airCraftAddress, RC_brushless_1_0_1, sizeof(RC_brushless_1_0_1));
      buzzer(1);
      RC_confirm = true;
      RC_version = "1.01";
    }
  }
  while (paringMax == true && RC_confirm == true && paringMin == false) {
    int reading = getAnalogHat(throttle);
    lock        = 0xe785;
    u8g2.clearBuffer();
    u8g2.setFont(pad_35);
    u8g2.drawGlyph(44, 38, lock);
    u8g2.setFont(u8g2_font_wqy12_t_gb2312b);
    u8g2.drawUTF8(18, 56, "再将油门推到最小");
    u8g2.sendBuffer();
    if (reading < ADC_OUT_MIN + 2) {
      paringMin = true;
      buzzer(0);
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

// 电压读取与转换
void BatteryReading() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousPadBattery >= PAD_BATTERY_READING_INTERVAL) {
    previousPadBattery = currentMillis;
    // 手柄电量
    BatReading::Bat batStatus = battery.read(AVERAGE_FILTER);
    padBatteryVoltage         = batStatus.voltage;
    padPercentage             = batStatus.voltsPercentage;
    // 接收机电量
    airCraftBatteryVoltage = aircraft.batteryValue[0];
    airCraftPercentage     = aircraft.batteryValue[1];
  }
  // 低电量报警
  if (esp_connected && (airCraftPercentage <= BATTERY_MIN_PERCENTAGE || padPercentage <= BATTERY_MIN_PERCENTAGE)) {
    buzzer(1);
  }
}

// 获取初始参数
// void getJoyStickValue() {
//   // 摇杆参数初始化
//   left_x_mid  = ADC_MAX / 2;                   // 油门中值
//   left_y_mid  = analogRead(STICK_DIFFRENTIAL); // 差速中值
//   right_x_mid = analogRead(STICK_ELEVATOR);    // 升降舵中值
//   right_y_mid = analogRead(STICK_AILERON);     // 副翼中值
// }

// 钮子开关及摇杆调参
void handleSWfunction() {
  // 只有当发送开关打开的时候，其余两个开关才能有效打开。
  // 微调开关
  if (digitalRead(BUTTON_THROTTLE) == 1 && (digitalRead(BUTTON_FINETUNING) == 1)) {
    finetuning_btn_status = "开";
    oled_display_flag     = true;
    num                   = 6;
    page                  = num % 4;
  } else {
    finetuning_btn_status = "关";
  }
  // 襟翼开关
  if (digitalRead(BUTTON_THROTTLE) == 1 && (digitalRead(BUTTON_FLAP) == 1)) {
    flap_btn_status   = "开";
    oled_display_flag = true;
    num               = 5;
    page              = num % 4;
  } else {
    flap_btn_status = "关";
  }
  // 发送开关
  if (digitalRead(BUTTON_THROTTLE) == 1) {
    send_icon = 0xE898;
  } else {
    send_icon = 0xf140;
  }
}

// 短按按钮功能
void btnShortPressed() {
  //  翻页
  if (oled_display_flag == true) {
    switch (button_pin) {
      // 翻页
    case BUTTON_R_1:
      num  = num + 1;
      page = num % 4;
      break;
    case BUTTON_L_1:
      if (page > 0) {
        num = num - 1;
      } else {
        num = 0;
      }
      page = num % 4;
      break;
      // 转向系数微调
    case BUTTON_L_2:
      diffrential_coe -= diffrential_adj_step;
      break;
    case BUTTON_R_2:
      diffrential_coe += diffrential_adj_step;
      break;
    default:
      break;
    }
  }
}

void btnLongPressed() {
  /*  buzzer = 左2长按，oled = 右2长按  */
  switch (button_pin) {
  case BUTTON_R_2:
    oled_display_flag  = !oled_display_flag;
    longPressTriggered = true;
    page               = 0;
    num                = 4;
    break;
  case BUTTON_L_2:
    buzzer(0);
    buzzer_flag = !buzzer_flag;
    if (buzzer_flag == true) {
      speaker = 59239;
    } else {
      speaker = 59215;
    }
    buzzer(0);
    longPressTriggered = true;
    break;
  default:
    break;
  }
}

// 按钮判断
void button_identify() {
  // 判断按下的是哪个按钮
  for (int i = 0; i < 4; i++) {
    if (digitalRead(buttonArray[i]) == HIGH) {
      button_pin = buttonArray[i];
    }
  }
  int reading = digitalRead(button_pin); // 读取按键状态

  // 去抖处理
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if (millis() - lastDebounceTime > debounceDelay) {
    // 更新按键状态
    if (reading != buttonState) {
      buttonState = reading;
      // 检测按键按下
      if (buttonState == HIGH) {
        pressStartTime     = millis();
        longPressTriggered = false;
      } else { // 按键释放
        if (!longPressTriggered) {
          // 短按触发
          btnShortPressed();
        }
        longPressTriggered = false;
      }
    }
    // 检测长按
    if (!longPressTriggered && buttonState == HIGH && millis() - pressStartTime > longPressDuration) {
      // 长按触发
      btnLongPressed();
    }
  }
  lastButtonState = reading;
}

// 数据发送
void transmitData() {
  /*
      int   button_status[3]    = {}; // 0、自稳开关    1、襟翼开关     2、微调开关
      int   joystick_cur_val[4] = {}; // 0、油门        1、差速         2、副翼         3、升降舵
      float diffrential_coe;
  */
  pad.button_status[0] = digitalRead(BUTTON_THROTTLE);
  pad.button_status[1] = digitalRead(BUTTON_FLAP);
  pad.button_status[2] = digitalRead(BUTTON_FINETUNING);

  pad.joystick_cur_val[0] = getAnalogHat(throttle);
  pad.joystick_cur_val[1] = getAnalogHat(diffrential);
  pad.joystick_cur_val[2] = getAnalogHat(aileron);
  pad.joystick_cur_val[3] = getAnalogHat(elevator);
  pad.diffrential_coe     = diffrential_coe;

  esp_now_send(airCraftAddress, (uint8_t*)&pad, sizeof(pad));
}

// OLED显示

// void oledDisplay_main() {
//   int throttle = pad.joystick_cur_val[0];
//   int aileron  = pad.joystick_cur_val[2];
//   int elevator = pad.joystick_cur_val[3];
//   // 设备状态
//   u8g2.clearBuffer();
//   u8g2.setFont(aircraft_14);
//   u8g2.drawGlyph(59, 14, speaker); // 扬声器图标
//   u8g2.drawGlyph(2, 63, 0xe7fc);   // 手柄图标
//   u8g2.drawGlyph(86, 62, 0xe709);  // 飞机图标
//   // 手柄电量
//   u8g2.setCursor(24, 61);
//   u8g2.setFont(u8g2_font_7x14B_tf);
//   u8g2.printf("%.0f%%", padPercentage);
//   // 信号
//   u8g2.setFont(aircraft_pad_icon_14);
//   u8g2.drawGlyph(2, 12, esp_now_signal); // 信号图标
//   u8g2.drawGlyph(110, 13, send_icon);    // 发送开关图标
//   // 连接状态
//   u8g2.setCursor(105, 12);
//   u8g2.setFont(u8g2_font_6x13B_tf);
//   u8g2.printf("%s", RSSI_status);
//   // 飞机电量
//   u8g2.setCursor(106, 61);
//   u8g2.setFont(u8g2_font_7x14B_tf);
//   u8g2.printf("%.0f%%", airCraftPercentage);
//   // 副翼
//   u8g2.setCursor(6, 38);
//   u8g2.setFont(u8g2_font_7x14B_tf);
//   u8g2.printf("%02d°", aileron = map(aileron, ADC_OUT_MIN, ADC_OUT_MAX, ADC_MIN, SERVO_MAX_ANGLE));
//   // 升降舵
//   u8g2.setCursor(102, 38);
//   u8g2.setFont(u8g2_font_7x14B_tf);
//   u8g2.printf("%02d°", elevator = map(elevator, ADC_OUT_MIN, ADC_OUT_MAX, ADC_MIN, SERVO_MAX_ANGLE));
//   // 油门
//   u8g2.setFont(u8g2_font_logisoso22_tr);
//   u8g2.setCursor(42, 42);
//   u8g2.printf("%03d", throttle = map(throttle, ADC_OUT_MIN, ADC_OUT_MAX, ADC_MIN, 256));
//   u8g2.sendBuffer();
// }

// void oledDisplay_servo() {
//   int aileron  = pad.joystick_cur_val[2];
//   int elevator = pad.joystick_cur_val[3];
//   u8g2.clearBuffer();
//   u8g2.setFont(u8g2_font_wqy12_t_gb2312b);
//   u8g2.drawUTF8(5, 10, "舵机");
//   u8g2.setCursor(75, 10);
//   u8g2.printf("襟翼 : %s", flap_btn_status); // 微调开关
//   u8g2.setCursor(5, 30);
//   u8g2.printf("副翼 ADC : %03d", aileron = map(aileron, ADC_OUT_MIN, ADC_OUT_MAX, ADC_MIN, ADC_MAX)); // ADC值
//   u8g2.setCursor(5, 45);
//   u8g2.printf("左 : %02d°", aileron = map(aileron, ADC_OUT_MIN, ADC_OUT_MAX, ADC_MIN, SERVO_MAX_ANGLE)); // 左副翼实时角度
//   u8g2.setCursor(70, 45);
//   u8g2.printf("右 : %02d°", SERVO_MAX_ANGLE - aileron); // 右副翼实时角度
//   u8g2.setCursor(5, 60);
//   u8g2.printf("升降 ADC : %03d", elevator = map(pad.joystick_cur_val[2], ADC_OUT_MIN, ADC_OUT_MAX, ADC_MIN, ADC_MAX)); // ADC值
//   u8g2.setCursor(100, 60);
//   u8g2.printf("%02d°", elevator = map(elevator, ADC_OUT_MIN, ADC_OUT_MAX, ADC_MIN, SERVO_MAX_ANGLE)); // 升降舵实时角度
//   u8g2.sendBuffer();
// }

// void oledDisplay_motor() {
//   int throttle = pad.joystick_cur_val[0];
//   int diffrential_r, diffrential_l;
//   if (pad.joystick_cur_val[1] >= 0) {
//     diffrential_l = pad.joystick_cur_val[1];
//   } else if (pad.joystick_cur_val[1] <= 0) {
//     diffrential_r = abs(pad.joystick_cur_val[1]);
//   }
//   u8g2.clearBuffer();
//   u8g2.setFont(u8g2_font_wqy12_t_gb2312b);
//   u8g2.drawUTF8(5, 10, "差速");
//   u8g2.setCursor(75, 10);
//   u8g2.printf("微调 : %s", finetuning_btn_status); // 微调开关
//   u8g2.setCursor(5, 30);
//   u8g2.printf("油门: %d", throttle = map(throttle, ADC_OUT_MIN, ADC_OUT_MAX, ADC_MIN, 256)); // 8位ADC值
//   u8g2.setCursor(70, 30);
//   u8g2.printf("系数: %.2f", diffrential_coe); // 转向系数
//   u8g2.setCursor(5, 45);
//   u8g2.printf("左 : %d", diffrential_r); // 左电机实时油门8位ADC值  pad.joystick_cur_val[0] = map(pad.joystick_cur_val[0], ADC_MIN, ADC_MAX / 2, ADC_MIN, 128));
//   u8g2.setCursor(70, 45);
//   u8g2.printf("值 : %d", diffrential_r * diffrential_coe); // 左电机转向加速
//   u8g2.setCursor(5, 60);
//   u8g2.printf("右 : %d", diffrential_l); // 右电机实时油门8位ADC值  pad.joystick_cur_val[1] = map(pad.joystick_cur_val[1], ADC_MIN, ADC_MAX / 2, ADC_MIN, 128));
//   u8g2.setCursor(70, 60);
//   u8g2.printf("值 : %d", diffrential_l * diffrential_coe); // 右电机转向加速
//   u8g2.sendBuffer();
// }

// void oledDisplay_battery() {
//   u8g2.clearBuffer();
//   u8g2.setFont(u8g2_font_wqy12_t_gb2312b);
//   u8g2.drawUTF8(5, 15, "电量");
//   u8g2.setCursor(5, 35);
//   u8g2.printf("遥控器: %.2fv", padBatteryVoltage);
//   u8g2.setCursor(5, 55);
//   u8g2.printf("接收机: %.2fv", airCraftBatteryVoltage);
//   u8g2.sendBuffer();
// }

void oledDisplay() {
  int throttle    = pad.joystick_cur_val[0];
  int diffrential = pad.joystick_cur_val[1];
  int aileron     = pad.joystick_cur_val[2];
  int elevator    = pad.joystick_cur_val[3];
  int diffrential_r, diffrential_l, aileron_l, aileron_r;

  diffrential_l = (diffrential >= 0) ? diffrential : 0;
  diffrential_r = (diffrential <= 0) ? abs(diffrential) : 0;
  aileron_l     = (aileron >= 0) ? aileron : 0;
  aileron_r     = (aileron <= 0) ? abs(aileron) : 0;

  if (oled_display_flag == true) {
    switch (page) {
    case 0:
      // 设备状态
      u8g2.clearBuffer();
      u8g2.setFont(aircraft_14);
      u8g2.drawGlyph(59, 14, speaker); // 扬声器图标
      u8g2.drawGlyph(2, 63, 0xe7fc);   // 手柄图标
      u8g2.drawGlyph(86, 62, 0xe709);  // 飞机图标
      // 手柄电量
      u8g2.setCursor(24, 61);
      u8g2.setFont(u8g2_font_7x14B_tf);
      u8g2.printf("%.0f%%", padPercentage);
      // 信号
      u8g2.setFont(aircraft_pad_icon_14);
      u8g2.drawGlyph(2, 12, esp_now_signal); // 信号图标
      u8g2.drawGlyph(110, 13, send_icon);    // 发送开关图标
      // 连接状态
      // u8g2.setCursor(105, 12);
      // u8g2.setFont(u8g2_font_6x13B_tf);
      // u8g2.printf("%s", RSSI_status);
      // 飞机电量
      u8g2.setCursor(106, 61);
      u8g2.setFont(u8g2_font_7x14B_tf);
      u8g2.printf("%.0f%%", airCraftPercentage);
      // 副翼
      u8g2.setCursor(6, 38);
      u8g2.setFont(u8g2_font_7x14B_tf);
      u8g2.printf("%02d°", aileron = map(aileron, ADC_OUT_MIN, ADC_OUT_MAX, ADC_MIN, SERVO_MAX_ANGLE));
      // 升降舵
      u8g2.setCursor(102, 38);
      u8g2.setFont(u8g2_font_7x14B_tf);
      u8g2.printf("%02d°", elevator = map(elevator, ADC_OUT_MIN, ADC_OUT_MAX, ADC_MIN, SERVO_MAX_ANGLE));
      // 油门
      u8g2.setFont(u8g2_font_logisoso22_tr);
      u8g2.setCursor(42, 42);
      u8g2.printf("%03d", throttle = map(throttle, ADC_OUT_MIN, ADC_OUT_MAX, ADC_MIN, 255));
      u8g2.sendBuffer();
      break;
    case 1:
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_wqy12_t_gb2312b);
      u8g2.drawUTF8(5, 10, "舵机");
      u8g2.setCursor(75, 10);
      u8g2.printf("襟翼 : %s", flap_btn_status); // 微调开关
      u8g2.setCursor(5, 30);
      u8g2.printf("副翼 ADC : %03d", aileron = map(aileron, ADC_OUT_MIN, ADC_OUT_MAX, ADC_MIN, ADC_MAX)); // ADC值
      u8g2.setCursor(5, 45);
      u8g2.printf("左 : %02d°", aileron = map(aileron, ADC_MIN, ADC_MAX, ADC_MIN, SERVO_MAX_ANGLE)); // 左副翼实时角度
      u8g2.setCursor(70, 45);
      u8g2.printf("右 : %02d°", SERVO_MAX_ANGLE - aileron); // 右副翼实时角度
      u8g2.setCursor(5, 60);
      u8g2.printf("升降 ADC : %03d", elevator = map(elevator, ADC_OUT_MIN, ADC_OUT_MAX, ADC_MIN, ADC_MAX)); // ADC值
      u8g2.setCursor(100, 60);
      u8g2.printf("%02d°", elevator = map(elevator, ADC_MIN, ADC_MAX, ADC_MIN, SERVO_MAX_ANGLE)); // 升降舵实时角度
      u8g2.sendBuffer();
      break;
    case 2:
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_wqy12_t_gb2312b);
      u8g2.drawUTF8(5, 10, "差速");
      u8g2.setCursor(75, 10);
      u8g2.printf("微调 : %s", finetuning_btn_status); // 微调开关
      u8g2.setCursor(5, 30);
      u8g2.printf("油门: %d", throttle = map(throttle, ADC_OUT_MIN, ADC_OUT_MAX, ADC_MIN, 255)); // 8位ADC值
      u8g2.setCursor(70, 30);
      u8g2.printf("系数: %.2f", diffrential_coe); // 转向系数
      u8g2.setCursor(5, 45);
      u8g2.printf("左 : %d", diffrential_r); // 左电机实时油门8位ADC值
      u8g2.setCursor(70, 45);
      u8g2.printf("值 : %.0f", diffrential_r * diffrential_coe); // 左电机转向加速
      u8g2.setCursor(5, 60);
      u8g2.printf("右 : %d", diffrential_l); // 右电机实时油门8位ADC值
      u8g2.setCursor(70, 60);
      u8g2.printf("值 : %.0f", diffrential_l * diffrential_coe); // 右电机转向加速
      u8g2.sendBuffer();
      break;
    case 3:
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_wqy12_t_gb2312b);
      u8g2.drawUTF8(5, 15, "电量");
      u8g2.setCursor(5, 35);
      u8g2.printf("遥控器: %.2fv", padBatteryVoltage);
      u8g2.setCursor(5, 55);
      u8g2.printf("接收机: %.2fv", airCraftBatteryVoltage);
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

/*-------------------------------------------------------------------------------------------------------------*/

void setup() {
  Serial.begin(115200);
  analogReadResolution(ADC_RESOLUTION); // 设置ADC精度

  // oled初始化
  u8g2.begin();
  u8g2.enableUTF8Print(); // 显示中文使能

  // 引脚初始化
  pinMode(BUTTON_THROTTLE, INPUT_PULLDOWN);   // 油门开关
  pinMode(BUTTON_FINETUNING, INPUT_PULLDOWN); // 微调开关
  pinMode(BUTTON_FLAP, INPUT_PULLDOWN);       // 襟翼开关
  pinMode(BUZZER_PIN, OUTPUT);                // 蜂鸣器引脚
  pinMode(BUTTON_L_1, INPUT_PULLDOWN);
  pinMode(BUTTON_L_2, INPUT_PULLDOWN);
  pinMode(BUTTON_R_1, INPUT_PULLDOWN);
  pinMode(BUTTON_R_2, INPUT_PULLDOWN);

  // 初始化摇杆
  setupAnalogHat();

  // 遥控器参数初始化
  // getJoyStickValue();

  // 遥控器解锁
  unlock();

  // ESP NOW及wifi初始化和连接
  esp_now_connect();

  // 电量读取初始化
  battery.init(BATTERY_PIN, R1, R2, BATTERY_MAX_VALUE, BATTERY_MIN_VALUE);
}

void loop() {
  BatteryReading();
  transmitData();
  button_identify();
  oledDisplay();
  handleSWfunction();
}