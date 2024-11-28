
/******************************************************************************************************

ESP32_PICO 手抛飞机自稳遥控器

- 版本：1.0.0    1、 发送包括：自稳开启flag，摇杆中值，油门、升降舵、副翼的ADC值，X轴、Y轴的PID参数，转向系数等。
                 2、 oled显示pid参数、调整步长、飞机当前姿态
                 3、 将button按钮功能分为不同功能的函数，功能如下：
                    * Xp = 右摇杆（左-、右+），Xi = btn_R（左-、右+），Xd = btn_L（左-、右+）
                    * Yp = 右摇杆（左-、右+），Yi = btn_R（左-、右+），Yd = btn_L（左-、右+）
                    * step = btn_L（同时打开两个钮子开关，左-、右+）
                    * coe =  btn_R（同时打开两个钮子开关，左-、右+）
                    * page = 左1-、右1+（钮子开关全部关闭）
                    * buzzer = 左2长按，oled = 右2长按
                 4、 PID、step和coe的调整只能在数据发送开关关闭并且调整开关打开的时候才能进行
                 5、 开机时需要采集一次摇杆中值，并发送到接收端作为摇杆是否拨动的参考值。
                 6、 ESP NOW回调函数尽量不要执行复杂的代码。
                 7、 取消滤波之后摇杆之间相互串扰消失，但是ADC读数会有波动，导致舵机有轻微抽搐。
                 8、 将解锁进度条读取动画放在setup中执行，确保只执行一次。
                 9、 重写电池电量算法
                 10、使用git管理代码

*******************************************************************************************************/

#include <Arduino.h>
#include <Ticker.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>

/*------------------------------------------------- ESP NOW -------------------------------------------------*/

uint8_t airCraftAddress[] = { 0xf0, 0x24, 0xf9, 0x8f, 0xb3, 0x9c }; // PICO_1

// 创建ESP NOW通讯实例
esp_now_peer_info_t peerInfo;

struct Pad {
  int   button_flag[2]      = {}; // 0、自稳开关         1、襟翼开关
  int   joystick_mid_val[2] = {}; // 0、副翼中值         1、升降舵中值
  int   motor_servo_ADC[3]  = {}; // 0、油门             1、副翼；       2、升降舵
  float x_pid_data[3]       = {}; // 0、X轴比例          1、X轴积分；    2、X轴微分
  float y_pid_data[3]       = {}; // 0、Y轴比例          1、Y轴积分；    2、Y轴微分
  float coe[1]              = {}; // 0、舵机角度换算系数
};
Pad pad;

struct Aircraft {
  int   batteryValue[1] = {}; // 0、电池电量ADC值
  int   servo_angle[2]  = {}; // 0、升降舵机角度   1、副翼舵机角度
  float x_data[2]       = {}; // 0、X轴角度        1、X轴角速度
  float y_data[2]       = {}; // 0、Y轴角度        1、Y轴角速度
};
Aircraft aircraft;

bool esp_connected;

/*------------------------------------------------- PID -------------------------------------------------*/

float Xp = -4.0, Xi = -0.01, Xd = -0.2,
      Yp = -4.0, Yi = -0.01, Yd = -0.2;

float P_adj_step = 0.5, I_adj_step = 0.01, D_adj_step = 0.01;

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

byte   num         = 4;
byte   page        = 0;
byte   progress    = 0;
String RSSI_status = "";

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

volatile bool paringMax    = false; // 油门推到最大标志位
volatile bool paringMin    = false; // 油门推到最小标志位
volatile bool angleConfirm = false; // 舵机角度标志位

/*-------------------------------------------------- 按钮 --------------------------------------------------*/

#define BUTTON_L_1 13
#define BUTTON_L_2 26
#define BUTTON_R_1 16
#define BUTTON_R_2 17

byte          button_pin;
byte          buttonArray[]      = { BUTTON_L_1, BUTTON_L_2, BUTTON_R_1, BUTTON_R_2 };
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
#define BATTERY_MIN_PERCENTAGE 20         // 电池最低百分比
#define PAD_BATTERY_READING_INTERVAL 2000 // 采样间隔
#define PAD_BATTERAY_COE 6.9              // 遥控器电量换算系数
#define BATTERAY_COE 7                    // 接收机电量换算系数

unsigned long previousPadBattery = 0; // 电量读取时间判断

float
    // 遥控端
    padBatterayReading, // 遥控器ADC电量读取
    padBatteryVoltage,  // 遥控器电池电量
    padPercentage,      // 遥控器电量百分比

    // 飞机端
    airCraftBatteryReading,
    airCraftBatteryVoltage, // 飞行器电池电量 单位v
    airCraftPercentage;     // 飞行器电量百分比

/*----------------------------------------------- 微调&襟翼&油门开关 ------------------------------------------------*/

#define BUTTON_THROTTLE 25   // 油门开关
#define BUTTON_FINETUNING 18 // 微调开关
#define BUTTON_FLAP 19       // 襟翼开关

String adj_switch_X = "";
String adj_switch_Y = "";

/*------------------------------------------------ 摇杆滤波 ------------------------------------------------*/

#define STICK_THROTTLE 39   // 油门
#define STICK_ELEVATOR 35   // 升降舵
#define STICK_AILERON 32    // 副翼
#define STICK_ADJUSTMENT 34 // 步长调整
#define LIMIT_FILTER 10     // 限幅滤波阈值，建议取值范围3~10，值越小，操控越需要柔和
#define AVERAGE_FILTER 50   // 均值滤波，N次取样平均，建议取值范围20~80
#define ADC_MIN 0
#define ADC_MAX 255

// 摇杆中间值
byte
    // left_y_mid,  // 空
    left_y_min,  // 油门最小值
    left_x_mid,  // 油门
    right_x_mid, // 升降舵
    right_y_mid; // 副翼

float turn_coe = 0.46875;

/*------------------------------------------------- 自定义函数 -------------------------------------------------*/

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
  while (paringMax == false) {
    int reading = analogRead(STICK_THROTTLE);
    u8g2.clearBuffer();
    u8g2.setFont(pad_35);
    u8g2.drawGlyph(44, 38, lock);
    u8g2.setFont(u8g2_font_wqy12_t_gb2312b);
    u8g2.drawUTF8(18, 56, "请将油门推到最大");
    u8g2.sendBuffer();
    if (reading > ADC_MAX - 10) {
      paringMax = true;
      buzzer(1);
    }
  }
  delay(500);
  while (paringMax == true && paringMin == false) {
    int reading = analogRead(STICK_THROTTLE);
    lock        = 0xe785;
    u8g2.clearBuffer();
    u8g2.setFont(pad_35);
    u8g2.drawGlyph(44, 38, lock);
    u8g2.setFont(u8g2_font_wqy12_t_gb2312b);
    u8g2.drawUTF8(18, 56, "再将油门推到最小");
    u8g2.sendBuffer();
    if (reading < ADC_MIN + 2) {
      paringMin = true;
      buzzer(0);
    }
    while (paringMax == true && paringMin == true && progress < 100) {
      progress += 2;
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_wqy14_t_gb2312b);
      u8g2.drawUTF8(33, 26, "解锁中...");
      u8g2.drawFrame(9, 40, 110, 16);
      u8g2.drawBox(14, 45, progress, 6);
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
    padBatterayReading = limit_avg_filter(BATTERY_PIN);
    padBatteryVoltage  = (padBatterayReading / ADC_MAX) * PAD_BATTERAY_COE;
    padPercentage      = (1.2 - (BATTERY_MAX_VALUE - padBatteryVoltage)) / 1.2 * 100;
    // 接收机电量
    airCraftBatteryReading = aircraft.batteryValue[0];
    airCraftBatteryVoltage = (airCraftBatteryReading / ADC_MAX) * BATTERAY_COE;                // 转换成电压值，单位v
    airCraftPercentage     = (1.2 - (BATTERY_MAX_VALUE - airCraftBatteryVoltage)) / 1.2 * 100; // 转换成百分比
  }
  // 低电量报警
  if (esp_connected && (airCraftPercentage <= BATTERY_MIN_PERCENTAGE || padPercentage <= BATTERY_MIN_PERCENTAGE)) {
    buzzer(1);
  }
}

// 获取初始参数
void getJoyStickValue() {
  // 摇杆参数初始化
  analogReadResolution(8); // 设置测量精度为8位
  left_x_mid  = 127;
  right_x_mid = analogRead(STICK_ELEVATOR);
  right_y_mid = analogRead(STICK_AILERON);
  // 电量初始化
  padBatterayReading = limit_avg_filter(BATTERY_PIN);
  padBatteryVoltage  = (padBatterayReading / ADC_MAX) * PAD_BATTERAY_COE;
  padPercentage      = (1.2 - (BATTERY_MAX_VALUE - padBatteryVoltage)) / 1.2 * 100;
}

// 钮子开关及摇杆调参
void handleSWfunction() {
  // X轴PID调参
  if ((digitalRead(BUTTON_FINETUNING) == 1) && (digitalRead(BUTTON_FLAP) == 0)) {
    adj_switch_X      = "开";
    oled_display_flag = true;
    num               = 5;
    page              = num % 4;
    // 比例项调参
    if (analogRead(STICK_AILERON) > 200) {
      Xp += P_adj_step;
    } else if (analogRead(STICK_AILERON) < 35) {
      Xp -= P_adj_step;
    }
  } else {
    adj_switch_X = "关";
  }
  // Y轴PID调参
  if ((digitalRead(BUTTON_FINETUNING) == 0) && (digitalRead(BUTTON_FLAP) == 1)) {
    adj_switch_Y      = "开";
    oled_display_flag = true;
    num               = 6;
    page              = num % 4;
    // 比例项调参
    if (analogRead(STICK_AILERON) > 200) {
      Yp += P_adj_step;
    } else if (analogRead(STICK_AILERON) < 35) {
      Yp -= P_adj_step;
    }
  } else {
    adj_switch_Y = "关";
  }
  // 转向系数步长调整
  if (oled_display_flag == true && (digitalRead(BUTTON_FINETUNING) == 1) && (digitalRead(BUTTON_FLAP) == 1)) {
    if (analogRead(STICK_ADJUSTMENT) > 200) {
      turn_coe += 0.1;
    } else if (analogRead(STICK_ADJUSTMENT) < 35) {
      turn_coe -= 0.1;
    }
  }
  //
  // 发送开关
  if (digitalRead(BUTTON_THROTTLE) == true) {
    send_icon = 0xE898;
  } else {
    send_icon = 0xf140;
  }
}

// 短按按钮功能
void btnShortPressed() {
  /*
   * Xp = 左摇杆（左-、右+），Xi = btn_R（左-、右+），Xd = btn_L（左-、右+）
   * Yp = 左摇杆（左-、右+），Yi = btn_R（左-、右+），Yd = btn_L（左-、右+）
   * step = btn_L（同时打开两个钮子开关，左-、右+）
   * coe =  btn_R（同时打开两个钮子开关，左-、右+）
   * page = 左1-、右1+（钮子开关全部关闭）
   */

  //  翻页
  if (oled_display_flag == true && (digitalRead(BUTTON_FINETUNING) == 0) && (digitalRead(BUTTON_FLAP) == 0)) {
    switch (button_pin) {
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
    default:
      break;
    }
  }
  // X轴
  if (oled_display_flag == true && (digitalRead(BUTTON_FINETUNING) == 1) && (digitalRead(BUTTON_FLAP) == 0)) {
    switch (button_pin) {
      // 积分项I
    case BUTTON_R_1:
      Xi -= I_adj_step;
      break;
    case BUTTON_R_2:
      Xi += I_adj_step;
      break;
      // 微分项D
    case BUTTON_L_1:
      Xd += D_adj_step;
      break;
      // 左按键
    case BUTTON_L_2:
      Xd -= D_adj_step;
      break;
    default:
      break;
    }
  }
  // Y轴
  if (oled_display_flag == true && (digitalRead(BUTTON_FINETUNING) == 0) && (digitalRead(BUTTON_FLAP) == 1)) {
    switch (button_pin) {
      // 积分项I
    case BUTTON_R_1:
      Yi -= I_adj_step;
      break;
    case BUTTON_R_2:
      Yi += I_adj_step;
      break;
      // 微分项D
    case BUTTON_L_1:
      Yd += D_adj_step;
      break;
    case BUTTON_L_2:
      Yd -= D_adj_step;
      break;
    default:
      break;
    }
  }
  // PID步长调整
  if (oled_display_flag == true && (digitalRead(BUTTON_FINETUNING) == 1) && (digitalRead(BUTTON_FLAP) == 1)) {
    switch (button_pin) {
    case BUTTON_L_1:
      I_adj_step += 0.01;
      break;
    case BUTTON_L_2:
      I_adj_step -= 0.01;
      break;
    case BUTTON_R_1:
      D_adj_step -= 0.01;
      break;
    case BUTTON_R_2:
      D_adj_step += 0.01;
      break;
    default:
      break;
    }
  }
}

// 按钮长按功能
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
    button_flag[2]      = 0、自稳开关         1、襟翼开关
    joystick_mid_val[2] = 0、副翼中值         1、升降舵中值
    motor_servo_ADC[3]  = 0、油门             1、副翼；       2、升降舵
    x_pid_data[3]       = 0、X轴比例          1、X轴积分；    2、X轴微分
    y_pid_data[3]       = 0、Y轴比例          1、Y轴积分；    2、Y轴微分
    coe[1]              = 0、舵机角度换算系数
  */
  pad.button_flag[0]      = digitalRead(BUTTON_THROTTLE);
  pad.button_flag[1]      = digitalRead(BUTTON_FLAP);
  pad.joystick_mid_val[0] = right_x_mid;
  pad.joystick_mid_val[1] = right_y_mid;
  pad.motor_servo_ADC[0]  = analogRead(STICK_THROTTLE);
  pad.motor_servo_ADC[1]  = analogRead(STICK_AILERON);
  pad.motor_servo_ADC[2]  = analogRead(STICK_ELEVATOR);
  pad.x_pid_data[0]       = Xp;
  pad.x_pid_data[1]       = Xi;
  pad.x_pid_data[2]       = Xd;
  pad.y_pid_data[0]       = Yp;
  pad.y_pid_data[1]       = Yi;
  pad.y_pid_data[2]       = Yd;
  pad.coe[0]              = turn_coe;

  // 油门修整
  if (pad.motor_servo_ADC[0] <= left_x_mid) {
    pad.motor_servo_ADC[0] = map(pad.motor_servo_ADC[0], ADC_MIN, left_x_mid, ADC_MIN, 127);
  } else {
    pad.motor_servo_ADC[0] = map(pad.motor_servo_ADC[0], left_x_mid + 1, ADC_MAX, 128, ADC_MAX);
  }
  // 副翼修正
  if (pad.motor_servo_ADC[1] <= right_y_mid) {
    pad.motor_servo_ADC[1] = map(pad.motor_servo_ADC[1], ADC_MIN, right_y_mid, ADC_MIN, 127);
  } else {
    pad.motor_servo_ADC[1] = map(pad.motor_servo_ADC[1], right_y_mid + 1, ADC_MAX, 128, ADC_MAX);
  }
  // 升降舵修正
  if (pad.motor_servo_ADC[2] <= right_x_mid) {
    pad.motor_servo_ADC[2] = map(pad.motor_servo_ADC[2], ADC_MIN, right_x_mid, ADC_MIN, 127);
  } else {
    pad.motor_servo_ADC[2] = map(pad.motor_servo_ADC[2], right_x_mid + 1, ADC_MAX, 128, ADC_MAX);
  }

  esp_now_send(airCraftAddress, (uint8_t*)&pad, sizeof(pad));

  /*
  // 发送
  if (digitalRead(BUTTON_THROTTLE) == true) {
    if (digitalRead(BUTTON_FLAP) == true && digitalRead(BUTTON_FINETUNING) == false) {
    }
    esp_now_send(airCraftAddress, (uint8_t*)&pad, sizeof(pad));
  } else {
    pad.button_flag[0]  = 0;
    pad.motor_servo_ADC[0] = 0;
    pad.motor_servo_ADC[1] = right_y_mid;
    pad.motor_servo_ADC[2] = right_x_mid;
    esp_now_send(airCraftAddress, (uint8_t*)&pad, sizeof(pad));
  }
  */
}

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

// OLED显示
void oledDisplay() {
  if (oled_display_flag == true) {
    switch (page) {
    case 0:
      // 第一页，设备状态
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
      u8g2.setCursor(105, 12);
      u8g2.setFont(u8g2_font_6x13B_tf);
      u8g2.printf("%s", RSSI_status);
      // 飞机电量
      u8g2.setCursor(106, 61);
      u8g2.setFont(u8g2_font_7x14B_tf);
      u8g2.printf("%.0f%%", airCraftPercentage);
      // 副翼
      u8g2.setCursor(6, 38);
      u8g2.setFont(u8g2_font_7x14B_tf);
      u8g2.printf("%02d°", pad.motor_servo_ADC[1] = map(pad.motor_servo_ADC[1], 0, 255, 0, 120));
      // 升降舵
      u8g2.setCursor(102, 38);
      u8g2.setFont(u8g2_font_7x14B_tf);
      u8g2.printf("%02d°", pad.motor_servo_ADC[2] = map(pad.motor_servo_ADC[2], 0, 255, 0, 120));
      // 油门
      u8g2.setFont(u8g2_font_logisoso22_tr);
      u8g2.setCursor(42, 42);
      u8g2.printf("%03d", pad.motor_servo_ADC[0]);
      u8g2.sendBuffer();
      break;
    case 1:
      // 第二页 X轴PID
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_wqy12_t_gb2312b);
      u8g2.drawUTF8(5, 10, "Pitch");
      u8g2.setCursor(75, 10);
      u8g2.printf("调参 : %s", adj_switch_X); // 微调开关
      // pid值
      u8g2.setCursor(5, 30);
      u8g2.printf("P: %.1f", Xp);
      u8g2.setCursor(45, 30);
      u8g2.printf("I: %.2f", Xi);
      u8g2.setCursor(85, 30);
      u8g2.printf("D: %.2f", Xd);
      // step值
      u8g2.setCursor(5, 45);
      u8g2.printf("%.1f", P_adj_step);
      u8g2.setCursor(45, 45);
      u8g2.printf("%.2f", I_adj_step);
      u8g2.setCursor(85, 45);
      u8g2.printf("%.2f", D_adj_step);
      // 飞机姿态
      u8g2.setCursor(5, 60);
      u8g2.printf("A: %.1f", aircraft.x_data[0]);
      u8g2.setCursor(50, 60);
      u8g2.printf("G: %.1f", aircraft.x_data[1]);
      u8g2.setCursor(95, 60);
      u8g2.printf("S: %d", aircraft.servo_angle[0]);
      u8g2.sendBuffer();
      break;
    case 2:
      // 第三页 Y轴PID
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_wqy12_t_gb2312b);
      u8g2.drawUTF8(5, 10, "Roll");
      u8g2.setCursor(75, 10);
      u8g2.printf("调参 : %s", adj_switch_Y); // 微调开关
      // pid值
      u8g2.setCursor(5, 30);
      u8g2.printf("P: %.1f", Yp);
      u8g2.setCursor(45, 30);
      u8g2.printf("I: %.2f", Yi);
      u8g2.setCursor(85, 30);
      u8g2.printf("D: %.2f", Yd);
      // step值
      u8g2.setCursor(5, 45);
      u8g2.printf("%.1f", P_adj_step);
      u8g2.setCursor(45, 45);
      u8g2.printf("%.2f", I_adj_step);
      u8g2.setCursor(85, 45);
      u8g2.printf("%.2f", D_adj_step);
      // 飞机姿态
      u8g2.setCursor(5, 60);
      u8g2.printf("A: %.1f", aircraft.y_data[0]);
      u8g2.setCursor(50, 60);
      u8g2.printf("G: %.1f", aircraft.y_data[1]);
      u8g2.setCursor(95, 60);
      u8g2.printf("S: %d", aircraft.servo_angle[1]);
      u8g2.sendBuffer();
      break;
    case 3:
      // 第四页 电量详情
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_wqy12_t_gb2312b);
      u8g2.drawUTF8(5, 15, "电量");
      u8g2.setCursor(5, 35);
      u8g2.printf("遥控 : %.0f", padBatterayReading);
      u8g2.setCursor(75, 35);
      u8g2.printf("%.2fv", padBatteryVoltage);
      u8g2.setCursor(5, 55);
      u8g2.printf("飞机 : %d", aircraft.batteryValue[0]);
      u8g2.setCursor(75, 55);
      u8g2.printf("%.2fv", airCraftBatteryVoltage);
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

  // 遥控器参数初始化
  getJoyStickValue();

  // 解锁配对
  unlock();

  // wifi及ESP NOW初始化
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

void loop() {
  BatteryReading();
  transmitData();
  button_identify();
  oledDisplay();
  handleSWfunction();
}