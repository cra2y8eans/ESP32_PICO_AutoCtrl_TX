#include "my_analog_hat.h"
#include "sendData.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// #define DEBUG

#define ESP_NOW_CONNECTED 0xe870
#define ESP_NOW_DISCONNECTED 0xe791
#define SEND_ON 0xE898
#define SEND_OFF 0xf140

#define SWITCH_SEND 25 // 发送数据开
#define SWITCH_AUTO 18 // 自稳开关
#define SWITCH_FLAP 19 // 襟翼开关

#define JOYSTICK_ADC_OUT_MAX 255        // 遥控器摇杆输出ADC最大值
#define JOYSTICK_ADC_OUT_MIN -255       // 遥控器摇杆输出ADC最小值
#define ADC_MIN 0                       // ADC最小值
#define SERVO_ANGLE_RANGE 120           // 舵机角度范围
#define ADC_MAX pow(2, ADC_RESOLUTION); // ADC最大值

#define QUEUE_MESSAGE_WAIT 10

int  send_icon = 0, esp_now_signal = 0;
bool esp_connected;

PadData  padData;
Aircraft aircraft;

// // uint8_t RC_coreless_c3mini[]  = { 0x9c, 0x9e, 0x6e, 0x86, 0x2b, 0x48 }; // 有刷c3mini（舵机）
// uint8_t RC_autoControl_pico[] = { 0xf0, 0x24, 0xf9, 0x8f, 0xb3, 0x9c }; // PICO_1 自稳
// uint8_t RC_brushless_1_0_1[]  = { 0x48, 0xca, 0x43, 0xed, 0xc4, 0x80 }; // 无刷v1.01
// uint8_t RC_brushless_1_0_2[]  = { 0x48, 0xca, 0x43, 0xed, 0xc4, 0x58 }; // 无刷v1.02
// uint8_t RC_coreless_c3mini[]  = { 0x9c, 0x9e, 0x6e, 0x84, 0xf2, 0x1c }; // 有刷c3mini（差速）
uint8_t airCraftAddress[6] = { 0x48, 0xca, 0x43, 0xed, 0xc4, 0x80 };

esp_now_peer_info_t peerInfo;
QueueHandle_t       PadDataQueue = NULL;

/**
 * @brief 数据发送成功的回调函数
 * 判断是否发送成功，确定显示图标和连接成功标志位
 */
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    esp_connected  = true;
    esp_now_signal = ESP_NOW_CONNECTED;
  } else {
    esp_now_signal = ESP_NOW_DISCONNECTED;
    esp_connected  = false;
  }
}

void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  memcpy(&aircraft, incomingData, sizeof(aircraft));
  //   batteryReading();
}

/**
 * @brief 发送数据任务
 * 初始化ESP NOW，发送数据
 */
void mainTask(void* pvParameters) {
  WiFi.mode(WIFI_STA); // 设置wifi为STA模式
  WiFi.begin();
  esp_now_init();                       // 初始化ESP NOW
  esp_now_register_send_cb(OnDataSent); // 注册发送成功的回调函数
  esp_now_register_recv_cb(OnDataRecv); // 注册接受数据后的回调函数
  memcpy(peerInfo.peer_addr, airCraftAddress, 6);
  peerInfo.channel = 1;        // 设置通信频道
  esp_now_add_peer(&peerInfo); // 添加通信对象

  TickType_t       xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod       = pdMS_TO_TICKS(12); // 频率 80Hz → 周期为 1/80 = 0.0125 秒 = 12.5 毫秒
  while (1) {
    if (digitalRead(SWITCH_SEND) == 1) {
      padData.switch_status[0]    = digitalRead(SWITCH_SEND);
      padData.switch_status[1]    = digitalRead(SWITCH_AUTO);
      padData.switch_status[2]    = digitalRead(SWITCH_FLAP);
      padData.joystick_cur_val[0] = getAnalogHat(throttle);
      padData.joystick_cur_val[1] = getAnalogHat(diffrential);
      padData.joystick_cur_val[2] = getAnalogHat(aileron);
      padData.joystick_cur_val[3] = getAnalogHat(elevator);
      // send_data.diffrential_coe     = 0.0;
      send_icon = SEND_ON;
      //   esp_now_send(airCraftAddress, (uint8_t*)&padData, sizeof(padData));
    } else {
      // 关闭发送按钮或关机断联
      padData.switch_status[0]    = 0;
      padData.switch_status[1]    = 0;
      padData.switch_status[2]    = 0;
      padData.joystick_cur_val[0] = -255;
      padData.joystick_cur_val[1] = 0;
      padData.joystick_cur_val[2] = 0;
      padData.joystick_cur_val[3] = 0;
      // send_data.diffrential_coe     = 0.0;
      send_icon = SEND_OFF;
    }
    esp_now_send(airCraftAddress, (uint8_t*)&padData, sizeof(padData));
    xQueueSend(PadDataQueue, &padData, 0);
    // Serial.printf("开关：%d\n", padData.switch_status[0]);
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

/**
 * @brief 任务和队列初始化
 */
void sendData_init() {
  pinMode(SWITCH_SEND, INPUT_PULLDOWN);
  PadDataQueue = xQueueCreate(3, sizeof(PadData));
  xTaskCreatePinnedToCore(mainTask, "mainTask", 2048, NULL, 1, NULL, 0);
}

/***********************************************************************************************************************************************************/

// #include "batteryReading.hpp"
// #include "buzzer/buzzer.h"
// #include "button/button.h"
// #include "filter/my_analog_hat.h"
// #include "oled/oled.h"
// #include "ESP_NOW/sendData.h"
// #include <WiFi.h>
// #include <esp_now.h>
// #include <esp_wifi.h>

// #define ESP_NOW_CONNECTED 0xe870
// #define ESP_NOW_DISCONNECTED 0xe791
// #define SEND_ON 0xE898
// #define SEND_OFF 0xf140

// #define ADC_RESOLUTION 12                 // ADC精度
// #define BATTERY_PIN 36                    // 电池电量读取引脚
// #define BATTERY_MAX_VALUE 4.2             // 电池最大电量
// #define BATTERY_MIN_VALUE 3.2             // 电池最小电量
// #define BATTERY_MIN_PERCENTAGE 20         // 电池最低百分比
// #define PAD_BATTERY_READING_INTERVAL 3000 // 采样间隔
// #define R1 10000
// #define R2 9950
// #define AVERAGE_FILTER 50         // 滤波平均次数
// #define BATTERY_MIN_PERCENTAGE 20 // 低电量报警阈值

// #define SWITCH_SEND 25 // 发送数据开
// #define SWITCH_AUTO 18 // 自稳开关
// #define SWITCH_FLAP 19 // 襟翼开关

// #define JOYSTICK_ADC_OUT_MAX 255        // 遥控器摇杆输出ADC最大值
// #define JOYSTICK_ADC_OUT_MIN -255       // 遥控器摇杆输出ADC最小值
// #define ADC_MIN 0                       // ADC最小值
// #define SERVO_ANGLE_RANGE 120           // 舵机角度范围
// #define ADC_MAX pow(2, ADC_RESOLUTION); // ADC最大值

// #define QUEUE_MESSAGE_WAIT 10

// int pitch_servo_angle, roll_servo_angle;

// esp_now_peer_info_t peerInfo;
// QueueHandle_t       PadDataQueue = NULL;

// // uint8_t RC_coreless_c3mini[]  = { 0x9c, 0x9e, 0x6e, 0x86, 0x2b, 0x48 }; // 有刷c3mini（舵机）
// uint8_t RC_autoControl_pico[] = { 0xf0, 0x24, 0xf9, 0x8f, 0xb3, 0x9c }; // PICO_1 自稳
// uint8_t RC_brushless_1_0_1[]  = { 0x48, 0xca, 0x43, 0xed, 0xc4, 0x80 }; // 无刷v1.01
// uint8_t RC_brushless_1_0_2[]  = { 0x48, 0xca, 0x43, 0xed, 0xc4, 0x58 }; // 无刷v1.02
// uint8_t RC_coreless_c3mini[]  = { 0x9c, 0x9e, 0x6e, 0x84, 0xf2, 0x1c }; // 有刷c3mini（差速）
// uint8_t airCraftAddress[6]    = {};

// // typedef enum {
// //   REMOTE_OFF,
// //   DEVICE_TEST_ON,
// //   SEND_DATA_ON
// // } SwitchState;

// // SwitchState switchstate;

// Pad        oled;
// Aircraft   aircraft;
// SendData   send_data;
// BatReading battery;

// bool esp_connected;

// /**
//  * @brief 读取电池电量方法
//  * 更新结构体电量数据，并判断是否需要报警
//  */
// void batteryReading() {
//   // 手柄电量
//   BatReading::Bat batStatus = battery.read(AVERAGE_FILTER);
//   oled.padBatteryVoltage    = batStatus.voltage;
//   oled.padPercentage        = batStatus.voltsPercentage;
//   // 接收机电量
//   oled.airCraftBatteryVoltage = aircraft.batteryValue[0];
//   oled.airCraftPercentage     = aircraft.batteryValue[1];
// }

// /**
//  * @brief 数据发送成功的回调函数
//  * 判断是否发送成功，确定显示图标和连接成功标志位
//  */
// void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
//   if (status == ESP_NOW_SEND_SUCCESS) {
//     esp_connected       = true;
//     oled.esp_now_signal = ESP_NOW_CONNECTED;
//   } else {
//     oled.esp_now_signal = ESP_NOW_DISCONNECTED;
//     esp_connected       = false;
//   }
// }

// /**
//  * @brief 接收数据后的回调函数
//  * 主要用于接收天空端的电池电量信息
//  * 同时调用电量读取和低电量报警函数
//  */
// void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
//   memcpy(&aircraft, incomingData, sizeof(aircraft));
//   batteryReading();
// }

// /**
//  * @brief 初始化ESP NOW，确定接收设备MAC地址
//  */
// void ESP_NOW_Init() {
//   WiFi.mode(WIFI_STA); // 设置wifi为STA模式
//   WiFi.begin();
//   esp_now_init();                       // 初始化ESP NOW
//   esp_now_register_send_cb(OnDataSent); // 注册发送成功的回调函数
//   esp_now_register_recv_cb(OnDataRecv); // 注册接受数据后的回调函数
//   switch (RC_num) {
//   case 1:
// #ifdef DEBUG
//     Serial.println("RC has been selected!");
// #endif
//     memcpy(airCraftAddress, RC_brushless_1_0_1, sizeof(RC_brushless_1_0_1));
//     break;
//   case 2:
// #ifdef DEBUG
//     Serial.println("RC has been selected!");
// #endif
//     memcpy(airCraftAddress, RC_brushless_1_0_2, sizeof(RC_brushless_1_0_2));
//     break;
//   default:
//     break;
//   }
//   memcpy(peerInfo.peer_addr, airCraftAddress, 6); // 设置配对设备的MAC地址并储存，参数为拷贝地址、拷贝对象、数据长度
//   peerInfo.channel = 1;                           // 设置通信频道
//   esp_now_add_peer(&peerInfo);                    // 添加通信对象
// }

// /**
//  * @brief 发送数据任务
//  * 初始化ESP NOW，发送数据
//  */
// void mainTask(void* pvParameters) {
//   ESP_NOW_Init();
//   battery.init(BATTERY_PIN, R1, R2, BATTERY_MAX_VALUE, BATTERY_MIN_VALUE);
//   TickType_t       xLastWakeTime = xTaskGetTickCount();
//   const TickType_t xPeriod       = pdMS_TO_TICKS(12.5); // 频率 80Hz → 周期为 1/80 = 0.0125 秒 = 12.5 毫秒
//   while (1) {
//     if (digitalRead(SWITCH_SEND) == 1) {
//       send_data.switch_status[0]    = digitalRead(SWITCH_SEND);
//       send_data.switch_status[1]    = digitalRead(SWITCH_AUTO);
//       send_data.switch_status[2]    = digitalRead(SWITCH_FLAP);
//       send_data.joystick_cur_val[0] = getAnalogHat(throttle);
//       send_data.joystick_cur_val[1] = getAnalogHat(diffrential);
//       send_data.joystick_cur_val[2] = getAnalogHat(aileron);
//       send_data.joystick_cur_val[3] = getAnalogHat(elevator);
//       // send_data.diffrential_coe     = 0.0;
//       oled.joystick_cur_val[0] = send_data.joystick_cur_val[0];
//       oled.joystick_cur_val[1] = send_data.joystick_cur_val[1];
//       oled.joystick_cur_val[2] = send_data.joystick_cur_val[2];
//       oled.joystick_cur_val[3] = send_data.joystick_cur_val[3];
//       oled.send_icon           = SEND_ON;
//     } else {
//       // 关闭发送按钮或关机断联
//       send_data.switch_status[0]    = 0;
//       send_data.switch_status[1]    = 0;
//       send_data.switch_status[2]    = 0;
//       send_data.joystick_cur_val[0] = -255;
//       send_data.joystick_cur_val[1] = 0;
//       send_data.joystick_cur_val[2] = 0;
//       send_data.joystick_cur_val[3] = 0;
//       // send_data.diffrential_coe     = 0.0;
//       oled.send_icon = SEND_OFF;
//     }
//     esp_now_send(airCraftAddress, (uint8_t*)&send_data, sizeof(send_data));
//     xQueueSend(PadDataQueue, &oled, QUEUE_MESSAGE_WAIT / portTICK_PERIOD_MS);
//     // 低电量报警
//     if ((esp_connected && oled.airCraftPercentage <= BATTERY_MIN_PERCENTAGE) || oled.padPercentage <= BATTERY_MIN_PERCENTAGE) {
//       buzzerStatuas buzzer;
//       buzzer = BUZZER_REPEAT;
//       xQueueSend(BuzzerEventQueue, &buzzer, QUEUE_MESSAGE_WAIT / portTICK_PERIOD_MS);
//     }
//     vTaskDelayUntil(&xLastWakeTime, xPeriod);
//   }
// }

// /**
//  * @brief 任务和队列初始化
//  */
// void sendData_init() {
//   PadDataQueue = xQueueCreate(3, sizeof(Pad));
//   xTaskCreatePinnedToCore(mainTask, "mainTask", 2048, NULL, 1, NULL, 0);
// }