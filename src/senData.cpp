#include "batteryReading.hpp"
#include "common.h"
#include "input_device.h"
#include "my_analog_hat.h"
#include "sendData.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// #define DEBUG

#define ESP_NOW_CONNECTED 0xe870
#define ESP_NOW_DISCONNECTED 0xe791
#define BATTERY_PIN 36                    // 电池电量读取引脚
#define BATTERY_MAX_VALUE 4.2             // 电池最大电量
#define BATTERY_MIN_VALUE 3.2             // 电池最小电量
#define BATTERY_MIN_PERCENTAGE 20         // 电池最低百分比
#define PAD_BATTERY_READING_INTERVAL 3000 // 采样间隔
#define R1 10000
#define R2 9950
#define AVERAGE_FILTER 50 // 滤波平均次数

// uint8_t RC_coreless_c3mini[]  = { 0x9c, 0x9e, 0x6e, 0x86, 0x2b, 0x48 }; // 有刷c3mini（舵机）
uint8_t RC_autoControl_pico[] = { 0xf0, 0x24, 0xf9, 0x8f, 0xb3, 0x9c }; // PICO_1 自稳
uint8_t RC_brushless_1_0_1[]  = { 0x48, 0xca, 0x43, 0xed, 0xc4, 0x80 }; // 无刷v1.01
uint8_t RC_brushless_1_0_2[]  = { 0x48, 0xca, 0x43, 0xed, 0xc4, 0x58 }; // 无刷v1.02
uint8_t RC_coreless_c3mini[]  = { 0x9c, 0x9e, 0x6e, 0x84, 0xf2, 0x1c }; // 有刷c3mini（差速）
uint8_t airCraftAddress[6]    = {};

esp_now_peer_info_t peerInfo;

Aircraft aircraft; // 飞机数据结构体

bool esp_connected  = false; // ESP NOW连接状态标志位
int  esp_now_signal = 0;     // ESP NOW信号标志位

BatReading battery;
Battery_t  batteryStatus;
sendData_t sendData; // 发送数据结构体

/**
 * @brief 数据发送成功的回调函数
 * 判断是否发送成功，确定显示图标和连接成功标志位
 */
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  esp_connected  = status == ESP_NOW_SEND_SUCCESS ? true : false;
  esp_now_signal = status == ESP_NOW_SEND_SUCCESS ? ESP_NOW_CONNECTED : ESP_NOW_DISCONNECTED;
}

void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  memcpy(&aircraft, incomingData, sizeof(aircraft)); // 将接收到的数据拷贝到飞机数据结构体中
  static unsigned long lastAlarmStart = 0;           // 上次报警开始时间
  static bool          isAlerted      = false;       // 是否已经报警过（进入静默期）
  BatReading::Bat      batStatus      = battery.read(AVERAGE_FILTER);
  batteryStatus.pad[0]                = batStatus.voltage;
  batteryStatus.pad[1]                = batStatus.voltsPercentage;
  if (batteryStatus.pad[1] < BATTERY_MIN_PERCENTAGE || aircraft.batteryValue[1] < BATTERY_MIN_PERCENTAGE) {
    if (isAlerted == false) {
      // 如果未报警过，开始报警
      for (int i = 0; i < 3; i++) {
        // buzzerStatuas buzzer = BUZZER_REPEAT;
        // xQueueSend(BatteryToBuzzerQueue, &buzzer, 10);
        buzzer(3);                             // 蜂鸣器报警，repeat模式
        vTaskDelay(1000 / portTICK_PERIOD_MS); // 每秒报警一次
      }
      isAlerted = true; // 设置为已报警状态
    } else {
      // 如果已经报警过，进入静默期
      unsigned long currentTime = millis();
      if (currentTime - lastAlarmStart > 15000) {
        // 如果静默期结束，重置状态
        isAlerted      = false;
        lastAlarmStart = currentTime; // 重置计时器
      }
    }
  }
}

void selectRC() {
  switch (RC_num) {
  case 1:
    memcpy(airCraftAddress, RC_brushless_1_0_1, sizeof(RC_brushless_1_0_1));
    break;
  case 2:
    memcpy(airCraftAddress, RC_brushless_1_0_2, sizeof(RC_brushless_1_0_2));
    break;
  default:
    break;
  }
  memcpy(peerInfo.peer_addr, airCraftAddress, 6); // 设置配对设备的MAC地址并储存，参数为拷贝地址、拷贝对象、数据长度
  peerInfo.channel = 1;                           // 设置通信频道
  esp_now_add_peer(&peerInfo);                    // 添加通信对象
}

/**
 * @brief 发送数据任务
 * 初始化ESP NOW，发送数据
 */
void mainTask(void* pvParameters) {
  WiFi.mode(WIFI_STA);                  // 设置wifi为STA模式
  esp_now_init();                       // 初始化ESP NOW
  esp_now_register_send_cb(OnDataSent); // 注册发送成功的回调函数
  esp_now_register_recv_cb(OnDataRecv); // 注册接受数据后的回调函数
  selectRC();
  battery.init(BATTERY_PIN, R1, R2, BATTERY_MAX_VALUE, BATTERY_MIN_VALUE);
  static uint8_t switchLastStatus[3] = { 0 }; // 存储开关状态

#ifdef DEBUG
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
  } else {
    Serial.println("ESP-NOW init success");
  }
#endif

  TickType_t       xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod       = pdMS_TO_TICKS(12); // 频率 80Hz → 周期为 1/80 = 0.0125 秒 = 12.5 毫秒
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
    esp_now_send(airCraftAddress, (uint8_t*)&sendData, sizeof(sendData));
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
/**
 * @brief 任务和队列初始化
 */
void sendDataInit() {
  xTaskCreatePinnedToCore(mainTask, "mainTask", 1024 * 6, NULL, 1, NULL, 1);
}