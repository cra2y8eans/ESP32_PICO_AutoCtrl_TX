#include "common.h"
#include "sendData.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define DEBUG

#define ESP_NOW_CONNECTED 0xe870
#define ESP_NOW_DISCONNECTED 0xe791

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

/**
 * @brief 数据发送成功的回调函数
 * 判断是否发送成功，确定显示图标和连接成功标志位
 */
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  esp_connected  = status == ESP_NOW_SEND_SUCCESS ? true : false;
  esp_now_signal = status == ESP_NOW_SEND_SUCCESS ? ESP_NOW_CONNECTED : ESP_NOW_DISCONNECTED;
}

void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  memcpy(&aircraft, incomingData, sizeof(aircraft));
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
  WiFi.mode(WIFI_STA); // 设置wifi为STA模式
  WiFi.begin();
  esp_now_init();                       // 初始化ESP NOW
  esp_now_register_send_cb(OnDataSent); // 注册发送成功的回调函数
  esp_now_register_recv_cb(OnDataRecv); // 注册接受数据后的回调函数
  selectRC();
  TickType_t       xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod       = pdMS_TO_TICKS(12); // 频率 80Hz → 周期为 1/80 = 0.0125 秒 = 12.5 毫秒
  while (1) {
    esp_now_send(airCraftAddress, (uint8_t*)&sendData, sizeof(sendData));
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

/**
 * @brief 任务和队列初始化
 */
void sendDataInit() {
  xTaskCreatePinnedToCore(mainTask, "mainTask", 2048, NULL, 1, NULL, 0);
}