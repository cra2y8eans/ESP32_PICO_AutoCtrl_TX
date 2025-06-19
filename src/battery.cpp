#include "battery.h"
#include "batteryReading.hpp"
#include "buzzer.h"
#include "common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>

#define BATTERY_PIN 36                    // 电池电量读取引脚
#define BATTERY_MAX_VALUE 4.2             // 电池最大电量
#define BATTERY_MIN_VALUE 3.2             // 电池最小电量
#define BATTERY_MIN_PERCENTAGE 20         // 电池最低百分比
#define PAD_BATTERY_READING_INTERVAL 3000 // 采样间隔
#define R1 10000
#define R2 9950
#define AVERAGE_FILTER 50         // 滤波平均次数

BatReading battery;
Battery_t  batteryStatus;
sendData_t sendData;

QueueHandle_t BatteryToBuzzerQueue = NULL; // 电池到蜂鸣器的消息队列

void batteryReadingTask(void* pvParameters) {
  TickType_t       xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod       = pdMS_TO_TICKS(PAD_BATTERY_READING_INTERVAL);
  while (1) {
    BatReading::Bat batStatus = battery.read(AVERAGE_FILTER);
    batteryStatus.pad[0]      = batStatus.voltage;
    batteryStatus.pad[1]      = batStatus.voltsPercentage;
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

void lowBatteryAlarmTask(void* pvParameters) {
  static unsigned long lastAlarmStart = 0;     // 上次报警开始时间
  static bool          isAlerted      = false; // 是否已经报警过（进入静默期）
  while (1) {
    if (batteryStatus.pad[1] < BATTERY_MIN_PERCENTAGE || batteryStatus.aircraft[1] < BATTERY_MIN_PERCENTAGE) {
      unsigned long currentTime = millis();
      if (!isAlerted) {
        // 未报警过，可以开始报警
        if (currentTime - lastAlarmStart <= 5000) {
          // 在5秒报警期内
          buzzerStatuas buzzer = BUZZER_REPEAT;
          xQueueSend(BatteryToBuzzerQueue, &buzzer, 10);
          // 发送一次报警后延迟1秒（每秒报警一次）
          vTaskDelay(1000 / portTICK_PERIOD_MS);
        } else {
          // 5秒报警结束，进入15秒静默期
          isAlerted      = true;
          lastAlarmStart = currentTime; // 重置计时器
        }
      } else {
        // 已经报警过，处于静默期
        if (currentTime - lastAlarmStart > 15000) {
          // 15秒静默结束，重置状态
          isAlerted      = false;
          lastAlarmStart = currentTime;
        }
      }
    } else {
      // 电池电量恢复正常
      isAlerted      = false; // 重置报警状态
      lastAlarmStart = 0;     // 重置计时器
    }
    // 基础循环延迟
    vTaskDelay(1500 / portTICK_PERIOD_MS);
  }
}

void battery_init() {
  BatteryToBuzzerQueue = xQueueCreate(3, sizeof(buzzerStatuas));
  battery.init(BATTERY_PIN, R1, R2, BATTERY_MAX_VALUE, BATTERY_MIN_VALUE);
  xTaskCreatePinnedToCore(batteryReadingTask, "batteryReading", 1024, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(lowBatteryAlarmTask, "lowBatteryAlarm", 1024, NULL, 1, NULL, 1);
}