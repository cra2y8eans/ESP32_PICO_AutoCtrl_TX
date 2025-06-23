// #include "battery.h"
// #include "batteryReading.hpp"
// #include "buzzer.h"
// #include "common.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include <Arduino.h>

// #define BATTERY_PIN 36                    // 电池电量读取引脚
// #define BATTERY_MAX_VALUE 4.2             // 电池最大电量
// #define BATTERY_MIN_VALUE 3.2             // 电池最小电量
// #define BATTERY_MIN_PERCENTAGE 20         // 电池最低百分比
// #define PAD_BATTERY_READING_INTERVAL 3000 // 采样间隔
// #define R1 10000
// #define R2 9950
// #define AVERAGE_FILTER 50 // 滤波平均次数

// BatReading battery;
// Battery_t  batteryStatus;
// sendData_t sendData;

// QueueHandle_t BatteryToBuzzerQueue = NULL; // 电池到蜂鸣器的消息队列

// void batteryReadingTask(void* pvParameters) {
//   static unsigned long lastAlarmStart = 0;     // 上次报警开始时间
//   static bool          isAlerted      = false; // 是否已经报警过（进入静默期）
//   battery.init(BATTERY_PIN, R1, R2, BATTERY_MAX_VALUE, BATTERY_MIN_VALUE);
//   while (1) {
//     BatReading::Bat batStatus = battery.read(AVERAGE_FILTER);
//     batteryStatus.pad[0]      = batStatus.voltage;
//     batteryStatus.pad[1]      = batStatus.voltsPercentage;
//     if (batteryStatus.pad[1] < BATTERY_MIN_PERCENTAGE || aircraft.batteryValue[1] < BATTERY_MIN_PERCENTAGE) {
//       if (isAlerted == false) {
//         // 如果未报警过，开始报警
//         for (int i = 0; i < 3; i++) {
//           // buzzerStatuas buzzer = BUZZER_REPEAT;
//           // xQueueSend(BatteryToBuzzerQueue, &buzzer, 10);
//           buzzer(3);                             // 蜂鸣器报警，repeat模式
//           vTaskDelay(1000 / portTICK_PERIOD_MS); // 每秒报警一次
//         }
//         isAlerted = true; // 设置为已报警状态
//       } else {
//         // 如果已经报警过，进入静默期
//         unsigned long currentTime = millis();
//         if (currentTime - lastAlarmStart > 15000) {
//           // 如果静默期结束，重置状态
//           isAlerted      = false;
//           lastAlarmStart = currentTime; // 重置计时器
//         }
//       }
//     }
//     vTaskDelay(3000 / portTICK_PERIOD_MS);
//   }
// }

// void battery_init() {
//   // BatteryToBuzzerQueue = xQueueCreate(3, sizeof(buzzerStatuas));
//   xTaskCreatePinnedToCore(batteryReadingTask, "batteryReading", 1024 * 2, NULL, 1, NULL, 1);
// }