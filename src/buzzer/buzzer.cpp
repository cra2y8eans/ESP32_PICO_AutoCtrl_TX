/**
 * @file buzzer.cpp
 * @brief 接收来自button的蜂鸣器控制标志位buzzerFlag，控制蜂鸣器开关
 * 接收ESP NOW关于电池电量的信息，控制蜂鸣器开关
 * 接收oled显示器中解锁操作的信息，控制蜂鸣器开关
 */

#include "buzzer.h"
#include "button/button.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DEBUG
#define BUZZER_PIN 25
#define BUZZER_REPEAT_INTERVAL 60
#define BUZZER_SHORT_INTERVAL 200
#define BUZZER_LONG_INTERVAL 1000

bool buzzerFlag;

void buzzerTask(void* pvParameters) {
  buzzerStatuas buzzerMode;
  while (1) {
    xQueueReceive(buttonEventQueueBUZZER, &buzzerFlag, portMAX_DELAY);
    if (buzzerFlag == true) {
      switch (buzzerMode) {
      case BUZZER_SHORT:
        digitalWrite(BUZZER_PIN, HIGH);
        vTaskDelay(BUZZER_SHORT_INTERVAL / portTICK_PERIOD_MS);
        digitalWrite(BUZZER_PIN, LOW);
        break;
      case BUZZER_LONG:
        digitalWrite(BUZZER_PIN, HIGH);
        vTaskDelay(BUZZER_LONG_INTERVAL / portTICK_PERIOD_MS);
        digitalWrite(BUZZER_PIN, LOW);
        break;
      case BUZZER_REPEAT:
        digitalWrite(BUZZER_PIN, HIGH);
        vTaskDelay(BUZZER_REPEAT_INTERVAL / portTICK_PERIOD_MS);
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(BUZZER_PIN, HIGH);
        vTaskDelay(BUZZER_REPEAT_INTERVAL / portTICK_PERIOD_MS);
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(BUZZER_PIN, HIGH);
        vTaskDelay(BUZZER_REPEAT_INTERVAL / portTICK_PERIOD_MS);
        digitalWrite(BUZZER_PIN, LOW);
        break;
      default:
        break;
      }
    } else {
      digitalWrite(BUZZER_PIN, LOW);
    }
  }
}

void buzzer_init() {
  pinMode(BUZZER_PIN, OUTPUT);
  xTaskCreatePinnedToCore(buzzerTask, "buzzerTask", 1024, NULL, 1, NULL, 1);
#ifdef DEBUG
  Serial.println(buzzerTask == NULL ? "Fail to create buzzerTask" : "buzzerTask is created!");
#endif
}