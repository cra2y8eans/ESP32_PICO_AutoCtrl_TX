#include "common.h"
#include <Arduino.h>

#define BUZZER_PIN 23
#define BUZZER_REPEAT_INTERVAL 60
#define BUZZER_SHORT_INTERVAL 200
#define BUZZER_LONG_INTERVAL 1000
#define QUEUE_MESSAGE_WAITING 20

uint8_t buzzerMode = 0; // 遥控器数量

void buzzer(uint8_t mode) {
  if (buzzerFlag == true) {
    switch (mode) {
    case 1:
      digitalWrite(BUZZER_PIN, HIGH);
      vTaskDelay(BUZZER_SHORT_INTERVAL / portTICK_PERIOD_MS);
      digitalWrite(BUZZER_PIN, LOW);
      break;
    case 2:
      digitalWrite(BUZZER_PIN, HIGH);
      vTaskDelay(BUZZER_LONG_INTERVAL / portTICK_PERIOD_MS);
      digitalWrite(BUZZER_PIN, LOW);
      break;
    case 3:
      digitalWrite(BUZZER_PIN, HIGH);
      vTaskDelay(BUZZER_REPEAT_INTERVAL / portTICK_PERIOD_MS);
      digitalWrite(BUZZER_PIN, LOW);
      vTaskDelay(BUZZER_REPEAT_INTERVAL / portTICK_PERIOD_MS);
      digitalWrite(BUZZER_PIN, HIGH);
      vTaskDelay(BUZZER_REPEAT_INTERVAL / portTICK_PERIOD_MS);
      digitalWrite(BUZZER_PIN, LOW);
      vTaskDelay(BUZZER_REPEAT_INTERVAL / portTICK_PERIOD_MS);
      digitalWrite(BUZZER_PIN, HIGH);
      vTaskDelay(BUZZER_REPEAT_INTERVAL / portTICK_PERIOD_MS);
      digitalWrite(BUZZER_PIN, LOW);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      break;
    default:
      break;
    }
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
}
