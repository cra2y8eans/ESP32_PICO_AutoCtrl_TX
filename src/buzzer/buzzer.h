#pragma once

#include <Arduino.h>

enum buzzerStatuas {
  BUZZER_SHORT,
  BUZZER_LONG,
  BUZZER_REPEAT
};

extern QueueHandle_t BuzzerEventQueue;
extern bool buzzerFlag;

void buzzer_init();
void buzzerTask(void* pvParameters);
