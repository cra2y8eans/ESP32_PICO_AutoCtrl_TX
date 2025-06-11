#pragma once
#ifndef _BUZZER_H_
#define _BUZZER_H_

#include <Arduino.h>

typedef enum buzzerStatuas{
  BUZZER_SHORT,
  BUZZER_LONG,
  BUZZER_REPEAT
};

extern bool buzzerFlag;

void buzzer_init();
void buzzerTask(void* pvParameters);

#endif