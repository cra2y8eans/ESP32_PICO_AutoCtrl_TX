#pragma once

#include <Arduino.h>

extern uint8_t       RC_num;

void oled_init();
void oled_task(void* pvParameters);
uint8_t get_MAC_address();
