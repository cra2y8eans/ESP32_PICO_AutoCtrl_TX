#pragma once
#ifndef OLED_H
#define OLED_H

#include <Arduino.h>


extern QueueHandle_t unlockEventQueueBUZZER;


void oled_init();
void oled_task(void* pvParameters);


#endif