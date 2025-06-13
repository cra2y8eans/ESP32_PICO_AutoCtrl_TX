/**
 * @file button.cpp
 * @brief 使用onebutton库对按钮事件进行判断，并将事件通过队列发送到oled显示和蜂鸣器任务作为判断依据
 */

#include "button.h"
#include "OneButton.h"
#include "buzzer/buzzer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>

// #define DEBUG

#define BUTTON_L 0
#define BUTTON_R 16
#define CLICK_INTERVAL 10
#define LONG_PRESS_INTERVAL 600
#define BUTTON_CHECK_INTERVAL 10
#define QUEUE_MESSAGE_WAITING 20

QueueHandle_t ButtonEventQueue = NULL;

OneButton buttonL(BUTTON_L, true); // 只支持内部上拉电阻
OneButton buttonR(BUTTON_R, true);

bool buzzerFlag        = true;
bool oled_display_flag = true;

void button_L_ShortPress() {
  ButtonState btnState;
  btnState = BUTTON_L_SHORT_PRESS;
  xQueueSend(ButtonEventQueue, &btnState, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
}

void button_L_LongPress() {
  ButtonState   btnState;
  buzzerStatuas buzzer;
  btnState   = BUTTON_L_LONG_PRESS;
  buzzerFlag = !buzzerFlag;
  xQueueSend(ButtonEventQueue, &btnState, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
  xQueueSend(BuzzerEventQueue, &buzzer, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
}

void button_L_RepeatPress() {
  ButtonState btnState;
  btnState = BUTTON_L_REPEAT_PRESS;
  xQueueSend(ButtonEventQueue, &btnState, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
}

void button_R_ShortPress() {
  ButtonState btnState;
  btnState = BUTTON_R_SHORT_PRESS;
  xQueueSend(ButtonEventQueue, &btnState, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
}

void button_R_LongPress() {
  ButtonState   btnState;
  buzzerStatuas buzzer;
  btnState          = BUTTON_R_LONG_PRESS;
  oled_display_flag = !oled_display_flag;
  xQueueSend(ButtonEventQueue, &btnState, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
  xQueueSend(BuzzerEventQueue, &buzzer, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
}

void button_R_RepeatPress() {
  ButtonState btnState;
  btnState = BUTTON_R_REPEAT_PRESS;
  xQueueSend(ButtonEventQueue, &btnState, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
}

void button_task(void* pvParameters) {
  buttonL.setClickMs(CLICK_INTERVAL / portTICK_PERIOD_MS);
  buttonR.setClickMs(CLICK_INTERVAL / portTICK_PERIOD_MS);
  buttonL.setPressMs(LONG_PRESS_INTERVAL / portTICK_PERIOD_MS);
  buttonR.setPressMs(LONG_PRESS_INTERVAL / portTICK_PERIOD_MS);

  buttonL.attachClick(button_L_ShortPress);
  buttonL.attachLongPressStart(button_L_LongPress);
  buttonL.attachDoubleClick(button_L_RepeatPress);

  buttonR.attachClick(button_R_ShortPress);
  buttonR.attachLongPressStart(button_R_LongPress);
  buttonR.attachDoubleClick(button_R_RepeatPress);

  while (1) {
    buttonL.tick();
    buttonR.tick();
    vTaskDelay(BUTTON_CHECK_INTERVAL / portTICK_PERIOD_MS);
  }
}

void button_init() {
  ButtonEventQueue = xQueueCreate(3, sizeof(ButtonState));
  xTaskCreatePinnedToCore(button_task, "button_task", 1024, NULL, 1, NULL, 1);
#ifdef DEBUG
  Serial.println(ButtonEventQueue == NULL ? "Failed to create OLED queue!" : "OLED queue created!");
  Serial.println(button_task == NULL ? "Failed to create button task!" : "Button task created!");
#endif
}
