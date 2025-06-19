/**
 * @file input_device.cpp
 * @brief 使用onebutton库对按钮事件进行判断，并将事件通过队列发送到数据处理任务和蜂鸣器任务作为判断依据
 * 将onebutton.cpp中setup函数中的activeLow参数false从input更改为input_pulldown，适配输入下拉
 * 将开关判断整合到input_device.cpp中，使用switchState函数判断开关状态变化
 * 将开关状态变化以数组的形式通过SwitchEventQueue发送到数据处理任务
 */

#include "input_device.h"
#include "OneButton.h"
#include "buzzer.h"
#include "common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>

#define DEBUG

#define BUTTON_PIN_L_1 13
#define BUTTON_PIN_L_2 26
#define BUTTON_PIN_R_1 16
#define BUTTON_PIN_R_2 17

#define CLICK_INTERVAL 150
#define LONG_PRESS_INTERVAL 800
#define BUTTON_CHECK_INTERVAL 20
#define QUEUE_MESSAGE_WAITING 10

#define SWITCH_SEND 25
#define SWITCH_AUTO 18
#define SWITCH_FLAP 19
#define SWITCH_INDEX sizeof(switchArr) / sizeof(switchArr[0])
uint8_t switchArr[]                    = { SWITCH_SEND, SWITCH_AUTO, SWITCH_FLAP };
uint8_t switchLastStatus[SWITCH_INDEX] = {};

QueueHandle_t ButtonToOledQueue   = NULL;
QueueHandle_t ButtonToBuzzerQueue = NULL;
QueueHandle_t SwitchEventQueue    = NULL;

OneButton button_l_1(BUTTON_PIN_L_1, false, false);
OneButton button_l_2(BUTTON_PIN_L_2, false, false);
OneButton button_r_1(BUTTON_PIN_R_1, false, false);
OneButton button_r_2(BUTTON_PIN_R_2, false, false);

bool buzzerFlag        = true;
bool oled_display_flag = true;

void switchState() {
  for (int i = 0; i < SWITCH_INDEX; i++) {
    bool currentStatus = digitalRead(switchArr[i]);
    if (currentStatus != switchLastStatus[i]) {
      switchLastStatus[i] = currentStatus; // 更新开关状态，确保只触发一次
      xQueueSend(SwitchEventQueue, switchLastStatus, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
    }
#ifdef DEBUG
    Serial.printf("Switches sent: SEND=%d AUTO=%d FLAP=%d\n",
        switchLastStatus[0],
        switchLastStatus[1],
        switchLastStatus[2]);
#endif
  }
}

void sendButtonEvent(ButtonState btnState, buzzerStatuas buzzer = BUZZER_NONE) {
  xQueueSend(ButtonToOledQueue, &btnState, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
  if (buzzer != BUZZER_NONE) {
    xQueueSend(ButtonToBuzzerQueue, &buzzer, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
  }
}

void button_L_1_ShortPress() {
  sendButtonEvent(BUTTON_L_1_SHORT_PRESS);
  // ButtonState btnState;
  // btnState = BUTTON_L_1_SHORT_PRESS;
  // xQueueSend(ButtonToOledQueue, &btnState, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
}

void button_L_1_LongPress() {
  sendButtonEvent(BUTTON_L_1_LONG_PRESS, BUZZER_LONG);
  // ButtonState btnState;
  // btnState = BUTTON_L_1_LONG_PRESS;
  // xQueueSend(ButtonToOledQueue, &btnState, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
}

void button_L_1_RepeatPress() {
  sendButtonEvent(BUTTON_L_1_REPEAT_PRESS);
  // ButtonState btnState;
  // btnState = BUTTON_L_1_REPEAT_PRESS;
  // xQueueSend(ButtonToOledQueue, &btnState, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
}

void button_L_2_ShortPress() {
  sendButtonEvent(BUTTON_L_2_SHORT_PRESS);
  // ButtonState btnState;
  // btnState = BUTTON_L_2_SHORT_PRESS;
  // xQueueSend(ButtonToOledQueue, &btnState, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
}

void button_L_2_LongPress() {
  buzzerFlag = !buzzerFlag;
  sendButtonEvent(BUTTON_L_2_LONG_PRESS, BUZZER_LONG);
  // ButtonState   btnState;
  // buzzerStatuas buzzer;
  // btnState = BUTTON_L_2_LONG_PRESS;
  // buzzer   = BUZZER_LONG;
  // xQueueSend(ButtonToOledQueue, &btnState, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
  // xQueueSend(ButtonToBuzzerQueue, &buzzer, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
}

void button_L_2_RepeatPress() {
  sendButtonEvent(BUTTON_L_2_REPEAT_PRESS);
  // ButtonState btnState;
  // btnState = BUTTON_L_2_REPEAT_PRESS;
  // xQueueSend(ButtonToOledQueue, &btnState, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
}

void button_R_1_ShortPress() {
  sendButtonEvent(BUTTON_R_1_SHORT_PRESS);
  // ButtonState btnState;
  // btnState = BUTTON_R_1_SHORT_PRESS;
  // xQueueSend(ButtonToOledQueue, &btnState, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
}

void button_R_1_LongPress() {
  sendButtonEvent(BUTTON_R_1_LONG_PRESS);
  // ButtonState btnState;
  // btnState = BUTTON_R_1_LONG_PRESS;
  // xQueueSend(ButtonToOledQueue, &btnState, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
}

void button_R_1_RepeatPress() {
  sendButtonEvent(BUTTON_R_1_REPEAT_PRESS);
  // ButtonState btnState;
  // btnState = BUTTON_R_1_REPEAT_PRESS;
  // xQueueSend(ButtonToOledQueue, &btnState, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
}

void button_R_2_ShortPress() {
  sendButtonEvent(BUTTON_R_2_SHORT_PRESS);
  // ButtonState btnState;
  // btnState = BUTTON_R_2_SHORT_PRESS;
  // xQueueSend(ButtonToOledQueue, &btnState, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
}

void button_R_2_LongPress() {
  oled_display_flag = !oled_display_flag;
  sendButtonEvent(BUTTON_R_2_LONG_PRESS, BUZZER_LONG);
  // ButtonState   btnState;
  // buzzerStatuas buzzer;
  // btnState = BUTTON_R_2_LONG_PRESS;
  // buzzer   = BUZZER_LONG;
  // xQueueSend(ButtonToOledQueue, &btnState, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
  // xQueueSend(ButtonToBuzzerQueue, &buzzer, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
}

void button_R_2_RepeatPress() {
  sendButtonEvent(BUTTON_R_2_REPEAT_PRESS);
  // ButtonState btnState;
  // btnState = BUTTON_R_2_REPEAT_PRESS;
  // xQueueSend(ButtonToOledQueue, &btnState, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
}

void input_device_task(void* pvParameters) {
  for (int i = 0; i < SWITCH_INDEX; i++) {
    pinMode(switchArr[i], INPUT_PULLDOWN);
    switchLastStatus[i] = digitalRead(switchArr[i]); // 初始化开关状态
  }

  button_l_1.setClickMs(CLICK_INTERVAL / portTICK_PERIOD_MS);
  button_l_1.setPressMs(LONG_PRESS_INTERVAL / portTICK_PERIOD_MS);
  button_l_2.setClickMs(CLICK_INTERVAL / portTICK_PERIOD_MS);
  button_l_2.setPressMs(LONG_PRESS_INTERVAL / portTICK_PERIOD_MS);

  button_r_1.setClickMs(CLICK_INTERVAL / portTICK_PERIOD_MS);
  button_r_1.setPressMs(LONG_PRESS_INTERVAL / portTICK_PERIOD_MS);
  button_r_2.setClickMs(CLICK_INTERVAL / portTICK_PERIOD_MS);
  button_r_2.setPressMs(LONG_PRESS_INTERVAL / portTICK_PERIOD_MS);

  button_l_1.attachClick(button_L_1_ShortPress);
  button_l_1.attachLongPressStart(button_L_1_LongPress);
  button_l_1.attachDoubleClick(button_L_1_RepeatPress);

  button_l_2.attachClick(button_L_2_ShortPress);
  button_l_2.attachLongPressStart(button_L_2_LongPress);
  button_l_2.attachDoubleClick(button_L_2_RepeatPress);

  button_r_1.attachClick(button_R_1_ShortPress);
  button_r_1.attachLongPressStart(button_R_1_LongPress);
  button_r_1.attachDoubleClick(button_R_1_RepeatPress);

  button_r_2.attachClick(button_R_2_ShortPress);
  button_r_2.attachLongPressStart(button_R_2_LongPress);
  button_r_2.attachDoubleClick(button_R_2_RepeatPress);

  while (1) {
    button_l_1.tick();
    button_l_2.tick();
    button_r_1.tick();
    button_r_2.tick();
    switchState();
    vTaskDelay(BUTTON_CHECK_INTERVAL / portTICK_PERIOD_MS);
  }
}

void input_device_init() {
  ButtonToOledQueue   = xQueueCreate(3, sizeof(ButtonState));
  ButtonToBuzzerQueue = xQueueCreate(3, sizeof(buzzerStatuas));
  SwitchEventQueue    = xQueueCreate(3, sizeof(switchLastStatus));
  xTaskCreatePinnedToCore(input_device_task, "input_device_task", 1024, NULL, 1, NULL, 1);
#ifdef DEBUG
  Serial.println(ButtonToOledQueue == NULL ? "Failed to create OLED queue!" : "OLED queue created!");
  Serial.println(input_device_task == NULL ? "Failed to create button task!" : "Button task created!");
#endif
}
