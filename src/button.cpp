// /**
//  * @file button.cpp
//  * @brief 使用onebutton库对按钮事件进行判断，并将事件通过队列发送到oled显示和蜂鸣器任务作为判断依据
//  */

// #include "button/button.h"
// #include "OneButton.h"
// #include "buzzer/buzzer.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include <Arduino.h>

// #define DEBUG

// #define BUTTON_PIN_L_1 13
// #define BUTTON_PIN_L_2 26
// #define BUTTON_PIN_R_1 16
// #define BUTTON_PIN_R_2 17
// #define CLICK_INTERVAL 10
// #define LONG_PRESS_INTERVAL 600
// #define BUTTON_CHECK_INTERVAL 10
// #define QUEUE_MESSAGE_WAITING 10

// QueueHandle_t ButtonEventQueue = NULL;
// QueueHandle_t BuzzerEventQueue = NULL;

// OneButton button_l_1(BUTTON_PIN_L_1, false);
// OneButton button_l_2(BUTTON_PIN_L_2, false);
// OneButton button_r_1(BUTTON_PIN_R_1, false);
// OneButton button_r_2(BUTTON_PIN_R_2, false);

// bool buzzerFlag        = true;
// bool oled_display_flag = true;

// void button_L_1_ShortPress() {
//   ButtonState btnState;
//   btnState = BUTTON_L_1_SHORT_PRESS;
//   xQueueSend(ButtonEventQueue, &btnState, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
// }

// void button_L_1_LongPress() {
//   ButtonState btnState;
//   btnState = BUTTON_L_1_LONG_PRESS;
//   xQueueSend(ButtonEventQueue, &btnState, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
// }

// void button_L_2_ShortPress() {
//   ButtonState btnState;
//   btnState = BUTTON_L_2_SHORT_PRESS;
//   xQueueSend(ButtonEventQueue, &btnState, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
// }

// void button_L_2_LongPress() {
//   ButtonState   btnState;
//   buzzerStatuas buzzer;
//   buzzer = BUZZER_LONG;
//   xQueueSend(ButtonEventQueue, &btnState, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
//   btnState = BUTTON_L_2_LONG_PRESS;
//   xQueueSend(BuzzerEventQueue, &buzzer, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
//   buzzerFlag = !buzzerFlag;
// }

// void button_R_1_ShortPress() {
//   ButtonState btnState;
//   btnState = BUTTON_R_1_SHORT_PRESS;
//   xQueueSend(ButtonEventQueue, &btnState, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
// }

// void button_R_1_LongPress() {
//   ButtonState btnState;
//   btnState = BUTTON_R_1_LONG_PRESS;
//   xQueueSend(ButtonEventQueue, &btnState, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
// }

// void button_R_2_ShortPress() {
//   ButtonState btnState;
//   btnState = BUTTON_R_2_SHORT_PRESS;
//   xQueueSend(ButtonEventQueue, &btnState, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
// }

// void button_R_2_LongPress() {
//   ButtonState   btnState;
//   buzzerStatuas buzzer;
//   buzzer = BUZZER_LONG;
//   xQueueSend(ButtonEventQueue, &btnState, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
//   btnState = BUTTON_R_2_LONG_PRESS;
//   xQueueSend(BuzzerEventQueue, &buzzer, QUEUE_MESSAGE_WAITING / portTICK_PERIOD_MS);
//   oled_display_flag = !oled_display_flag;
// }

// void button_task(void* pvParameters) {
//   button_l_1.setClickMs(CLICK_INTERVAL / portTICK_PERIOD_MS);
//   button_l_1.setPressMs(LONG_PRESS_INTERVAL / portTICK_PERIOD_MS);
//   button_l_2.setClickMs(CLICK_INTERVAL / portTICK_PERIOD_MS);
//   button_l_2.setPressMs(LONG_PRESS_INTERVAL / portTICK_PERIOD_MS);

//   button_r_1.setClickMs(CLICK_INTERVAL / portTICK_PERIOD_MS);
//   button_r_1.setPressMs(LONG_PRESS_INTERVAL / portTICK_PERIOD_MS);
//   button_r_2.setClickMs(CLICK_INTERVAL / portTICK_PERIOD_MS);
//   button_r_2.setPressMs(LONG_PRESS_INTERVAL / portTICK_PERIOD_MS);

//   button_l_1.attachClick(button_L_1_ShortPress);
//   button_l_1.attachLongPressStart(button_L_1_LongPress);
//   button_l_2.attachClick(button_L_2_ShortPress);
//   button_l_2.attachLongPressStart(button_L_2_LongPress);
//   button_r_1.attachClick(button_R_1_ShortPress);
//   button_r_1.attachLongPressStart(button_R_1_LongPress);
//   button_r_2.attachClick(button_R_2_ShortPress);
//   button_r_2.attachLongPressStart(button_R_2_LongPress);

//   while (1) {
//     button_l_1.tick();
//     button_l_2.tick();
//     button_r_1.tick();
//     button_r_2.tick();
//     vTaskDelay(BUTTON_CHECK_INTERVAL / portTICK_PERIOD_MS);
//   }
// }

// void button_init() {
//   ButtonEventQueue = xQueueCreate(3, sizeof(ButtonState));
//   BuzzerEventQueue = xQueueCreate(3, sizeof(buzzerStatuas));
//   xTaskCreatePinnedToCore(button_task, "button_task", 1024, NULL, 1, NULL, 1);
// #ifdef DEBUG
//   Serial.println(ButtonEventQueue == NULL ? "Failed to create OLED queue!" : "OLED queue created!");
//   Serial.println(button_task == NULL ? "Failed to create button task!" : "Button task created!");
// #endif
// }
