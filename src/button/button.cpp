#include "button.h"
#include "OneButton.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>

#define DEBUG

#define BUTTON_L 0
#define BUTTON_R 16
#define CLICK_INTERVAL 10
#define LONG_PRESS_INTERVAL 600
#define BUTTON_CHECK_INTERVAL 10

QueueHandle_t buttonEventQueueOLED   = NULL;
QueueHandle_t buttonEventQueueBUZZER = NULL;

OneButton buttonL(BUTTON_L, true); // 只支持内部上拉电阻
OneButton buttonR(BUTTON_R, true);

bool buzzerFlag = false;

void button_L_ShortPress() {
  ButtonState btnState;
  btnState = BUTTON_L_SHORT_PRESS;
  xQueueSend(buttonEventQueueOLED, &btnState, portMAX_DELAY);
}

void button_L_LongPress() {
  ButtonState btnState;
  btnState   = BUTTON_L_LONG_PRESS;
  buzzerFlag = !buzzerFlag;
  xQueueSend(buttonEventQueueOLED, &btnState, portMAX_DELAY);
  xQueueSend(buttonEventQueueBUZZER, &buzzerFlag, portMAX_DELAY);
}

void button_L_RepeatPress() {
  ButtonState btnState;
  btnState = BUTTON_L_REPEAT_PRESS;
  xQueueSend(buttonEventQueueOLED, &btnState, portMAX_DELAY);
}

void button_R_ShortPress() {
  ButtonState btnState;
  btnState = BUTTON_R_SHORT_PRESS;
  xQueueSend(buttonEventQueueOLED, &btnState, portMAX_DELAY);
}

void button_R_LongPress() {
  ButtonState btnState;
  btnState = BUTTON_R_LONG_PRESS;
  xQueueSend(buttonEventQueueOLED, &btnState, portMAX_DELAY);
}

void button_R_RepeatPress() {
  ButtonState btnState;
  btnState = BUTTON_R_REPEAT_PRESS;
  xQueueSend(buttonEventQueueOLED, &btnState, portMAX_DELAY);
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
  buttonEventQueueOLED   = xQueueCreate(3, sizeof(ButtonState));
  buttonEventQueueBUZZER = xQueueCreate(3, sizeof(bool));
  xTaskCreatePinnedToCore(button_task, "button_task", 1024, NULL, 1, NULL, 1);
#ifdef DEBUG
  Serial.println(buttonEventQueueOLED == NULL ? "Failed to create OLED queue!" : "OLED queue created!");
  Serial.println(buttonEventQueueBUZZER == NULL ? "Failed to create BUZZER queue!" : "BUZZER queue created!");
  Serial.println(button_task == NULL ? "Failed to create button task!" : "Button task created!");
#endif
}
