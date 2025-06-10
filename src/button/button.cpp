#include "button.h"
#include "Arduino.h"
#include "OneButton.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BUTTON_L 13
#define BUTTON_R 16
#define BUTTON_CHECK_INTERVAL 10

QueueHandle_t buttonEventQueueOLED = NULL;
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

void button_init() {
  buttonEventQueueOLED   = xQueueCreate(3, sizeof(ButtonState));
  buttonEventQueueBUZZER = xQueueCreate(3, sizeof(bool));
  ESP_ERROR_CHECK(buttonEventQueueOLED == NULL ? ESP_FAIL : ESP_OK);
  xTaskCreatePinnedToCore(button_task, "button_task", 1024, NULL, 1, NULL, 1);
}

void button_task(void* pvParameters) {

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
