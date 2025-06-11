#include "sendData.h"

#define SERVO_MAX_ANGLE 120 // 舵机最大角度

QueueHandle_t padDataQueueOLED = NULL;

void sendData_init() {
  padDataQueueOLED = xQueueCreate(3, sizeof(Pad));
}