#include "Arduino.h"

#define SWITCH_AILERON_FULL 34
#define SWITCH_AILERON_HALF 35
#define SWITCH_ELEVATOR_FULL 36
#define SWITCH_ELEVATOR_HALF 37
#define SWITCH_RUDDER_FULL 38
#define SWITCH_RUDDER_HALF 39

#define SWITCH_TRANSMIT 25
#define SWITCH_FLAP 26
#define SWITCH_AUTO 27

enum swtichState {
  AILERON_ANGLE_FULL,           // 副翼最大角度
  ELEVATOR_ANGLE_FULL,          // 升降舵最大角度
  RUDDER_ANGLE_FULL,            // 方向舵最大角度
  AILERON_ANGLE_THREE_QUARTER,  // 副翼三分之二角度
  ELEVATOR_ANGLE_THREE_QUARTER, // 升降舵三分之二角度
  RUDDER_ANGLE_THREE_QUARTER,   // 方向舵三分之二角度
  AILERON_ANGLE_HALF,           // 副翼一半角度
  ELEVATOR_ANGLE_HALF,          // 升降舵一半角度
  RUDDER_ANGLE_HALF,            // 方向舵一半角度

  TRANSMIT_ON,  // 发送开启
  TRANSMIT_OFF, // 发送关闭
  FLAP_ON,      // 襟翼开启
  FLAP_OFF,     // 襟翼关闭
  AUTO_ON,      // 自动模式开启
  AUTO_OFF      // 自动模式关闭
};

swtichState mySwitch;

void button_AileronFull() {
  if (digitalRead(SWITCH_AILERON_FULL) == HIGH && digitalRead(SWITCH_AILERON_HALF) == LOW) {
    mySwitch = AILERON_ANGLE_FULL;
  } else if (digitalRead(SWITCH_AILERON_FULL) == LOW && digitalRead(SWITCH_AILERON_HALF) == HIGH) {
    mySwitch = AILERON_ANGLE_HALF;
  } else if (digitalRead(SWITCH_AILERON_FULL) == LOW && digitalRead(SWITCH_AILERON_HALF) == LOW) {
    mySwitch = AILERON_ANGLE_THREE_QUARTER;
  }
}