#ifndef MALT
#define MALT
#include <Arduino.h>

typedef enum
{
    throttle,  // 左摇杆水平轴
    diffrential,  // 左摇杆垂直轴
    joyLPress, // 左摇杆按下

    elevator,  // 右摇杆水平轴
    aileron,  // 右摇杆垂直轴
    joyRPress, // 右摇杆按下

} joyAxis_t; // 摇杆轴向

void setupAnalogHat();

int16_t getAnalogHat(joyAxis_t axis);

#endif
