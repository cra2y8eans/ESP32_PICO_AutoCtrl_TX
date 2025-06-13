#include "batteryReading.hpp"
#include "buzzer/buzzer.h"
#include "filter/my_analog_hat.h"
#include "oled/oled.h"
#include "sendData.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define ESP_NOW_CONNECTED 0xe870
#define ESP_NOW_DISCONNECTED 0xe791
#define SEND_ON 0xE898
#define SEND_OFF 0xf140

#define BATTERY_PIN 36                    // 电池电量读取引脚
#define BATTERY_MAX_VALUE 4.2             // 电池最大电量
#define BATTERY_MIN_VALUE 3.2             // 电池最小电量
#define BATTERY_MIN_PERCENTAGE 20         // 电池最低百分比
#define PAD_BATTERY_READING_INTERVAL 3000 // 采样间隔
#define R1 10000
#define R2 9950
#define AVERAGE_FILTER 50         // 滤波平均次数
#define BATTERY_MIN_PERCENTAGE 20 // 低电量报警阈值

#define SWITCH_SEND_UP 25       // 发送数据开
#define SWITCH_SEND_DOWN -1     // 设备测试
#define SWITCH_YAW_UP -2        // 偏航最小
#define SWITCH_YAW_DOWN -3      // 偏航最大
#define SWITCH_AILERON_UP -5    // 副翼最小
#define SWITCH_AILERON_DOWN -6  // 副翼最大
#define SWITCH_ELEVATOR_UP -8   // 升降舵最小
#define SWITCH_ELEVATOR_DOWN -9 // 升降舵最大
#define SWITCH_AUTO 17          // 自稳开关
#define SWITCH_FLAP 19          // 襟翼开关

#define HERTZ 50              // 频率
#define SERVO_AILERON_L 2     // 左副翼引脚
#define SERVO_AILERON_R 8     // 右副翼引脚
#define SERVO_ELEVATOR 3      // 升降舵引脚
#define SERVO_FREQ_MIN 500    // 舵机最小频率
#define SERVO_FREQ_MAX 2500   // 舵机最大频率
#define SERVO_ANGLE_RANGE 120 // 舵机最大角度
#define MOTOR_PIN_L 5         // 左电机引脚
#define MOTOR_PIN_R 6         // 右电机引脚
#define MOTOR_PWM_MIN 205     // 电机频率
#define MOTOR_PWM_MAX 410
#define MOTOR_CHANNEL_L 4
#define MOTOR_CHANNEL_R 5
#define MOTOR_FREQUENCY 50
#define MOTOR_RESOLUTION 12
#define JOYSTICK_ADC_OUT_MAX 255  // 遥控器摇杆输出ADC最大值
#define JOYSTICK_ADC_OUT_MIN -255 // 遥控器摇杆输出ADC最小值
#define ADC_MIN 0                 // ADC最小值

#define YAW_ADJ_PIN -20
#define AILERON_ADJ_PIN -21
#define ELEVATOR_ADJ_PIN -22

#define QUEUE_MESSAGE_WAIT 20

int pitch_servo_angle, roll_servo_angle;

esp_now_peer_info_t peerInfo;
QueueHandle_t       PadDataQueue = NULL;

// uint8_t RC_coreless_c3mini[]  = { 0x9c, 0x9e, 0x6e, 0x86, 0x2b, 0x48 }; // 有刷c3mini（舵机）
uint8_t RC_autoControl_pico[] = { 0xf0, 0x24, 0xf9, 0x8f, 0xb3, 0x9c }; // PICO_1 自稳
uint8_t RC_brushless_1_0_1[]  = { 0x48, 0xca, 0x43, 0xed, 0xc4, 0x80 }; // 无刷v1.01
uint8_t RC_brushless_1_0_2[]  = { 0x48, 0xca, 0x43, 0xed, 0xc4, 0x58 }; // 无刷v1.02
uint8_t RC_coreless_c3mini[]  = { 0x9c, 0x9e, 0x6e, 0x84, 0xf2, 0x1c }; // 有刷c3mini（差速）
uint8_t airCraftAddress[6]    = {};

typedef enum {
  REMOTE_OFF,
  DEVICE_TEST_ON,
  SEND_DATA_ON
} SwitchState;

SwitchState switchstate;

Pad        pad;
Aircraft   aircraft;
SendData   send_data;
BatReading battery;

bool esp_connected;

/**
 * @brief 读取电池电量方法
 * 更新结构体电量数据，并判断是否需要报警
 */
void batteryReading() {
  // 手柄电量
  BatReading::Bat batStatus = battery.read(AVERAGE_FILTER);
  pad.padBatteryVoltage     = batStatus.voltage;
  pad.padPercentage         = batStatus.voltsPercentage;
  // 接收机电量
  pad.airCraftBatteryVoltage = aircraft.batteryValue[0];
  pad.airCraftPercentage     = aircraft.batteryValue[1];
  // 低电量报警
  if (esp_connected && (pad.airCraftPercentage <= BATTERY_MIN_PERCENTAGE || pad.padPercentage <= BATTERY_MIN_PERCENTAGE)) {
    buzzerStatuas buzzer;
    buzzer = BUZZER_REPEAT;
    xQueueSend(BuzzerEventQueue, &buzzer, QUEUE_MESSAGE_WAIT / portTICK_PERIOD_MS);
  }
}

/**
 * @brief 数据发送成功的回调函数
 * 判断是否发送成功，确定显示图标和连接成功标志位
 */
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    esp_connected      = true;
    pad.esp_now_signal = ESP_NOW_CONNECTED;
  } else {
    pad.esp_now_signal = ESP_NOW_DISCONNECTED;
    esp_connected      = false;
  }
}

/**
 * @brief 接收数据后的回调函数
 * 主要用于接收天空端的电池电量信息
 * 同时调用电量读取和低电量报警函数
 */
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  memcpy(&aircraft, incomingData, sizeof(aircraft));
  batteryReading();
}

/**
 * @brief 初始化ESP NOW，确定接收设备MAC地址
 */
void ESP_NOW_Init() {
  WiFi.mode(WIFI_STA); // 设置wifi为STA模式
  WiFi.begin();
  esp_now_init();                       // 初始化ESP NOW
  esp_now_register_send_cb(OnDataSent); // 注册发送成功的回调函数
  esp_now_register_recv_cb(OnDataRecv); // 注册接受数据后的回调函数
  switch (RC_num) {
  case 1:
    memcpy(airCraftAddress, RC_brushless_1_0_1, sizeof(RC_brushless_1_0_1));
    break;
  case 2:
    memcpy(airCraftAddress, RC_brushless_1_0_2, sizeof(RC_brushless_1_0_2));
    break;
  default:
    break;
  }
  memcpy(peerInfo.peer_addr, airCraftAddress, 6); // 设置配对设备的MAC地址并储存，参数为拷贝地址、拷贝对象、数据长度
  peerInfo.channel = 1;                           // 设置通信频道
  esp_now_add_peer(&peerInfo);                    // 添加通信对象
}

/**
 * @brief 读取数据任务
 *
  int   button_status[5]    = {}; // 0、方向开关    1、副翼开关     2、升降舵开关   3、差速（自稳）开关      4、襟翼开关
  int   joystick_cur_val[4] = {}; // 0、油门        1、差速         2、副翼         3、升降舵
  int   joystick_adj_val[3] = {}; // 0、差速        1、副翼         2、升降舵
  int   send_icon, esp_now_signal;
  float diffrential_coe,
      airCraftPercentage,
      airCraftBatteryVoltage,
      padPercentage,
      padBatteryVoltage;
 */
void switchStateIdentify() {
  if (digitalRead(SWITCH_SEND_UP) == 0 && digitalRead(SWITCH_SEND_DOWN) == 1) {
    switchstate = SEND_DATA_ON;
  } else if (digitalRead(SWITCH_SEND_UP) == 0 && digitalRead(SWITCH_SEND_DOWN) == 0) {
    switchstate = DEVICE_TEST_ON;
  } else if (digitalRead(SWITCH_SEND_UP) == 1 && digitalRead(SWITCH_SEND_DOWN) == 0) {
    switchstate = REMOTE_OFF;
  }

  if (digitalRead(SWITCH_YAW_UP) == 1 && digitalRead(SWITCH_YAW_DOWN) == 0) {
    send_data.switch_status[0] = 2;
  } else if (digitalRead(SWITCH_YAW_UP) == 0 && digitalRead(SWITCH_YAW_DOWN) == 0) {
    send_data.switch_status[0] = 1;
  } else if (digitalRead(SWITCH_YAW_UP) == 0 && digitalRead(SWITCH_YAW_DOWN) == 1) {
    send_data.switch_status[0] = 0;
  }

  if (digitalRead(SWITCH_AILERON_UP) == 1 && digitalRead(SWITCH_AILERON_DOWN) == 0) {
    send_data.switch_status[1] = 2;
  } else if (digitalRead(SWITCH_AILERON_UP) == 0 && digitalRead(SWITCH_AILERON_DOWN) == 0) {
    send_data.switch_status[1] = 1;
  } else if (digitalRead(SWITCH_AILERON_UP) == 0 && digitalRead(SWITCH_AILERON_DOWN) == 1) {
    send_data.switch_status[1] = 0;
  }

  if (digitalRead(SWITCH_ELEVATOR_UP) == 1 && digitalRead(SWITCH_ELEVATOR_DOWN) == 0) {
    send_data.switch_status[2] = 2;
  } else if (digitalRead(SWITCH_ELEVATOR_UP) == 0 && digitalRead(SWITCH_ELEVATOR_DOWN) == 0) {
    send_data.switch_status[2] = 1;
  } else if (digitalRead(SWITCH_ELEVATOR_UP) == 0 && digitalRead(SWITCH_ELEVATOR_DOWN) == 1) {
    send_data.switch_status[2] = 0;
  }

  if (digitalRead(SWITCH_FLAP) == 0) {
    send_data.switch_status[3] = 1;
  }
  if (digitalRead(SWITCH_AUTO) == 0) {
    send_data.switch_status[4] = 0;
  }
}

void dataProcess() {
  int angle1 = map(getAnalogHat(diffrential), JOYSTICK_ADC_OUT_MIN, JOYSTICK_ADC_OUT_MAX, ADC_MIN, SERVO_ANGLE_RANGE);
  int angle2 = map(getAnalogHat(aileron), JOYSTICK_ADC_OUT_MIN, JOYSTICK_ADC_OUT_MAX, ADC_MIN, SERVO_ANGLE_RANGE);
  int angle3 = SERVO_ANGLE_RANGE - map(getAnalogHat(aileron), JOYSTICK_ADC_OUT_MIN, JOYSTICK_ADC_OUT_MAX, ADC_MIN, SERVO_ANGLE_RANGE);
  int angle4 = map(getAnalogHat(elevator), JOYSTICK_ADC_OUT_MIN, JOYSTICK_ADC_OUT_MAX, ADC_MIN, SERVO_ANGLE_RANGE);
  switch (switchstate) {
  case SEND_DATA_ON:
    send_data.joystick_cur_val[0] = getAnalogHat(throttle);
    send_data.joystick_cur_val[1] = getAnalogHat(diffrential);
    send_data.joystick_cur_val[2] = getAnalogHat(aileron);
    send_data.joystick_cur_val[3] = getAnalogHat(elevator);
    send_data.diffrential_coe     = 0.0;
    pad.joystick_cur_val[0]       = send_data.joystick_cur_val[0];
    pad.joystick_cur_val[1]       = send_data.joystick_cur_val[1];
    pad.joystick_cur_val[2]       = send_data.joystick_cur_val[2];
    pad.joystick_cur_val[3]       = send_data.joystick_cur_val[3];
    pad.send_icon                 = SEND_ON;
    break;
  case REMOTE_OFF:
    send_data.switch_status[0]    = 0;
    send_data.switch_status[1]    = 0;
    send_data.switch_status[2]    = 0;
    send_data.switch_status[3]    = 0;
    send_data.switch_status[4]    = 0;
    send_data.joystick_cur_val[0] = -255;
    send_data.joystick_cur_val[1] = 0;
    send_data.joystick_cur_val[2] = 0;
    send_data.joystick_cur_val[3] = 0;
    send_data.diffrential_coe     = 0.0;
    pad.joystick_cur_val[0]       = send_data.joystick_cur_val[0];
    pad.joystick_cur_val[1]       = send_data.joystick_cur_val[1];
    pad.joystick_cur_val[2]       = send_data.joystick_cur_val[2];
    pad.joystick_cur_val[3]       = send_data.joystick_cur_val[3];
    pad.send_icon                 = SEND_OFF;
    break;

  default:
    break;
  }
}
/**
 * @brief 发送数据任务
 * 初始化ESP NOW，发送数据
 */
void sendDataTask(void* pvParameters) {
  ESP_NOW_Init();
  battery.init(BATTERY_PIN, R1, R2, BATTERY_MAX_VALUE, BATTERY_MIN_VALUE);

  TickType_t       xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod       = pdMS_TO_TICKS(12.5); // 频率 80Hz → 周期为 1/80 = 0.0125 秒 = 12.5 毫秒
  while (1) {
    switchStateIdentify();
    dataProcess();
    xQueueSend(PadDataQueue, &pad, QUEUE_MESSAGE_WAIT / portTICK_PERIOD_MS);
    esp_now_send(airCraftAddress, (uint8_t*)&send_data, sizeof(pad));
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

/**
 * @brief 任务和队列初始化
 */
void sendData_init() {
  PadDataQueue = xQueueCreate(3, sizeof(Pad));
  xTaskCreatePinnedToCore(sendDataTask, "sendData", 2048, NULL, 1, NULL, 0);
}