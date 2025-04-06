#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "XFS.h"      //封装好的命令库
#include "TextTab.h"  //中文需要放在该记事本中（因为编码不兼容）
#include <esp_now.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <math.h>

// 非阻塞式超声波测距模块（带滑动窗口）
#define TRIG_PIN 18
#define ECHO_PIN 19
// ==================== 超声波相关变量 ====================
float currentDistance = 0;
float learnedStableDistance = 0;
bool echoCaptured = false;
unsigned long echoStartTime = 0, echoEndTime = 0;
unsigned long lastTriggerTime = 0;
bool learningPhase = true;
unsigned long learningStartTime = 0;
float distanceSum = 0;
int distanceCount = 0;
const int LEARNING_DURATION = 5000;  // 5秒学习时间
const float DISTANCE_DELTA = 20.0;   // 缩短超过10cm视为障碍物


// 4G模块 使用 UART2
// HardwareSerial mySerial2(2);
#define RXD 4
#define TXD 5

// JY901传感器初始化
HardwareSerial MySerial(1);
#define RXD2 16
#define TXD2 17
// #define RXD2 26
// #define TXD2 25


uint8_t buffer[11];
// 摔倒检测滑动窗口设置
const int WINDOW_SIZE = 20;
bool fallWindow[WINDOW_SIZE];
int windowIndex = 0;
// 摔倒检测参数
bool isLying = false;
unsigned long lieStartTime = 0;
unsigned long lastAlarmTime = 0;
const unsigned long LIE_DURATION_THRESHOLD = 5000;  // 超过60秒开始报警
const unsigned long ALARM_INTERVAL = 1000;          // 每秒报警一次


/*实例化语音合成对象*/
XFS5152CE xfs;
/*超时设置，示例为30S*/
static uint32_t LastSpeakTime = 0;
#define SpeakTimeOut 10000

QueueHandle_t voiceQueue;  // 语音播报队列句柄

// 任务句柄
TaskHandle_t i2cTaskHandle = NULL;
TaskHandle_t uartTaskHandle = NULL;
TaskHandle_t gpioTaskHandle = NULL;
TaskHandle_t espNowTaskHandle = NULL;
TaskHandle_t voiceTaskHandle = NULL;  // 语音播报任务

// ================== 语音播报任务（准备阶段） ==================
// 定义语音播报编号的枚举
enum VoiceMessage {
  ESP_NOW_STRAIGHT_BLIND_PATH = 1,  // 来自ESP-NOW的前方直行盲道
  ESP_NOW_ZEBRA_CROSSING_OR_TURN,   // 来自ESP-NOW的前方斑马线或转弯
  ESP_NOW_GREEN_LIGHT,              // 来自ESP-NOW的绿灯请放心通行
  ESP_NOW_RED_LIGHT,                // 来自ESP-NOW的红灯请等待片刻
  ESP_NOW_WAITING,                  // 来自ESP-NOW的时间不足请等待
  UART_EMERGENCY,                   // 来自UART的紧急求救
  UART_FALL_DETECTED,               // 来自UART的检测到摔倒，请协助
  GPIO_ONE_METER_OBSTACLE,          // 来自IIC的一米内有障碍
  GPIO_HALF_METER_OBSTACLE          // 来自IIC的半米内有障碍
};

// 语音播报词典：根据编号返回对应的内容
const char *voiceDict(VoiceMessage messageId) {
  switch (messageId) {
    case ESP_NOW_STRAIGHT_BLIND_PATH: return "来自ESP-NOW的前方直行盲道";
    case ESP_NOW_ZEBRA_CROSSING_OR_TURN: return "来自ESP-NOW的前方斑马线或转弯";
    case ESP_NOW_GREEN_LIGHT: return "来自ESP-NOW的绿灯请放心通行";
    case ESP_NOW_RED_LIGHT: return "来自ESP-NOW的红灯请等待片刻";
    case ESP_NOW_WAITING: return "来自ESP-NOW的时间不足请等待";
    case UART_EMERGENCY: return "来自UART的紧急求救，请协助";
    case UART_FALL_DETECTED: return "来自UART的检测到摔倒，请协助";
    case GPIO_ONE_METER_OBSTACLE: return "来自IIC的一米内有障碍";
    case GPIO_HALF_METER_OBSTACLE: return "来自IIC的半米内有障碍";
    default: return "未知编号";
  }
}

// ================== 语音播报任务（使用模块） ==================

// 定义结构体来存储语音消息和时间戳
typedef struct {
  VoiceMessage message;
  uint32_t timestamp;
} MessageWithTimestamp;

// 环形数组存储最近播报的消息及其时间戳
#define MAX_HISTORY 5                              // 存储的最大历史记录数
MessageWithTimestamp recentMessages[MAX_HISTORY];  // 存储最近消息的数组
uint8_t messageIndex = 0;                          // 环形数组的索引

// 获取当前时间戳（假设每次任务调用时系统时间已经精确到毫秒）
uint32_t getCurrentTimestamp() {
  return millis();  // 使用 millis() 获取当前时间戳（单位：毫秒）
}

// 判断消息是否在4秒内已经播报过
bool isMessageRecentlyReported(VoiceMessage message) {
  uint32_t currentTime = getCurrentTimestamp();
  for (int i = 0; i < MAX_HISTORY; i++) {
    if (recentMessages[i].message == message && (currentTime - recentMessages[i].timestamp) < 2000) {
      return true;  // 如果找到相同的消息且时间差小于4秒，认为是重复消息
    }
  }
  return false;  // 没有重复消息
}

// // ================== ESPNOW接收端（准备阶段） ==================
// 接收数据结构体
typedef struct struct_message {
  int data;
} struct_message;

struct_message myData;

/**
    @brief  初始化语音合成
    @param  无
    @retval 无
*/

uint8_t n = 1;
static void XFS_Init() {
  Serial.println("进入初始化");
  xfs.Begin(0x30);  //设备i2c地址，地址为0x30
  delay(n);
  xfs.SetReader(XFS5152CE::Reader_XiaoYan);  //设置发音人
  delay(n);
  xfs.SetEncodingFormat(XFS5152CE::GB2312);  //文本的编码格式
  delay(n);
  xfs.SetLanguage(xfs.Language_Auto);  //语种判断
  delay(n);
  xfs.SetStyle(XFS5152CE::Style_Continue);  //合成风格设置
  delay(n);
  xfs.SetArticulation(XFS5152CE::Articulation_Letter);  //设置单词的发音方式
  delay(n);
  xfs.SetSpeed(5);  //设置语速1~10
  delay(n);
  xfs.SetIntonation(5);  //设置语调1~10
  delay(n);
  xfs.SetVolume(10);  //设置音量1~10
  delay(n);
  Serial.println("退出初始化");
}

unsigned char result = 0xFF;

// ================== I2C 任务 ==================
// void i2cTask(void *pvParameters) {
//   VoiceMessage msg = I2C_ONE_METER_OBSTACLE;  // 假设发送编号：一米内有障碍
//   while (1) {
//     // 模拟发送编号到语音播报队列
//     xQueueSend(voiceQueue, &msg, portMAX_DELAY);
//     Serial.print("[I2C] 发送编号到语音队列：");
//     Serial.println(msg);
//     msg = (msg == I2C_ONE_METER_OBSTACLE) ? I2C_HALF_METER_OBSTACLE : I2C_ONE_METER_OBSTACLE;  // 切换编号
//     vTaskDelay(5000 / portTICK_PERIOD_MS);                                                     // 每5秒执行一次
//   }
// }


// ================== UART 任务 ==================
// JY901S TX → ESP32 GPIO16
// JY901S RX → ESP32 GPIO17
// GND → GND
// VCC → 3.3V
void processSensorData(uint8_t *data) {
  if (data[0] != 0x55 || data[1] != 0x51) return;  // 只处理加速度帧

  int16_t ax = (int16_t)(data[3] << 8 | data[2]);
  int16_t ay = (int16_t)(data[5] << 8 | data[4]);
  int16_t az = (int16_t)(data[7] << 8 | data[6]);

  float acc_x = ax / 32768.0f * 16;
  float acc_y = ay / 32768.0f * 16;
  float acc_z = az / 32768.0f * 16;

  float acc_mag = sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);

  float abs_x = fabs(acc_x);
  float abs_y = fabs(acc_y);
  float abs_z = fabs(acc_z);

  float acc_main_value = abs_x;
  char acc_main_axis = 'X';
  if (abs_y > acc_main_value) {
    acc_main_value = abs_y;
    acc_main_axis = 'Y';
  }
  if (abs_z > acc_main_value) {
    acc_main_value = abs_z;
    acc_main_axis = 'Z';
  }

  bool isLyingCurrent = false;
  if (acc_mag > 0.95 && acc_mag < 1.05 && acc_main_axis != 'Z') {
    isLyingCurrent = true;
  }

  fallWindow[windowIndex] = isLyingCurrent;
  windowIndex = (windowIndex + 1) % WINDOW_SIZE;

  int count = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    if (fallWindow[i]) count++;
  }

  if (count >= WINDOW_SIZE * 0.8) {
    if (!isLying) {
      lieStartTime = millis();
      lastAlarmTime = 0;
    }
    isLying = true;
  } else {
    isLying = false;
  }
  Serial.print("mag: ");
  Serial.print(acc_mag, 3);
  Serial.print(" | 主轴: ");
  Serial.print(acc_main_axis);
  Serial.print(" = ");
  Serial.print(acc_main_value, 3);
  Serial.print(" | 摔倒帧数: ");
  Serial.println(count);
}

void uartTask(void *pvParameters) {
  VoiceMessage msg = UART_EMERGENCY;
  
  while (1) {
    if (MySerial.available() < 11) {
      vTaskDelay(10 / portTICK_PERIOD_MS);  // 避免空读过快
      continue;
    }

    int header = MySerial.read();
    if (header == 0x55) {
      buffer[0] = 0x55;
      bool valid = true;
      for (int i = 1; i < 11; i++) {
        if (MySerial.available()) {
          buffer[i] = MySerial.read();
        } else {
          valid = false;
          break;
        }
      }
      if (valid) processSensorData(buffer);
    }

    if (isLying) {
      unsigned long now = millis();
      if (now - lieStartTime >= LIE_DURATION_THRESHOLD && now - lastAlarmTime >= ALARM_INTERVAL) {
        lastAlarmTime = now;
        xQueueSend(voiceQueue, &msg, portMAX_DELAY);
        Serial.print("[UART] 发送编号到语音队列：");
        Serial.println(msg);
        // mySerial2.write(0xFF);
      }
    }
    // vTaskDelay(5 / portTICK_PERIOD_MS);  // 控制频率
  }
}



// ================== ESP-NOW 任务 ==================
// 接收数据回调函数
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("来自esp-now的信号:");
  Serial.println(myData.data);
  int msg = myData.data;
  xQueueSend(voiceQueue, &msg, portMAX_DELAY);
  Serial.print("[ESP-NOW] 发送编号到语音队列：");
  Serial.println(msg);
  // vTaskDelay(10000 / portTICK_PERIOD_MS);                                                                     // 每10秒执行一次
}

void espNowTask(void *pvParameters) {

  WiFi.mode(WIFI_STA);  // Wi-Fi Station

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  Serial.println("注册 ESP-NOW");
  // 注册接收数据回调函数
  esp_now_register_recv_cb(OnDataRecv);
}

// ================== gpio任务 ==================
// VCC	5V
// TRIG	GPIO 18
// ECHO	GPIO 19
// GND	GND

void IRAM_ATTR echoFall() {
  echoEndTime = micros();
  echoCaptured = true;
  detachInterrupt(digitalPinToInterrupt(ECHO_PIN));
}

void IRAM_ATTR echoRise() {
  echoStartTime = micros();
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoFall, FALLING);
}

void initUltrasonic() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  learningStartTime = millis();
  distanceSum = 0;
  distanceCount = 0;
  learningPhase = true;
}

// 定义窗口参数
#define FILTER_WINDOW 5
float distanceWindow[FILTER_WINDOW] = {0};
int filterIndex = 0;

// 替换 updateUltrasonic 函数
void updateUltrasonic() {
  unsigned long now = millis();

  if (now - lastTriggerTime > 100) {
    lastTriggerTime = now;
    echoCaptured = false;
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoRise, RISING);
  }

  if (echoCaptured) {
    echoCaptured = false;
    unsigned long duration = echoEndTime - echoStartTime;
    currentDistance = duration * 0.0343 / 2.0;

    // 滤波处理：更新滑动窗口
    distanceWindow[filterIndex] = currentDistance;
    filterIndex = (filterIndex + 1) % FILTER_WINDOW;

    float sum = 0;
    for (int i = 0; i < FILTER_WINDOW; i++) sum += distanceWindow[i];
    float filteredDistance = sum / FILTER_WINDOW;

    if (learningPhase) {
      if (now - learningStartTime < LEARNING_DURATION) {
        if (filteredDistance > 5 && filteredDistance < 400) {
          distanceSum += filteredDistance;
          distanceCount++;
        }
      } else {
        learningPhase = false;
        if (distanceCount > 0) {
          learnedStableDistance = distanceSum / distanceCount;
        } else {
          learnedStableDistance = 100.0;  // 默认值
        }
        Serial.printf("学习阶段结束，参考距离为：%.1fcm\n", learnedStableDistance);
      }
    } else {
      Serial.printf("滤波后距离：%.1fcm | 基准：%.1fcm\n", filteredDistance, learnedStableDistance);
      if (filteredDistance < learnedStableDistance - DISTANCE_DELTA) {
        VoiceMessage msg = GPIO_ONE_METER_OBSTACLE;
        xQueueSend(voiceQueue, &msg, portMAX_DELAY);
      }
    }
  }
}


void gpioTask(void *pvParameters) {
  initUltrasonic();
  while (1) {
    updateUltrasonic();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}


// ================== 语音播报任务（消费者） ==================
// SDA(21) - SDA
// SCL(22) - SCL
// VIN - 5V
// GND - GND
void voiceTask(void *pvParameters) {
  // Serial.println("[DEBUG] voiceTask 已启动");

  VoiceMessage receivedMessage;

  while (1) {
    // 从队列接收消息
    // Serial.println("语音循环");
    if (xQueueReceive(voiceQueue, &receivedMessage, portMAX_DELAY) == pdPASS) {
      // Serial.println("接收到消息");
      // 判断消息是否在4秒内已经播报过
      if (!isMessageRecentlyReported(receivedMessage)) {
        // 如果没有重复消息，则进行播报
        const char *message = voiceDict(receivedMessage);
        Serial.println("");
        Serial.println("");
        Serial.println("==================");
        Serial.print("语音播报队列中接收到编号：");
        Serial.print(receivedMessage);
        Serial.print(" - 内容：");
        Serial.println(message);
        Serial.println("==================");
        Serial.println("");
        Serial.println("");

        // 播报语音消息
        xfs.StartSynthesis(TextTab1[receivedMessage - 1]);
        while (xfs.GetChipStatus() != xfs.ChipStatus_Idle) {
          delay(30);  // 等待语音合成完成
        }

        // 更新最近播报的消息历史记录
        recentMessages[messageIndex].message = receivedMessage;
        recentMessages[messageIndex].timestamp = getCurrentTimestamp();
        messageIndex = (messageIndex + 1) % MAX_HISTORY;  // 环形更新索引
      } else {
        // 如果消息已经播报过，跳过
        Serial.print("跳过重复消息：");
        Serial.println(receivedMessage);
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  //初始化超声波模块
  // initUltrasonic();
  // 初始化串口 2 (UART2)，假设 RX = GPIO 4, TX = GPIO 5
  mySerial2.begin(115200, SERIAL_8N1, RXD, TXD);  // RX = GPIO 4, TX = GPIO 5
  MySerial.begin(9600, SERIAL_8N1, RXD2, TXD2);  // 16 17
  
  // Serial.println("=== ESP32 泛化躺下检测启动 ===");
  for (int i = 0; i < WINDOW_SIZE; i++) fallWindow[i] = false;

  // 创建语音播报队列（队列长度为10，单个消息大小为int类型）
  voiceQueue = xQueueCreate(10, sizeof(VoiceMessage));
  if (voiceQueue == NULL) {
    Serial.println("队列创建失败");
    while (1)
      ;
  }

  Serial.println("注册ESP-NOW的WIFI");
  WiFi.mode(WIFI_STA);  // Wi-Fi Station

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // 注册接收数据回调函数
  esp_now_register_recv_cb(OnDataRecv);

  // 创建任务
  Serial.println("注册voiceTask");
  // 初始化语音合成模块
  XFS_Init();
  xTaskCreate(voiceTask, "Voice_Task", 2048, NULL, 6, &voiceTaskHandle);  // 语音播报任务
  Serial.println("注册gpioTask");
  xTaskCreate(gpioTask, "GPIO_Task", 2048, NULL, 5, &gpioTaskHandle);  // 现在定义了gpioTask函数
  Serial.println("注册uartTask");
  xTaskCreate(uartTask, "UART_Task", 2048, NULL, 8, &uartTaskHandle);
  Serial.println("注册espNowTask");
  xTaskCreate(espNowTask, "ESP_NOW_Task", 2048, NULL, 8, &espNowTaskHandle);
}

void loop() {
  // 所有任务由 FreeRTOS 调度，loop 为空
}
