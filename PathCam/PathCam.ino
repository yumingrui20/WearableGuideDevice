// 发送设备代码
// 发送四个类型的数据

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "k210_msg_deal.h"

#define K210Serial Serial
msg_k210 k210_msg;
char buff_com[50];


// 2C:BC:BB:93:32:08
// 08:D1:F9:EB:52:E8

//D4:8C:49:E3:87:68 发送端
//30:C6:F7:39:46:C8  接收端
uint8_t broadcastAddress[] = { 0x30, 0xC6, 0xF7, 0x39, 0x46, 0xC8 };

// 发送的数据结构
typedef struct struct_message {
  int data;
} struct_message;

struct_message myData;

esp_now_peer_info_t peerInfo;


// 发送数据回调函数
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // 打印消息是否成功传递
  Serial.print("Last Packet Send Status:");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


void setup() {
  Serial.begin(115200);
  K210Serial.begin(115200);

  WiFi.mode(WIFI_STA);  // Wi-Fi Station

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);  // 注册发送数据的回调函数

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);  // 复制 MAC 地址
  peerInfo.channel = 0;                             // 使用当前打开的通道
  peerInfo.encrypt = false;                         // 未加密

  // 添加以上列表
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}


void loop() {
  // // 设置发送信息
  // myData.data = random(1, 6);

  // esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));  // 发送

  // if (result == ESP_OK) {
  //   Serial.println("Sent with success");
  // } else {
  //   Serial.println("Error sending the data");
  // }

  // delay(5000);



  while (K210Serial.available()) {
    recv_k210msg(K210Serial.read());

    if (k210_msg.class_n != 0)  //例程号不为空
    {
      if (k210_msg.class_n == 1)  //是条形码识别
      {
        sprintf(buff_com, "x=%d,y=%d,w=%d,h=%d\r\n", k210_msg.x, k210_msg.y, k210_msg.w, k210_msg.h);
        K210Serial.print(buff_com);

        sprintf(buff_com, "str = %s\r\n", k210_msg.msg_msg);
        K210Serial.print(buff_com);

        k210_msg.class_n = 0;  //清除例程号

        //使用espnow发送数据
        myData.data = atoi((char*)k210_msg.msg_msg);  // 使用 atoi() 将字符数组转换为整数                                              // 假设 k210_msg.msg_msg 是 String 类型的 "2"
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));  // 发送
        Serial.println(myData.data);

        if (result == ESP_OK) {
          Serial.println("Sent with success");
        } else {
          Serial.println("Error sending the data");
        }
      }
    }
  }
}