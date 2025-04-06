#include <HardwareSerial.h>

// 使用 UART1 接 JY901S
HardwareSerial MySerial(1);

#define RXD2 16  // ESP32 接收引脚（接 JY901S TX）
#define TXD2 17  // ESP32 发送引脚（一般不用）

uint8_t buffer[11];

void setup() {
  Serial.begin(115200);
  MySerial.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("--- ESP32 RESTARTED ---");
}

void loop() {
  // 接收到数据才处理
  if (MySerial.available() >= 11) {
    if (MySerial.read() == 0x55) {
      buffer[0] = 0x55;
      for (int i = 1; i < 11; i++) {
        buffer[i] = MySerial.read();
      }
      processData(buffer);
    }
  }
}

void processData(uint8_t* data) {
  if (data[0] != 0x55) return;

  char type = data[1];
  if (type != 0x51 && type != 0x52 && type != 0x54) return;  // 加速度、角速度、磁力计

  int16_t a = (int16_t)(data[3] << 8 | data[2]);
  int16_t b = (int16_t)(data[5] << 8 | data[4]);
  int16_t c = (int16_t)(data[7] << 8 | data[6]);

  float x = 0, y = 0, z = 0;
  String label;
  unsigned long timestamp = millis();

  if (type == 0x51) {  // 加速度
    label = "ACC";
    x = a / 32768.0 * 16;
    y = b / 32768.0 * 16;
    z = c / 32768.0 * 16;
  } else if (type == 0x52) {  // 角速度
    label = "GYRO";
    x = a / 32768.0 * 2000;
    y = b / 32768.0 * 2000;
    z = c / 32768.0 * 2000;
  } else if (type == 0x54) {  // 磁力计
    label = "MAG";
    x = a / 32768.0 * 1200;  // ±1200 μT
    y = b / 32768.0 * 1200;
    z = c / 32768.0 * 1200;
  }

  // 串口输出数据
  Serial.print(timestamp);
  Serial.print(",");
  Serial.print(label);
  Serial.print(",");
  Serial.print(x, 3);
  Serial.print(",");
  Serial.print(y, 3);
  Serial.print(",");
  Serial.println(z, 3);
}
