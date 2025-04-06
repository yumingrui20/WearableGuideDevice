import serial
import csv
import os
import sys

PORT = "COM8"
BAUDRATE = 115200
FIXED_INDEX = 3
FILENAME = f"uart_data_{FIXED_INDEX}.csv"
MAX_LINES = 500  # 写入行数达到该值后退出

ser = serial.Serial(PORT, BAUDRATE, timeout=1)

file = None
writer = None
recording = False
line_count = 0

print("等待 ESP32 重启标志（--- ESP32 RESTARTED ---）...")

try:
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()

        if not line:
            continue

        if "--- ESP32 RESTARTED ---" in line:
            print("\n=== 检测到 ESP32 重启 ===")

            if os.path.exists(FILENAME):
                print(f"文件 {FILENAME} 已存在，跳过记录。")
                break

            file = open(FILENAME, mode='w', newline='')
            writer = csv.writer(file)
            writer.writerow(["Time(ms)", "Type", "X", "Y", "Z"])
            writer.writerow(["---", "ESP32 RESET", "", "", ""])
            recording = True

            print(f"=== 开始记录：{FILENAME} ===")
            continue

        if not recording:
            continue

        if line.count(',') == 4:
            parts = line.split(',')
            type_str = parts[1]
            if type_str in ["ACC", "GYRO", "MAG"]:
                writer.writerow(parts)
                line_count += 1
                print(line)

        if line_count >= MAX_LINES:
            print(f"\n✅ 已写入 {line_count} 行数据，程序自动结束。")
            break

except KeyboardInterrupt:
    print("\n用户中断，程序终止。")

finally:
    if file:
        file.close()
    ser.close()
