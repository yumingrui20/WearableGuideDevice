import serial
import csv
import os

PORT = "COM8"  # 替换为你的串口号
BAUDRATE = 115200

ser = serial.Serial(PORT, BAUDRATE, timeout=1)

file = None
writer = None
recording = False

def generate_next_filename(prefix="uart_data_", ext=".csv"):
    index = 0
    while True:
        filename = f"{prefix}{index}{ext}"
        if not os.path.exists(filename):
            return filename
        index += 1

print("等待 ESP32 重启标志（--- ESP32 RESTARTED ---）...")

try:
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()

        if not line:
            continue

        # ✅ 严格要求：必须收到重启标志才开始记录
        if "--- ESP32 RESTARTED ---" in line:
            print("\n=== 检测到 ESP32 重启 ===")

            if file:
                file.close()
                print("=== 上一个文件已保存 ===")

            # 创建新文件
            filename = generate_next_filename()
            file = open(filename, mode='w', newline='')
            writer = csv.writer(file)
            writer.writerow(["Time(ms)", "Type", "X", "Y", "Z"])
            writer.writerow(["---", "ESP32 RESET", "", "", ""])
            recording = True

            print(f"=== 开始新文件记录：{filename} ===")
            continue

        # ❗没有收到重启前，哪怕是数据也不记录
        if not recording:
            continue

        # ✅ 收到重启后，记录 ACC / GYRO / MAG 数据
        if line.count(',') == 4:
            parts = line.split(',')
            type_str = parts[1]
            if type_str in ["ACC", "GYRO", "MAG"]:
                writer.writerow(parts)
                print(line)

except KeyboardInterrupt:
    print("\n程序终止，关闭文件与串口...")
    if file:
        file.close()
    ser.close()
