import os
import pandas as pd
import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams['font.sans-serif'] = ['SimHei']
matplotlib.rcParams['axes.unicode_minus'] = False

import matplotlib.pyplot as plt

# ===== 指定处理的 CSV 文件序号 =====
csv_index = 3
csv_filename = f"uart_data_{csv_index}.csv"
output_dir = f"uart_data_image/uart_data_image_{csv_index}"
combined_dir = "combined_image"

# ===== 创建输出目录 =====
os.makedirs(output_dir, exist_ok=True)
os.makedirs(combined_dir, exist_ok=True)

# ===== 读取数据文件 =====
df = pd.read_csv(csv_filename)
df = df[df["Time(ms)"] != "---"]
df["Time(ms)"] = pd.to_numeric(df["Time(ms)"])

# ===== 分离数据类型 =====
df_acc = df[df["Type"] == "ACC"].copy()
df_gyro = df[df["Type"] == "GYRO"].copy()
df_mag = df[df["Type"] == "MAG"].copy()

# ===== 转换为数值型 =====
for axis in ["X", "Y", "Z"]:
    df_acc[axis] = pd.to_numeric(df_acc[axis])
    df_gyro[axis] = pd.to_numeric(df_gyro[axis])
    df_mag[axis] = pd.to_numeric(df_mag[axis])

# ===== ACC 图 =====
plt.figure(figsize=(10, 4))
plt.plot(df_acc["Time(ms)"], df_acc["X"], label="ACC_X")
plt.plot(df_acc["Time(ms)"], df_acc["Y"], label="ACC_Y")
plt.plot(df_acc["Time(ms)"], df_acc["Z"], label="ACC_Z")
plt.title("加速度 (ACC)")
plt.xlabel("时间 (ms)")
plt.ylabel("加速度 (g)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(output_dir, "acc_plot.png"))

# ===== GYRO 图 =====
plt.figure(figsize=(10, 4))
plt.plot(df_gyro["Time(ms)"], df_gyro["X"], label="GYRO_X")
plt.plot(df_gyro["Time(ms)"], df_gyro["Y"], label="GYRO_Y")
plt.plot(df_gyro["Time(ms)"], df_gyro["Z"], label="GYRO_Z")
plt.title("角速度 (GYRO)")
plt.xlabel("时间 (ms)")
plt.ylabel("角速度 (°/s)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(output_dir, "gyro_plot.png"))

# ===== MAG 图 =====
plt.figure(figsize=(10, 4))
plt.plot(df_mag["Time(ms)"], df_mag["X"], label="MAG_X")
plt.plot(df_mag["Time(ms)"], df_mag["Y"], label="MAG_Y")
plt.plot(df_mag["Time(ms)"], df_mag["Z"], label="MAG_Z")
plt.title("磁力计 (MAG)")
plt.xlabel("时间 (ms)")
plt.ylabel("磁场强度 (μT)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(output_dir, "mag_plot.png"))

# ===== 综合图 =====
plt.figure(figsize=(12, 6))
plt.plot(df_acc["Time(ms)"], df_acc["X"], label="ACC_X", linestyle="--")
plt.plot(df_acc["Time(ms)"], df_acc["Y"], label="ACC_Y", linestyle="--")
plt.plot(df_acc["Time(ms)"], df_acc["Z"], label="ACC_Z", linestyle="--")
plt.plot(df_gyro["Time(ms)"], df_gyro["X"], label="GYRO_X", linestyle="-")
plt.plot(df_gyro["Time(ms)"], df_gyro["Y"], label="GYRO_Y", linestyle="-")
plt.plot(df_gyro["Time(ms)"], df_gyro["Z"], label="GYRO_Z", linestyle="-")
plt.plot(df_mag["Time(ms)"], df_mag["X"], label="MAG_X", linestyle=":")
plt.plot(df_mag["Time(ms)"], df_mag["Y"], label="MAG_Y", linestyle=":")
plt.plot(df_mag["Time(ms)"], df_mag["Z"], label="MAG_Z", linestyle=":")
plt.title("九轴综合图 (ACC + GYRO + MAG)")
plt.xlabel("时间 (ms)")
plt.ylabel("数值 (g / °/s / μT)")
plt.legend()
plt.grid(True)
plt.tight_layout()

combined_filename = os.path.join(combined_dir, f"combined_plot_{csv_index}.png")
plt.savefig(combined_filename)

print(f"✅ 单独图已保存到：{output_dir}")
print(f"✅ 综合图已保存为：{combined_filename}")
