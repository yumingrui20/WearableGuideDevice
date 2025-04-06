import os
import pandas as pd
import numpy as np

# 标签映射（按编号）
labels = {
    0: "走路",
    1: "站立",
    2: "静坐",
    3: "躺下"
}

# 输出目录
output_dir = "analysis_data_csv"
os.makedirs(output_dir, exist_ok=True)

all_stats = {}
available_indices = []

def load_data(index):
    filename = f"uart_data_{index}.csv"
    if not os.path.exists(filename):
        print(f"⚠️ 缺失文件：{filename}，跳过")
        return None
    df = pd.read_csv(filename)
    df = df[df["Time(ms)"] != "---"]
    df["Time(ms)"] = pd.to_numeric(df["Time(ms)"])
    return df

def extract_stats(df, sensor_type):
    df_type = df[df["Type"] == sensor_type].copy()
    for axis in ["X", "Y", "Z"]:
        df_type[axis] = pd.to_numeric(df_type[axis])
    return df_type[["X", "Y", "Z"]].mean(), df_type[["X", "Y", "Z"]].std()

mean_rows, std_rows, diff_rows = [], [], []

for i in range(4):
    df = load_data(i)
    if df is None:
        continue

    label = labels[i]
    available_indices.append(i)
    all_stats[label] = {}

    row_mean = {"动作": label}
    row_std = {"动作": label}

    for sensor in ["ACC", "GYRO", "MAG"]:
        mean, std = extract_stats(df, sensor)
        all_stats[label][f"{sensor}_mean"] = mean
        all_stats[label][f"{sensor}_std"] = std
        for axis in ["X", "Y", "Z"]:
            row_mean[f"{sensor}_{axis}"] = round(mean[axis], 3)
            row_std[f"{sensor}_{axis}"] = round(std[axis], 3)

    mean_rows.append(row_mean)
    std_rows.append(row_std)

df_mean = pd.DataFrame(mean_rows)
df_std = pd.DataFrame(std_rows)

# 差异分析（躺下为基准）
if "躺下" in all_stats:
    ref = all_stats["躺下"]
    for i in available_indices:
        name = labels[i]
        if name == "躺下":
            continue
        row_diff = {"动作": name}
        diffs = {}
        for sensor in ["ACC", "GYRO", "MAG"]:
            for axis in ["X", "Y", "Z"]:
                ref_val = ref[f"{sensor}_mean"][axis]
                cur_val = all_stats[name][f"{sensor}_mean"][axis]
                diff_val = abs(cur_val - ref_val)
                row_diff[f"{sensor}_{axis}"] = round(diff_val, 3)
                diffs[f"{sensor}_{axis}"] = diff_val
        max_feature = max(diffs, key=diffs.get)
        row_diff["最显著差异"] = max_feature
        diff_rows.append(row_diff)

    df_diff = pd.DataFrame(diff_rows)
else:
    df_diff = pd.DataFrame()

# 添加类型列
df_mean["统计类型"] = "均值"
df_std["统计类型"] = "标准差"
if not df_diff.empty:
    df_diff["统计类型"] = "与躺下差异"

# 合并总表
df_all = pd.concat([df_mean, df_std, df_diff], ignore_index=True)
cols = ["统计类型", "动作"] + [col for col in df_all.columns if col not in ["统计类型", "动作", "最显著差异"]]
if "最显著差异" in df_all.columns:
    cols.append("最显著差异")
df_all = df_all[cols]

# 保存表格
output_file = os.path.join(output_dir, "sensor_combined_summary.csv")
df_all.to_csv(output_file, index=False)
print(f"✅ 合并统计文件已保存：{output_file}")

if "躺下" in all_stats:
    acc_ref = ref["ACC_mean"]
    acc_std = ref["ACC_std"]
    mag_ref = ref["MAG_mean"]

    acc_ref_mag = np.linalg.norm(acc_ref)
    acc_ref_axis = acc_ref.abs()
    acc_ref_main_axis = acc_ref_axis.idxmax()
    acc_ref_main_value = acc_ref[acc_ref_main_axis]

    mag_ref_norm = np.linalg.norm(mag_ref)
    unit_y_ref = abs(mag_ref["Y"]) / mag_ref_norm
    unit_z_ref = abs(mag_ref["Z"]) / mag_ref_norm

    print("\n✅ 实时姿态判定建议（基于主轴动态识别）：")
    print(f"- 躺下状态下主导轴为：{acc_ref_main_axis}，平均加速度为：{acc_ref_main_value:.2f}")
    print(f"- 躺下状态下磁方向投影比例：unit_y={unit_y_ref:.2f}，unit_z={unit_z_ref:.2f}")

    print("\n建议伪代码：")
    print(f"""
获取当前加速度模长 acc_magnitude 和 abs(acc_x/y/z)，找出最大轴方向：

main_axis = max(abs(acc_x), abs(acc_y), abs(acc_z)) 对应的轴

if 0.95 < acc_magnitude < 1.05:
    if main_axis == '{acc_ref_main_axis}' and
       abs(acc_main_value - {acc_ref_main_value:.2f}) < {acc_std.mean():.2f}:
        状态 = "躺下（主导轴匹配）"
    else:
        状态 = "其他"
""")
else:
    print("⚠️ 无躺下参考，无法生成动态判断建议")



