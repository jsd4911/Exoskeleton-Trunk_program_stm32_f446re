import pandas as pd
import matplotlib.pyplot as plt

# --- 1. 載入數據 ---
# 讀取 CSV 檔案 (請換成您實際存下的檔名，例如 'BNO085_data_20260504_151401.csv')
filename = "BNO085_data_20260506_102638.csv" 

print(f"正在讀取檔案: {filename} ...")
df = pd.read_csv(filename)

# 簡單印出前 5 筆資料，確認有沒有讀錯
print("數據預覽：")
print(df.head())


# --- 2. 數據視覺化 (繪製三軸子圖) ---
# 建立一個大畫布，裡面包含 3 張上下排列的子圖，並共用底部的 X 軸 (時間)
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
fig.suptitle('BNO085 IMU 原始姿態數據 (Raw Data)', fontsize=16)

# 畫第一張圖：Roll (翻滾角)
ax1.plot(df['Time(s)'], df['Roll(deg)'], color='blue', linewidth=1.5, label='Roll (Left/Right Tilt)')
ax1.set_ylabel('Degrees')
ax1.legend(loc='upper right')
ax1.grid(True, linestyle='--', alpha=0.6)

# 畫第二張圖：Pitch (俯仰角) 👉 這是您彎腰的關鍵！
ax2.plot(df['Time(s)'], df['Pitch(deg)'], color='orange', linewidth=2.0, label='Pitch (Bending)')
ax2.set_ylabel('Degrees')
ax2.legend(loc='upper right')
ax2.grid(True, linestyle='--', alpha=0.6)

# 畫第三張圖：Yaw (偏航角)
ax3.plot(df['Time(s)'], df['Yaw(deg)'], color='green', linewidth=1.5, label='Yaw (Rotation)')
ax3.set_xlabel('Time (seconds)', fontsize=12)
ax3.set_ylabel('Degrees')
ax3.legend(loc='upper right')
ax3.grid(True, linestyle='--', alpha=0.6)

# 自動調整排版，避免字體重疊
plt.tight_layout()

# 顯示圖表
plt.show()