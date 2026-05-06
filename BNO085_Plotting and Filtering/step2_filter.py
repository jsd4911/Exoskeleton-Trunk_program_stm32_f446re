import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt

# --- 1. 載入原始數據 ---
filename = "BNO085_data_20260506_102638.csv" 
df = pd.read_csv(filename)

# 我們專注處理彎腰的關鍵數據：Pitch
time = df['Time(s)']
pitch_raw = df['Pitch(deg)']

# --- 2. 方法一：移動平均濾波 (Moving Average) ---
# window=10 代表取過去 10 筆資料平均 (BNO085 為 20Hz，10筆約等於 0.5 秒)
df['Pitch_MA'] = pitch_raw.rolling(window=10, center=False).mean()

# --- 3. 方法二：巴特沃斯零相移低通濾波 (Butterworth Zero-phase Low-pass) ---
# 設定濾波參數
fs = 20.0       # 感測器取樣率 (20Hz)
cutoff = 2.0    # 截止頻率 (2Hz，人體動作通常低於這個頻率)
order = 4       # 濾波器階數 (階數越高，切斷越乾淨)

# 計算濾波器係數 (Nyquist 頻率為 fs/2)
nyq = 0.5 * fs
normal_cutoff = cutoff / nyq
b, a = butter(order, normal_cutoff, btype='low', analog=False)

# 使用 filtfilt 進行雙向濾波 (消除時間延遲！)
df['Pitch_Butter'] = filtfilt(b, a, pitch_raw)

# --- 4. 視覺化比較 ---
plt.figure(figsize=(12, 6))
plt.title('BNO085 Pitch Data: Raw vs Filtered', fontsize=16)

# 畫出原始雜訊 (淺灰色，方便當背景比對)
plt.plot(time, pitch_raw, label='Raw Data (原始雜訊)', color='lightgray', alpha=0.8, linewidth=1.5)

# 畫出移動平均 (藍色虛線)
plt.plot(time, df['Pitch_MA'], label='Moving Average (Window=10)', color='blue', linestyle='--', linewidth=2)

# 畫出巴特沃斯 (紅色實線 - 完美平滑且不延遲)
plt.plot(time, df['Pitch_Butter'], label='Butterworth Low-pass (2Hz, Zero-phase)', color='red', linewidth=2.5)

plt.xlabel('Time (seconds)', fontsize=12)
plt.ylabel('Pitch Angle (Degrees)', fontsize=12)
plt.legend(loc='upper right')
plt.grid(True, linestyle='--', alpha=0.6)
plt.tight_layout()
plt.show()

# (選擇性) 把乾淨的數據存成新的 CSV，準備給後續機器學習使用
# df.to_csv("BNO085_Clean_Data.csv", index=False)