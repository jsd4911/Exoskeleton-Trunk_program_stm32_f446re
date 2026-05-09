import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
import os

# ==========================================
# 參數設定區
# ==========================================
INPUT_FILE = "BNO085_DeltaData_20260508_174435.csv"  # 您的第二次測試檔案
OUTPUT_FILE = "BNO085_Train_Delta_Labeled_10.csv" 

# 濾波與防抖參數
FS = 20.0       
CUTOFF = 2.0    
SMOOTH_WINDOW = 10  # 10筆滑動平均，解決 1,2 狀態來回跳動的問題

# ⚠️ 動作標記參數 (已經改為「相對角度」)
BEND_THRESHOLD = 15.0    # 只要相對起點「彎超過 15 度」，就視為脫離站直
DELTA_THRESHOLD = 0.08   # 變化量閾值

# ==========================================
# 步驟 A: 讀取原始數據
# ==========================================
if not os.path.exists(INPUT_FILE):
    print(f"找不到檔案 {INPUT_FILE}！")
    exit()
df = pd.read_csv(INPUT_FILE)

# ==========================================
# 步驟 B: 濾波、自動歸零 (Auto-Tare) 與防抖
# ==========================================
print("執行低通濾波與自動歸零...")
nyq = 0.5 * FS
normal_cutoff = CUTOFF / nyq
b, a = butter(4, normal_cutoff, btype='low', analog=False)

# 1. 低通濾波
df['Roll_Filtered']  = filtfilt(b, a, df['Roll(deg)'])
df['Pitch_Filtered'] = filtfilt(b, a, df['Pitch(deg)'])
df['Yaw_Filtered']   = filtfilt(b, a, df['Yaw(deg)'])

# 🌟 2. 自動歸零引擎 (Auto-Tare)：取前 10 筆站立資料當作絕對零點
tare_roll  = df['Roll_Filtered'].head(10).mean()
tare_pitch = df['Pitch_Filtered'].head(10).mean()
tare_yaw   = df['Yaw_Filtered'].head(10).mean()

# 將所有角度扣除零點，得到「相對純淨角度」
df['Pitch_Normalized'] = df['Pitch_Filtered'] - tare_pitch
# 注意：若彎腰時數值是往負的走，加個絕對值或乘-1，確保彎腰都是正數
if df['Pitch_Normalized'].max() < abs(df['Pitch_Normalized'].min()):
    df['Pitch_Normalized'] = df['Pitch_Normalized'] * -1.0

# 3. 防抖變化量 (Delta) 計算
raw_delta_pitch = df['Pitch_Normalized'].diff().fillna(0)
df['Delta_Pitch_Smoothed'] = raw_delta_pitch.rolling(window=SMOOTH_WINDOW, center=True, min_periods=1).mean()

# ==========================================
# 步驟 C: 四態自動標註 (0=站直, 1=彎下, 2=彎起, 3=維持)
# ==========================================
print("執行四態標記...")
df['Label'] = 0  

# 以「歸零後的相對角度」來判斷是否彎腰 (不再依賴絕對 -65 或 -88)
is_bent = df['Pitch_Normalized'] > BEND_THRESHOLD

df.loc[is_bent & (df['Delta_Pitch_Smoothed'] > DELTA_THRESHOLD), 'Label'] = 1
df.loc[is_bent & (df['Delta_Pitch_Smoothed'] < -DELTA_THRESHOLD), 'Label'] = 2
df.loc[is_bent & (df['Delta_Pitch_Smoothed'] >= -DELTA_THRESHOLD) & (df['Delta_Pitch_Smoothed'] <= DELTA_THRESHOLD), 'Label'] = 3

print("\n--- 標記結果統計 ---")
print(df['Label'].value_counts().sort_index().rename({0: '站直 (0)', 1: '彎下 (1)', 2: '彎起 (2)', 3: '維持 (3)'}))

# ==========================================
# 步驟 D: 視覺化與存檔
# ==========================================
# 提取給 AI 的特徵
final_df = df[['Time(s)', 'Pitch_Normalized', 'Delta_Pitch_Smoothed', 'Label']]
final_df.to_csv(OUTPUT_FILE, index=False)
print(f"✅ 完美數據已儲存為: {OUTPUT_FILE}")

# 畫圖
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
ax1.plot(df['Time(s)'], df['Pitch_Normalized'], color='blue', linewidth=2, label='Normalized Pitch (Starts at 0)')
ax1.axhline(y=BEND_THRESHOLD, color='gray', linestyle='--', label=f'Bend Threshold (+{BEND_THRESHOLD}°)')
ax1.set_ylabel('Relative Degrees')
ax1.legend(loc='upper right')
ax1.grid(True)
ax1.set_title('Step 1: Auto-Tared Posture (Immune to Sensor Drift)')

ax2.plot(df['Time(s)'], df['Delta_Pitch_Smoothed'], color='orange', label='Smoothed Delta Pitch')
ax2.axhline(y=DELTA_THRESHOLD, color='green', linestyle=':', label='Down Threshold')
ax2.axhline(y=-DELTA_THRESHOLD, color='red', linestyle=':', label='Up Threshold')
ax2.set_ylabel('Delta Degrees')
ax2.grid(True)

ax2_twin = ax2.twinx()
ax2_twin.plot(df['Time(s)'], df['Label'], color='purple', alpha=0.6, linewidth=3)
ax2_twin.set_ylabel('Label (0=Stand, 1=Dwn, 2=Up, 3=Hold)', color='purple', fontweight='bold')
ax2_twin.set_yticks([0, 1, 2, 3])
ax2_twin.set_ylim(-0.5, 3.5)

plt.tight_layout()
plt.show()