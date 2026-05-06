import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
import os

# ==========================================
# 參數設定區 (您只需要改這裡就好！)
# ==========================================
INPUT_FILE = "BNO085_data_20260506_102638.csv"  # 您剛剛存下的原始 CSV 檔名
OUTPUT_FILE = "BNO085_ReadyForML_Dataset2.csv" # 處理完要存出的檔名

# 濾波參數
FS = 20.0       # 感測器頻率 20Hz
CUTOFF = 2.0    # 低通截止頻率 2Hz

# 標註參數
BEND_THRESHOLD = -45.0  # Pitch 小於此度數，標記為彎腰 (Label = 1)

# ==========================================
# 步驟 A: 讀取原始數據
# ==========================================
if not os.path.exists(INPUT_FILE):
    print(f"找不到檔案 {INPUT_FILE}，請確認檔名是否正確！")
    exit()
    
print(f"正在處理: {INPUT_FILE} ...")
df = pd.read_csv(INPUT_FILE)

# 確保必要欄位存在
required_cols = ['Time(s)', 'Roll(deg)', 'Pitch(deg)', 'Yaw(deg)']
for col in required_cols:
    if col not in df.columns:
        print(f"錯誤：CSV 缺少欄位 '{col}'")
        exit()

# ==========================================
# 步驟 B: 數位濾波 (巴特沃斯低通濾波)
# ==========================================
print("執行零相位低通濾波...")
nyq = 0.5 * FS
normal_cutoff = CUTOFF / nyq
b, a = butter(4, normal_cutoff, btype='low', analog=False)

# 為三軸都加上濾波後的新欄位
df['Roll_Filtered']  = filtfilt(b, a, df['Roll(deg)'])
df['Pitch_Filtered'] = filtfilt(b, a, df['Pitch(deg)'])
df['Yaw_Filtered']   = filtfilt(b, a, df['Yaw(deg)'])

# ==========================================
# 步驟 C: 自動標註 (Labeling)
# ==========================================
print("執行自動標註 (Labeling)...")
df['Label'] = 0  # 預設為 0 (站直)

# 注意！我們這裡是拿「濾波過」的乾淨 Pitch 來做標註判斷，會比原始數據準確很多！
df.loc[df['Pitch_Filtered'] > BEND_THRESHOLD, 'Label'] = 1

# ==========================================
# 步驟 D: 儲存最終訓練集
# ==========================================
# 為了讓 PyTorch 訓練更乾淨，我們只保留需要餵給模型的特徵和標籤
final_df = df[['Time(s)', 'Roll_Filtered', 'Pitch_Filtered', 'Yaw_Filtered', 'Label']]
final_df.to_csv(OUTPUT_FILE, index=False)
print(f"✅ 處理完成！完美數據已儲存為: {OUTPUT_FILE}")

# ==========================================
# 步驟 E: 視覺化總結報告
# ==========================================
print("繪製總結圖表...")
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
fig.suptitle('Data Preprocessing Pipeline Summary', fontsize=16)

# 上圖：濾波前後比對 (以 Pitch 為例)
ax1.plot(df['Time(s)'], df['Pitch(deg)'], color='lightgray', label='Raw Pitch', linewidth=1)
ax1.plot(df['Time(s)'], df['Pitch_Filtered'], color='orange', label='Filtered Pitch', linewidth=2)
ax1.axhline(y=BEND_THRESHOLD, color='red', linestyle='--', alpha=0.5, label='Bend Threshold')
ax1.set_ylabel('Degrees')
ax1.legend(loc='upper right')
ax1.grid(True, linestyle='--', alpha=0.6)
ax1.set_title('Step 1 & 2: Raw vs Filtered Data')

# 下圖：純淨數據與標籤對應
ax2.plot(final_df['Time(s)'], final_df['Pitch_Filtered'], color='orange', label='Filtered Pitch')
ax2.set_ylabel('Degrees')
ax2.grid(True, linestyle='--', alpha=0.6)

# 建立共用 X 軸的第二個 Y 軸，畫紅色的 0/1 標籤方波
ax2_twin = ax2.twinx()
ax2_twin.plot(final_df['Time(s)'], final_df['Label'], color='red', alpha=0.5, linewidth=2, label='Action Label')
ax2_twin.set_ylabel('Label (0=Stand, 1=Bend)', color='red')
ax2_twin.set_yticks([0, 1])

# 合併下圖的圖例
lines, labels = ax2.get_legend_handles_labels()
lines2, labels2 = ax2_twin.get_legend_handles_labels()
ax2_twin.legend(lines + lines2, labels + labels2, loc='upper right')
ax2.set_title('Step 3: Auto-Labeling Result')
ax2.set_xlabel('Time (seconds)')

plt.tight_layout()
plt.show()