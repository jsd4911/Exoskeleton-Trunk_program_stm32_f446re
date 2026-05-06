import pandas as pd
import matplotlib.pyplot as plt

# --- 1. 載入數據 ---
# (建議載入您在上一階段做過「低通濾波」的乾淨數據，這裡先用您的截圖檔名示範)
filename = "BNO085_data_20260504_154721.csv" 
df = pd.read_csv(filename)

# --- 2. 建立標籤欄位 (Label) ---
# 先預設所有的時間點，動作標籤都是 0 (站直)
df['Label'] = 0

# --- 3. 自動標註邏輯 (Rule-based Labeling) ---
# 利用我們觀察到的物理特性：只要 Pitch 小於 -45 度，就把該列的 Label 改為 1 (彎腰)
# (您可以根據濾波後的數據，微調 -45 這個數字)
bend_threshold = -45.0
df.loc[df['Pitch(deg)'] > bend_threshold, 'Label'] = 1

# --- 4. 驗證與視覺化 ---
# 畫圖來檢查我們自動貼的標籤對不對
fig, ax1 = plt.subplots(figsize=(10, 5))

# 畫出 Pitch 角度曲線
ax1.plot(df['Time(s)'], df['Pitch(deg)'], color='orange', label='Pitch Angle')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Pitch (deg)', color='orange')
ax1.tick_params(axis='y', labelcolor='orange')

# 在同一張圖上，畫出 Label 的方波圖 (0 或 1)
ax2 = ax1.twinx()  # 建立共用 X 軸的第二個 Y 軸
ax2.plot(df['Time(s)'], df['Label'], color='red', alpha=0.5, label='Label (0=Stand, 1=Bend)')
ax2.set_ylabel('Label Action', color='red')
ax2.set_yticks([0, 1])
ax2.tick_params(axis='y', labelcolor='red')

plt.title('Auto-Labeling Verification')
fig.tight_layout()
plt.show()

# --- 5. 存出帶有標籤的最終資料集 ---
# 這份 CSV 就是未來要餵給 PyTorch 吃的黃金訓練資料！
output_filename = "BNO085_Labeled_Dataset.csv"
df.to_csv(output_filename, index=False)
print(f"標註完成！已儲存為 {output_filename}")