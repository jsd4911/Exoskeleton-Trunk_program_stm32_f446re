import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import SpanSelector, TextBox, Button
from scipy.signal import butter, filtfilt
import os

# ==========================================
# 參數設定區
# ==========================================
INPUT_FILE = "V7_BNO085_RightBend_20260515_211121.csv"  # 替換成您的檔案
OUTPUT_FILE = "training_data/V7_BNO085_RightBend_Label_20260515_211121.csv" 

FS = 20.0       
CUTOFF = 2.0    
SMOOTH_WINDOW = 10  
SKIP_SECONDS = 1.0  

# ==========================================
# 步驟 A: 讀取與終極防呆預處理
# ==========================================
if not os.path.exists(INPUT_FILE):
    print(f"找不到檔案 {INPUT_FILE}！")
    exit()
df = pd.read_csv(INPUT_FILE)

df = df[df['Time(s)'] > SKIP_SECONDS].reset_index(drop=True)

print("執行極端值過濾與空值修復 (Anti-Glitch)...")
df.replace([np.inf, -np.inf], np.nan, inplace=True) 

for col in ['Roll(deg)', 'Pitch(deg)', 'Yaw(deg)']:
    df[col] = pd.to_numeric(df[col], errors='coerce') 
    df.loc[df[col].abs() > 360, col] = np.nan         
    df[col] = df[col].interpolate(method='linear').ffill().bfill()

print("執行歐拉角邊界連續化 (Unwrap)...")
for col in ['Roll(deg)', 'Pitch(deg)', 'Yaw(deg)']:
    df[col] = np.rad2deg(np.unwrap(np.deg2rad(df[col].values)))

# ==========================================
# 步驟 B: 濾波與自動歸零 (已移除方位偏移邏輯)
# ==========================================
print("執行 3D 濾波與自動歸零...")
nyq = 0.5 * FS
b, a = butter(4, CUTOFF / nyq, btype='low', analog=False)

try:
    df['Roll_Filtered']  = filtfilt(b, a, df['Roll(deg)'])
    df['Pitch_Filtered'] = filtfilt(b, a, df['Pitch(deg)'])
    df['Yaw_Filtered']   = filtfilt(b, a, df['Yaw(deg)'])
except Exception as e:
    print(f"⚠️ 濾波失敗 ({e})，系統將退回使用原始數據！")
    df['Roll_Filtered']  = df['Roll(deg)']
    df['Pitch_Filtered'] = df['Pitch(deg)']
    df['Yaw_Filtered']   = df['Yaw(deg)']

df['Roll_Normalized']  = df['Roll_Filtered'] - df['Roll_Filtered'].head(10).mean()
df['Pitch_Normalized'] = df['Pitch_Filtered'] - df['Pitch_Filtered'].head(10).mean()
df['Yaw_Normalized']   = df['Yaw_Filtered'] - df['Yaw_Filtered'].head(10).mean() # 單純歸零，留作視覺監控用

if not df['Pitch_Normalized'].isna().all():
    if df['Pitch_Normalized'].max() < abs(df['Pitch_Normalized'].min()):
        df['Pitch_Normalized'] = df['Pitch_Normalized'] * -1.0

df['Delta_Pitch_Smoothed'] = df['Pitch_Normalized'].diff().fillna(0).rolling(window=SMOOTH_WINDOW, center=True, min_periods=1).mean()
df['Delta_Roll_Smoothed']  = df['Roll_Normalized'].diff().fillna(0).rolling(window=SMOOTH_WINDOW, center=True, min_periods=1).mean()
df['Delta_Yaw_Smoothed']   = df['Yaw_Normalized'].diff().fillna(0).rolling(window=SMOOTH_WINDOW, center=True, min_periods=1).mean()

# ==========================================
# 步驟 C: 自動化標籤預處理 (移除 L3)
# ==========================================
df['Label1_Move'] = 0
df['Label2_Bend'] = 0
df['Label4_Speed'] = 0

# ==========================================
# 步驟 D: 建立互動式 GUI
# ==========================================
fig = plt.figure(figsize=(15, 9))
plt.subplots_adjust(left=0.05, right=0.75, top=0.95, bottom=0.1)
fig.suptitle('V7.6 Labeling Tool (Compass Removed)', fontsize=16, fontweight='bold')

ax1 = fig.add_subplot(311)
ax1.plot(df['Time(s)'], df['Pitch_Normalized'], 'b-', label='Pitch (Forward)')
ax1.plot(df['Time(s)'], df['Roll_Normalized'], 'g-', label='Roll (Left/Right)')
ax1.plot(df['Time(s)'], df['Yaw_Normalized'], 'purple', alpha=0.3, label='Yaw (Monitor Only)')
ax1.set_ylabel('Angle (deg)')
ax1.legend(loc='upper right')
ax1.grid(True)

ax2 = fig.add_subplot(312, sharex=ax1)
ax2.plot(df['Time(s)'], df['Delta_Pitch_Smoothed'], 'b-', label='d_Pitch')
ax2.plot(df['Time(s)'], df['Delta_Roll_Smoothed'], 'g-', label='d_Roll')
ax2.plot(df['Time(s)'], df['Delta_Yaw_Smoothed'], 'purple', alpha=0.3, label='d_Yaw')
ax2.set_ylabel('Speed (Delta)')
ax2.legend(loc='upper right')
ax2.grid(True)

ax3 = fig.add_subplot(313, sharex=ax1)
l1_line, = ax3.plot(df['Time(s)'], df['Label1_Move'], 'r-', linewidth=2, drawstyle='steps-mid', label='L1: Move')
l2_line, = ax3.plot(df['Time(s)'], df['Label2_Bend'] + 3, 'orange', linewidth=2, drawstyle='steps-mid', label='L2: Bend (+3)')
l4_line, = ax3.plot(df['Time(s)'], df['Label4_Speed'] + 12, 'magenta', linewidth=2, drawstyle='steps-mid', label='L4: Speed (+12)')
ax3.set_ylabel('Labels Status')
ax3.set_xlabel('Time (s)')
ax3.set_ylim(-1, 16) # 縮小圖表範圍讓線條更清晰
ax3.legend(loc='upper right')
ax3.grid(True)

# --- 建立右側 UI 面板 ---
cheat_sheet = """
【Label Guide】
L1 Movement: 
 0=Stat, 1=Move

L2 Forward/Side Bend: 
 0=Standing, 1=Bending Down
 2=Bending Up, 3=Holding
 4=Slightly Bent (微彎)
 5=Left Bend (左側彎)
 6=Right Bend (右側彎)

L4 Movement Speed: 
 0=Slow, 1=Medium, 2=Fast
"""
fig.text(0.77, 0.60, cheat_sheet, fontsize=10, family='monospace', bbox=dict(facecolor='white', alpha=0.8))

ax_box1 = plt.axes([0.83, 0.50, 0.05, 0.04])
text_l1 = TextBox(ax_box1, 'L1(Move): ', initial='0')
ax_box2 = plt.axes([0.83, 0.44, 0.05, 0.04])
text_l2 = TextBox(ax_box2, 'L2(Bend): ', initial='0')
ax_box4 = plt.axes([0.83, 0.38, 0.05, 0.04])
text_l4 = TextBox(ax_box4, 'L4(Spd):  ', initial='0')

ax_btn_apply = plt.axes([0.77, 0.25, 0.15, 0.06])
btn_apply = Button(ax_btn_apply, '1. Apply Labels', color='lightblue')
ax_btn_save = plt.axes([0.77, 0.15, 0.15, 0.06])
btn_save = Button(ax_btn_save, '2. Save CSV', color='lightgreen')

selected_range = [None, None]
def onselect(xmin, xmax):
    selected_range[0] = xmin; selected_range[1] = xmax

span1 = SpanSelector(ax1, onselect, 'horizontal', useblit=True, props=dict(alpha=0.3, facecolor='yellow'))
span2 = SpanSelector(ax2, onselect, 'horizontal', useblit=True, props=dict(alpha=0.3, facecolor='yellow'))

def apply_labels(event):
    if selected_range[0] is not None and selected_range[1] is not None:
        try:
            val_l1 = int(text_l1.text)
            val_l2 = int(text_l2.text)
            val_l4 = int(text_l4.text)
            
            mask = (df['Time(s)'] >= selected_range[0]) & (df['Time(s)'] <= selected_range[1])
            df.loc[mask, 'Label1_Move'] = val_l1
            df.loc[mask, 'Label2_Bend'] = val_l2
            df.loc[mask, 'Label4_Speed'] = val_l4
            
            l1_line.set_ydata(df['Label1_Move'])
            l2_line.set_ydata(df['Label2_Bend'] + 3)
            l4_line.set_ydata(df['Label4_Speed'] + 12)
            fig.canvas.draw_idle()
            print(f"✅ Applied! (L1:{val_l1}, L2:{val_l2}, L4:{val_l4})")
        except ValueError:
            print("❌ Error: Please enter integers!")

btn_apply.on_clicked(apply_labels)

def save_data(event):
    out_dir = os.path.dirname(OUTPUT_FILE)
    if out_dir: os.makedirs(out_dir, exist_ok=True)
    # 🌟 存檔時只保留我們需要的 L1, L2, L4
    final_df = df[['Pitch_Normalized', 'Delta_Pitch_Smoothed', 'Roll_Normalized', 'Delta_Roll_Smoothed', 'Yaw_Normalized', 'Delta_Yaw_Smoothed', 'Label1_Move', 'Label2_Bend', 'Label4_Speed']]
    final_df.to_csv(OUTPUT_FILE, index=False)
    print(f"\n💾 Saved to: {OUTPUT_FILE}")

btn_save.on_clicked(save_data)
plt.show()