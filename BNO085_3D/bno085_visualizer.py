import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation
import os

# ==========================================
# 讀取 BNO085 CSV 資料
# ==========================================
CSV_FILE = "simulation/V7_BNO085_Kalman_20260528_181909.csv"  # 換成您的 CSV 檔名

if not os.path.exists(CSV_FILE):
    print(f"❌ 找不到檔案 {CSV_FILE}")
    exit()

df = pd.read_csv(CSV_FILE)

# 取出必要欄位
t_data = df['Time(s)'].values
roll_data = df['Filtered_Roll'].values
pitch_data = df['Filtered_Pitch'].values
yaw_data = df['Filtered_Yaw'].values

# 計算重播速度
FPS = 20  # BNO085 的採樣率是 20Hz
total_frames = len(t_data)
max_time = t_data[-1]

# ==========================================
# 建立雙視窗介面
# ==========================================
fig = plt.figure(figsize=(14, 6))
fig.suptitle('🤖 BNO085 動作軌跡重播器 (Motion Player)', fontsize=16, fontweight='bold')

# 左側 3D 視圖
ax_3d = fig.add_subplot(1, 2, 1, projection='3d')
ax_3d.set_xlim([-1, 1]); ax_3d.set_ylim([-1, 1]); ax_3d.set_zlim([-1, 1])
ax_3d.set_xlabel('X'); ax_3d.set_ylabel('Y'); ax_3d.set_zlabel('Z')
ax_3d.set_title('3D 即時重播')

# 紅綠藍三軸
line_x, = ax_3d.plot([], [], [], color='purple', linewidth=4, label='Yaw (Up)')
line_y, = ax_3d.plot([], [], [], color='green', linewidth=4, label='Roll (Left)')
line_z, = ax_3d.plot([], [], [], color='blue', linewidth=4, label='Pitch (Forward)')
ax_3d.legend(loc='upper left')

# 右側 2D 歷史波形圖
ax_2d = fig.add_subplot(1, 2, 2)
ax_2d.set_xlim([0, max_time])
# 🌟 讓 Y 軸根據您這次錄製的最高與最低角度自動調整，並上下多留 20 度的顯示空間
min_angle = min(min(roll_data), min(pitch_data), min(yaw_data)) - 20
max_angle = max(max(roll_data), max(pitch_data), max(yaw_data)) + 20
ax_2d.set_ylim([min_angle, max_angle])
ax_2d.set_xlabel('Time (s)'); ax_2d.set_ylabel('Angle (deg)')
ax_2d.set_title('感測器原始數據')
ax_2d.grid(True)

line_roll, = ax_2d.plot([], [], 'g-', label='Roll')
line_pitch, = ax_2d.plot([], [], 'b-', label='Pitch')
line_yaw, = ax_2d.plot([], [], 'purple', label='Yaw')

# 加上一條紅色的垂直線，指示目前重播到哪裡
time_cursor = ax_2d.axvline(x=0, color='r', linestyle='--', alpha=0.7)
ax_2d.legend(loc='upper right')

# ==========================================
# 動畫更新邏輯
# ==========================================
def update(frame):
    current_t = t_data[frame]
    r = roll_data[frame]
    p = pitch_data[frame]
    y = yaw_data[frame]
    
    # 處理防呆：如果遇到 NaN (例如未濾除的突波)，就跳過這幀的畫面更新
    if np.isnan(r) or np.isnan(p) or np.isnan(y):
        return line_x, line_y, line_z, line_roll, line_pitch, line_yaw, time_cursor
    
    # 1. 更新 3D 旋轉 (Z-Y-X 順序通常對應 Yaw-Pitch-Roll)
    rot = Rotation.from_euler('zyx', [y, p, r], degrees=True)
    
    base_x = np.array([1, 0, 0])
    base_y = np.array([0, 1, 0])
    base_z = np.array([0, 0, 1])
    
    rot_x = rot.apply(base_x)
    rot_y = rot.apply(base_y)
    rot_z = rot.apply(base_z)
    
    line_x.set_data_3d([0, rot_x[0]], [0, rot_x[1]], [0, rot_x[2]])
    line_y.set_data_3d([0, rot_y[0]], [0, rot_y[1]], [0, rot_y[2]])
    line_z.set_data_3d([0, rot_z[0]], [0, rot_z[1]], [0, rot_z[2]])
    
    # 2. 更新 2D 歷史波形與進度條
    line_roll.set_data(t_data[:frame], roll_data[:frame])
    line_pitch.set_data(t_data[:frame], pitch_data[:frame])
    line_yaw.set_data(t_data[:frame], yaw_data[:frame])
    time_cursor.set_xdata([current_t, current_t])
    
    return line_x, line_y, line_z, line_roll, line_pitch, line_yaw, time_cursor

print("▶️ 開始重播動作...")
ani = FuncAnimation(fig, update, frames=total_frames, interval=1000/FPS, blit=False)
plt.show()