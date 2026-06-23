import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # 用於繪製 3D 軌跡

# ==========================================
# 1. 參數設定區
# ==========================================
CSV_FILE = 'simulation/V8_BNO085_Stoop_20260530_165715.csv' # 請確認檔名

# 🌟 手動標註時間點 (秒)
TIME_START = 3.679   # 起點時間
TIME_END = 5.177     # 終點時間
NUM_POINTS = 100 

# ==========================================
# 2. 核心函數：五次多項式軌跡生成
# ==========================================
def generate_quintic_trajectory(theta_start, theta_end, tf, num_points):
    c0 = theta_start
    c1, c2 = 0.0, 0.0
    c3 = 10.0 * (theta_end - theta_start) / (tf ** 3)
    c4 = -15.0 * (theta_end - theta_start) / (tf ** 4)
    c5 = 6.0 * (theta_end - theta_start) / (tf ** 5)
    
    t = np.linspace(0, tf, num_points)
    position = c0 + c1*t + c2*(t**2) + c3*(t**3) + c4*(t**4) + c5*(t**5)
    velocity = c1 + 2*c2*t + 3*c3*(t**2) + 4*c4*(t**3) + 5*c5*(t**4)
    acceleration = 2*c2 + 6*c3*t + 12*c4*(t**2) + 20*c5*(t**3)
    
    return t, position, velocity, acceleration

# ==========================================
# 3. 讀取 CSV 並抓取【三個軸】的邊界條件
# ==========================================
try:
    df = pd.read_csv(CSV_FILE)
except FileNotFoundError:
    print(f"❌ 找不到檔案：{CSV_FILE}")
    exit()

t_data = df['Time(s)'].astype(float).values
idx_start = np.abs(t_data - TIME_START).argmin()
idx_end = np.abs(t_data - TIME_END).argmin()
tf = t_data[idx_end] - t_data[idx_start]

# 萃取 3 軸起始與終點
start_angles = {
    'Roll': df['Filtered_Roll'].iloc[idx_start],
    'Pitch': df['Filtered_Pitch'].iloc[idx_start],
    'Yaw': df['Filtered_Yaw'].iloc[idx_start]
}
end_angles = {
    'Roll': df['Filtered_Roll'].iloc[idx_end],
    'Pitch': df['Filtered_Pitch'].iloc[idx_end],
    'Yaw': df['Filtered_Yaw'].iloc[idx_end]
}

print(f"--- 🎯 3D 邊界條件 (tf = {tf:.2f}s) ---")
for axis in ['Roll', 'Pitch', 'Yaw']:
    print(f"{axis:5s}: {start_angles[axis]:8.2f}° -> {end_angles[axis]:8.2f}°")

# ==========================================
# 4. 分別計算 3 軸的多項式 (時間 tf 是共用的！)
# ==========================================
traj = {}
for axis in ['Roll', 'Pitch', 'Yaw']:
    t_q, pos, vel, acc = generate_quintic_trajectory(
        start_angles[axis], end_angles[axis], tf, NUM_POINTS
    )
    traj[axis] = {'pos': pos, 'vel': vel, 'acc': acc}

# ==========================================
# 5. 繪製 3D 軌跡圖表
# ==========================================
fig = plt.figure(figsize=(15, 10))
fig.suptitle('3D Multi-Axis Trajectory Generation', fontsize=16)

# (圖 1) 角度位置 vs 時間
ax1 = plt.subplot(2, 2, 1)
ax1.plot(t_q, traj['Roll']['pos'], 'r-', linewidth=2, label='Roll')
ax1.plot(t_q, traj['Pitch']['pos'], 'g-', linewidth=2, label='Pitch')
ax1.plot(t_q, traj['Yaw']['pos'], 'b-', linewidth=2, label='Yaw')
ax1.set_title('Position Trajectories')
ax1.set_ylabel('Degrees')
ax1.grid(True); ax1.legend()

# (圖 2) 角速度 vs 時間 (觀察鐘型曲線如何同步縮放)
ax2 = plt.subplot(2, 2, 2)
ax2.plot(t_q, traj['Roll']['vel'], 'r--', linewidth=2, label='Roll Vel')
ax2.plot(t_q, traj['Pitch']['vel'], 'g--', linewidth=2, label='Pitch Vel')
ax2.plot(t_q, traj['Yaw']['vel'], 'b--', linewidth=2, label='Yaw Vel')
ax2.set_title('Velocity Profiles (Synchronized)')
ax2.set_ylabel('Degrees / sec')
ax2.grid(True); ax2.legend()

# (圖 3) 3D 空間姿態軌跡圖 (Euler Space)
ax3 = plt.subplot(2, 1, 2, projection='3d')
# 畫出感測器原本的雜訊路徑 (細線)
ax3.plot(df['Filtered_Roll'].iloc[idx_start:idx_end], 
         df['Filtered_Pitch'].iloc[idx_start:idx_end], 
         df['Filtered_Yaw'].iloc[idx_start:idx_end], 
         color='gray', alpha=0.5, label='Actual Human Path')
# 畫出五次多項式產生的平滑 3D 曲線 (粗線)
ax3.plot(traj['Roll']['pos'], traj['Pitch']['pos'], traj['Yaw']['pos'], 
         color='blue', linewidth=3, label='Quintic Smooth Path')

# 標記起點與終點
ax3.scatter(start_angles['Roll'], start_angles['Pitch'], start_angles['Yaw'], color='green', s=100, label='Start')
ax3.scatter(end_angles['Roll'], end_angles['Pitch'], end_angles['Yaw'], color='red', s=100, label='End')

ax3.set_title('3D Posture Trajectory (Roll-Pitch-Yaw Space)')
ax3.set_xlabel('Roll (X)')
ax3.set_ylabel('Pitch (Y)')
ax3.set_zlabel('Yaw (Z)')
ax3.legend()

plt.tight_layout()
plt.show()

# 模擬匯出 STM32 控制格式
print("\n--- 🤖 匯出為 STM32 3D 座標控制陣列 (前 3 點) ---")
for i in range(3):
    print(f"Goal_Vel_Point_{i}: [Roll: {traj['Roll']['vel'][i]:.2f}, Pitch: {traj['Pitch']['vel'][i]:.2f}, Yaw: {traj['Yaw']['vel'][i]:.2f}]")