import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ==========================================
# 1. 參數設定區 (請在這裡設定您的檔案與標註點)
# ==========================================
CSV_FILE = 'simulation/V8_BNO085_Stoop_20260530_165715.csv' # 請換成您想分析的 CSV 檔名

# 🌟 手動標註時間點 (秒)
# 觀察您的 CSV 或圖表，找出「開始彎腰」和「彎腰到底」的時間點
TIME_STAND_START = 2.98  # 剛開始要彎腰的瞬間 (例如第 2.0 秒)
TIME_BEND_END = 4.627     # 彎腰動作結束，停留在底部的瞬間 (例如第 4.5 秒)

# 軌跡解析度 (要產生幾個控制點)
NUM_POINTS = 100 

# ==========================================
# 2. 核心函數：五次多項式軌跡生成
# ==========================================
def generate_quintic_trajectory(theta_start, theta_end, tf, num_points):
    c0 = theta_start
    c1 = 0.0
    c2 = 0.0
    c3 = 10.0 * (theta_end - theta_start) / (tf ** 3)
    c4 = -15.0 * (theta_end - theta_start) / (tf ** 4)
    c5 = 6.0 * (theta_end - theta_start) / (tf ** 5)
    
    t = np.linspace(0, tf, num_points)
    position = c0 + c1*t + c2*(t**2) + c3*(t**3) + c4*(t**4) + c5*(t**5)
    velocity = c1 + 2*c2*t + 3*c3*(t**2) + 4*c4*(t**3) + 5*c5*(t**4)
    acceleration = 2*c2 + 6*c3*t + 12*c4*(t**2) + 20*c5*(t**3)
    
    return t, position, velocity, acceleration

# ==========================================
# 3. 讀取 CSV 並抓取邊界條件
# ==========================================
try:
    df = pd.read_csv(CSV_FILE)
except FileNotFoundError:
    print(f"❌ 找不到檔案：{CSV_FILE}")
    exit()

# 確保時間欄位存在且轉為浮點數
t_data = df['Time(s)'].astype(float).values
pitch_data = df['Filtered_Pitch'].astype(float).values

# 尋找最接近您標註時間的資料點索引 (Index)
idx_start = np.abs(t_data - TIME_STAND_START).argmin()
idx_end = np.abs(t_data - TIME_BEND_END).argmin()

# 萃取邊界條件角度
theta_start = pitch_data[idx_start]
theta_end = pitch_data[idx_end]
tf = t_data[idx_end] - t_data[idx_start]

print("--- 🎯 成功萃取邊界條件 ---")
print(f"站立起點 (t={t_data[idx_start]:.2f}s): {theta_start:.2f} 度")
print(f"彎腰終點 (t={t_data[idx_end]:.2f}s): {theta_end:.2f} 度")
print(f"動作總時長 (tf): {tf:.2f} 秒")

# ==========================================
# 4. 運算五次多項式
# ==========================================
t_quintic, pos_quintic, vel_quintic, acc_quintic = generate_quintic_trajectory(
    theta_start, theta_end, tf, NUM_POINTS
)

# 模擬生成 C 語言陣列 (供 STM32 / Dynamixel 使用)
print("\n--- 🤖 匯出為 STM32 控制陣列 (前 5 點) ---")
print("float goal_velocity[] = {", end="")
for v in vel_quintic[:5]:
    print(f"{v:.2f}, ", end="")
print("...};")

# ==========================================
# 5. 繪製對比圖表
# ==========================================
plt.figure(figsize=(12, 8))
plt.suptitle('Human Motion vs. Minimum Jerk Trajectory', fontsize=16)

# (上) BNO085 實際量測的 Pitch 軌跡與標記點
plt.subplot(2, 1, 1)
plt.plot(t_data, pitch_data, 'gray', label='Sensor Data (Kalman Filtered)')
plt.plot(t_data[idx_start:idx_end+1], pitch_data[idx_start:idx_end+1], 'b-', linewidth=3, label='Segment to Model')
plt.plot([t_data[idx_start], t_data[idx_end]], [theta_start, theta_end], 'ro', markersize=8, label='Boundary Constraints')
plt.title('Step 1: Extract Boundary Conditions from BNO085')
plt.ylabel('Pitch (deg)')
plt.xlim([max(0, TIME_STAND_START - 2), TIME_BEND_END + 2])
plt.legend()
plt.grid(True)

# (下) 五次多項式生成的平滑目標位移與速度
ax1 = plt.subplot(2, 1, 2)
# 將多項式的時間軸平移，以便與上圖對齊
t_shifted = t_quintic + t_data[idx_start] 
line1, = ax1.plot(t_shifted, pos_quintic, 'b-', linewidth=3, label='Generated Position (deg)')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Position (deg)', color='b')
ax1.tick_params(axis='y', labelcolor='b')
ax1.grid(True)

# 在同一個圖表右側 Y 軸畫出馬達角速度命令
ax2 = ax1.twinx()
line2, = ax2.plot(t_shifted, vel_quintic, 'g--', linewidth=2, label='Motor Velocity Cmd (deg/s)')
ax2.set_ylabel('Velocity (deg/s)', color='g')
ax2.tick_params(axis='y', labelcolor='g')

plt.title('Step 2: Generate Motor Commands via Quintic Polynomial')
plt.xlim([max(0, TIME_STAND_START - 2), TIME_BEND_END + 2])
plt.show()