import serial
import csv
import time
from datetime import datetime
import os

# =================設定區=================
SERIAL_PORT = 'COM3'  # 請確認您的 COM Port 是否正確
BAUD_RATE = 115200

# 🌟 晃動觸發閾值 (超過此變化量，啟動高阻尼抗震)
SHAKE_THRESHOLD = 5.0  

# ========================================
# 🌟 核心升級：1D 動態卡爾曼濾波器 (1D Dynamic Kalman Filter)
# ========================================
class KalmanFilter1D:
    def __init__(self, process_noise=0.01, measurement_noise=0.1):
        # Q: 過程雜訊 (越小越相信物理慣性，模型較硬)
        self.q = process_noise  
        # R: 測量雜訊 (正常狀態下對 BNO085 讀數的不信任度)
        self.r = measurement_noise 
        self.x = 0.0 # 狀態估計值 (當前角度)
        self.p = 1.0 # 估計誤差共變異數
        self.k = 0.0 # 卡爾曼增益
        self.initialized = False

    def update(self, measurement, dynamic_r=None):
        if not self.initialized:
            self.x = measurement
            self.initialized = True
            return self.x

        # 決定當下要用正常的 R，還是受到衝擊時的高阻抗 R
        current_r = dynamic_r if dynamic_r is not None else self.r

        # 1. 預測 (Predict): 假設維持前一刻狀態 (在 1D 中，先驗估計等於前一次後驗)
        self.p = self.p + self.q

        # 2. 更新 (Update): 計算卡爾曼增益，並融合測量值
        self.k = self.p / (self.p + current_r)
        self.x = self.x + self.k * (measurement - self.x)
        self.p = (1 - self.k) * self.p

        return self.x

# 為 R, P, Y 三個軸分別實體化專屬的卡爾曼濾波器
# 預設狀態：過程雜訊(Q)設小，測量雜訊(R)適中，讓正常動作滑順跟隨
kf_roll = KalmanFilter1D(process_noise=0.005, measurement_noise=0.5)
kf_pitch = KalmanFilter1D(process_noise=0.005, measurement_noise=0.5)
kf_yaw = KalmanFilter1D(process_noise=0.005, measurement_noise=0.5)

# 記錄解折疊後的連續角度
unwrapped_r, unwrapped_p, unwrapped_y = None, None, None 

# 記錄上一幀的濾波結果，用來算 Delta
last_r, last_p, last_y = None, None, None

# ========================================
# 🌟 相角解折疊函數 (Unwrap) - 完美解決 +-180 突波
# ========================================
def unwrap(curr, prev):
    diff = (curr - prev + 180) % 360 - 180
    return prev + diff

# ========================================

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"成功連接 {SERIAL_PORT}")
except Exception as e:
    print(f"無法打開 {SERIAL_PORT}: {e}")
    exit()

timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
filename = f"V8_BNO085_Stoop_{timestamp_str}.csv"

print("開始錄製，按 Ctrl+C 停止...")

with open(filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Time(s)', 
                     'Raw_Roll', 'Raw_Pitch', 'Raw_Yaw', 
                     'Filtered_Roll', 'Filtered_Pitch', 'Filtered_Yaw', 
                     'Delta_Roll', 'Delta_Pitch', 'Delta_Yaw'])

    start_time = time.time()

    try:
        while True:
            if ser.in_waiting > 0:
                line_data = ser.readline().decode('utf-8', errors='ignore').strip()

                try:
                    parts = line_data.split(',')
                    
                    if len(parts) == 3:
                        r_raw = float(parts[0])  
                        p_raw = float(parts[1])  
                        y_raw = float(parts[2])  
                    else:
                        continue 

                    # 1. 🌟 解除相角折疊 (Unwrap) 與 計算單幀真實角速度
                    if unwrapped_r is None:
                        unwrapped_r, unwrapped_p, unwrapped_y = r_raw, p_raw, y_raw
                        # 初始化卡爾曼濾波器
                        filtered_r = kf_roll.update(r_raw)
                        filtered_p = kf_pitch.update(p_raw)
                        filtered_y = kf_yaw.update(y_raw)
                        last_r, last_p, last_y = r_raw, p_raw, y_raw
                        dr, dp, dy = 0.0, 0.0, 0.0
                        r, p, y = r_raw, p_raw, y_raw
                    else:
                        # 將有跳變的原始資料，轉換為平滑連續的角度
                        r = unwrap(r_raw, unwrapped_r)
                        p = unwrap(p_raw, unwrapped_p)
                        y = unwrap(y_raw, unwrapped_y)
                        
                        # 🌟 修正核心：計算真正的「單幀物理變化量」(無延遲)
                        delta_raw_r = abs(r - unwrapped_r)
                        delta_raw_p = abs(p - unwrapped_p)
                        delta_raw_y = abs(y - unwrapped_y)

                        # 更新記憶，供下一幀比對
                        unwrapped_r, unwrapped_p, unwrapped_y = r, p, y

                        # 2. 🌟 動態卡爾曼濾波 (Dynamic Kalman Filter)
                        # 🌟 修正核心：用 delta_raw (無延遲的真實角速度) 取代原本的誤差比較
                        dynamic_r_roll = 50.0 if delta_raw_r > SHAKE_THRESHOLD else kf_roll.r
                        dynamic_r_pitch = 50.0 if delta_raw_p > SHAKE_THRESHOLD else kf_pitch.r
                        dynamic_r_yaw = 50.0 if delta_raw_y > SHAKE_THRESHOLD else kf_yaw.r

                        # 將數據送入卡爾曼濾波器進行 Predict & Update
                        filtered_r = kf_roll.update(r, dynamic_r_roll)
                        filtered_p = kf_pitch.update(p, dynamic_r_pitch)
                        filtered_y = kf_yaw.update(y, dynamic_r_yaw)

                    # 3. 計算最終的 Delta 變化量
                    dr = filtered_r - last_r
                    dp = filtered_p - last_p
                    dy = filtered_y - last_y

                    last_r, last_p, last_y = filtered_r, filtered_p, filtered_y
                    current_time = time.time() - start_time

                    # 4. 寫入資料
                    writer.writerow([f"{current_time:.3f}", 
                                     round(r, 4), round(p, 4), round(y, 4),  
                                     round(filtered_r, 4), round(filtered_p, 4), round(filtered_y, 4), 
                                     round(dr, 4), round(dp, 4), round(dy, 4)])

                    # 在終端機印出即時對比，並顯示當下的卡爾曼增益 K
                    # K 越小，代表系統越處於「高阻尼/抗震」狀態
                    print(f"[{current_time:.1f}s] Pitch: Raw {p:5.1f} -> Kalman {filtered_p:5.1f} | K:{kf_pitch.k:.3f}")

                except ValueError:
                    pass
    except KeyboardInterrupt:
        print("\n錄製結束！")
    finally:
        ser.close()
        print(f"資料已儲存至 {filename}")