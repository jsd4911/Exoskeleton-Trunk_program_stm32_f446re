import serial
import csv
import time
from datetime import datetime
import os

# =================設定區=================
SERIAL_PORT = 'COM3'  # 請確認您的 COM Port 是否正確
BAUD_RATE = 115200

# 🌟 虛擬阻抗觀測器 (Virtual Impedance Observer) 參數
SHAKE_THRESHOLD = 5.0  # 單幀角度變化超過 5 度視為高頻晃動突波
filtered_r, filtered_p, filtered_y = None, None, None

# ========================================

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"成功連接 {SERIAL_PORT}")
except Exception as e:
    print(f"無法打開 {SERIAL_PORT}: {e}")
    exit()

# 準備輸出檔名
timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
filename = f"V7-1_BNO085_Move_{timestamp_str}.csv"

print("開始錄製，按 Ctrl+C 停止...")

last_r, last_p, last_y = None, None, None

# 打開一個 CSV 檔案準備寫入
with open(filename, mode='w', newline='') as file:
    writer = csv.writer(file)

    # 寫入標題列 (新增了濾波後的欄位，方便您事後對比！)
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
                    # 1. 讀取原始數據 (STM32 傳來 "30.5,10.2,45.0" 格式)
                    # 🌟 核心修改：直接用逗號切開字串
                    parts = line_data.split(',')
                    
                    if len(parts) == 3:
                        r = float(parts[0])  # 第一個數字是 Roll
                        p = float(parts[1])  # 第二個數字是 Pitch
                        y = float(parts[2])  # 第三個數字是 Yaw
                    else:
                        # 如果不是剛好 3 個數字 (可能有雜訊或傳輸錯誤)，就跳過這筆
                        continue 

                    # 2. 🌟 虛擬阻抗濾波 (Adaptive EMA)
                    if filtered_r is None:
                        # 初始化第一筆資料
                        filtered_r, filtered_p, filtered_y = r, p, y
                        last_r, last_p, last_y = r, p, y
                        dr, dp, dy = 0.0, 0.0, 0.0
                    else:
                        # 計算單幀絕對變化量 (角速度)
                        delta_r_abs = abs(r - filtered_r)
                        delta_p_abs = abs(p - filtered_p)
                        delta_y_abs = abs(y - filtered_y)

                        # 預設信任因子 (低阻抗，緊密跟隨)
                        alpha_r, alpha_p, alpha_y = 0.8, 0.8, 0.8

                        # 啟動擾動觀測器防禦 (高阻抗，拒絕吸收雜訊)
                        if delta_r_abs > SHAKE_THRESHOLD: alpha_r = 0.05
                        if delta_p_abs > SHAKE_THRESHOLD: alpha_p = 0.05
                        if delta_y_abs > SHAKE_THRESHOLD: alpha_y = 0.05

                        # 套用阻抗方程式
                        filtered_r = alpha_r * r + (1 - alpha_r) * filtered_r
                        filtered_p = alpha_p * p + (1 - alpha_p) * filtered_p
                        filtered_y = alpha_y * y + (1 - alpha_y) * filtered_y

                        # 3. 計算 Delta 變化量 (基於濾波後的平滑資料)
                        dr = filtered_r - last_r
                        dp = filtered_p - last_p
                        dy = filtered_y - last_y

                        last_r, last_p, last_y = filtered_r, filtered_p, filtered_y

                    current_time = time.time() - start_time

                    # 4. 寫入資料
                    writer.writerow([f"{current_time:.3f}", 
                                     r, p, y,  # 原始有雜訊的數據
                                     round(filtered_r, 4), round(filtered_p, 4), round(filtered_y, 4), # 抗晃動後的數據
                                     round(dr, 4), round(dp, 4), round(dy, 4)])

                    # 在終端機印出，讓您即時監看抗晃動效果
                    print(f"[{current_time:.1f}s] Raw_P:{p:5.1f} -> Filtered_P:{filtered_p:5.1f} | dP:{dp:5.2f}")

                except ValueError:
                    # 略過無法轉換為浮點數的行
                    pass
    except KeyboardInterrupt:
        print("\n錄製結束！")
    finally:
        ser.close()
        print(f"資料已儲存至 {filename}")