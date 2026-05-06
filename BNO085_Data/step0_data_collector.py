import serial
import time
import csv
import os
from datetime import datetime

# ==========================================
# 參數設定
# ==========================================
COM_PORT = 'COM3'      # ⚠️ 請改成您 STM32 連接的 COM Port 號碼
BAUD_RATE = 115200     # 必須與 STM32 設定一致
DATA_FOLDER = 'dataset' # 存放 CSV 的資料夾

# 確保資料夾存在
if not os.path.exists(DATA_FOLDER):
    os.makedirs(DATA_FOLDER)

# ==========================================
# 選擇要錄製的動作類型
# ==========================================
print("========================================")
print("🤖 外骨骼動作數據採集系統啟動！")
print("========================================")
print("請選擇您現在要錄製的動作：")
print("1: 慢慢彎腰 (Slow Bend)")
print("2: 快速彎腰 (Fast Bend)")
print("3: 純走路不彎腰 (Walking)")
print("4: 靜止站立 (Idle)")
print("5: 自訂動作 (Custom)")

choice = input("👉 請輸入數字選項 (1-5): ")

action_dict = {'1': 'slow_bend', '2': 'fast_bend', '3': 'walking', '4': 'idle', '5': 'custom'}
action_name = action_dict.get(choice, 'custom')

if action_name == 'custom':
    action_name = input("請輸入自訂動作名稱 (英文): ")

# 產生檔名 (加入時間戳記避免覆蓋)
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
filename = f"{DATA_FOLDER}/data_{action_name}_{timestamp}.csv"

# ==========================================
# 開始連線並錄製
# ==========================================
try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    print(f"\n✅ 成功連接 {COM_PORT}！")
    print(f"📁 數據將儲存至: {filename}")
    print("🔴 開始錄製！請開始做動作... (隨時按下 Ctrl+C 可以停止錄製)")
    
    # 開啟 CSV 檔案並寫入標題
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Time(s)', 'Roll(deg)', 'Pitch(deg)', 'Yaw(deg)'])
        
        start_time = time.time()
        
        while True:
            # 讀取一行 STM32 傳來的資料 (例如 "P:-50.2,R:5.0,Y:10.0\n")
            line = ser.readline().decode('utf-8').strip()
            
            # 確保讀到的是正確的格式才處理
            if line.startswith("P:"):
                try:
                    # 字串切割解析
                    parts = line.split(',')
                    pitch = float(parts[0].split(':')[1])
                    roll  = float(parts[1].split(':')[1])
                    yaw   = float(parts[2].split(':')[1])
                    
                    # 計算經過的時間
                    current_time = time.time() - start_time
                    
                    # 寫入 CSV
                    writer.writerow([round(current_time, 3), roll, pitch, yaw])
                    
                    # 在螢幕上顯示讓您安心
                    print(f"Time: {current_time:.2f}s | Pitch: {pitch:.1f} | 寫入中...", end='\r')
                    
                except Exception as e:
                    pass # 忽略解析錯誤的雜訊

# 當您按下 Ctrl+C 停止程式時，會安全地關閉檔案
except KeyboardInterrupt:
    print("\n\n⏹️ 錄製結束！")
    print(f"✅ 檔案已成功存檔於 {filename}")
except serial.SerialException:
    print(f"\n❌ 無法開啟 {COM_PORT}，請檢查線有沒有接好，或是 Tera Term 是否佔用了連接埠！")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()