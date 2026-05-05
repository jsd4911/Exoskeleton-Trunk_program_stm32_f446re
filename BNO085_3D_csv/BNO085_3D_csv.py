import serial
import time
import csv

# --- 設定通訊埠 (請修改為您的 COM Port) ---
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200

# 設定輸出的檔案名稱 (可自行修改，加上時間戳記避免覆蓋)
filename = time.strftime("BNO085_data_%Y%m%d_%H%M%S.csv")

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"成功連接 {SERIAL_PORT}")
except Exception as e:
    print(f"無法打開通訊埠: {e}")
    exit()

print(f"開始記錄資料至 {filename}...")
print("請隨時按 Ctrl+C 停止記錄。")

# 打開一個 CSV 檔案準備寫入
with open(filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    
    # 寫入第一行 (標題列)
    writer.writerow(['Time(s)', 'Roll(deg)', 'Pitch(deg)', 'Yaw(deg)'])
    
    start_time = time.time()
    
    try:
        while True:
            if ser.in_waiting > 0:
                # 讀取一行 STM32 傳來的數據 (例如 "30.5,10.2,45.0\r\n")
                line_data = ser.readline().decode('utf-8').strip()
                
                try:
                    # 用逗號切開字串，轉換成浮點數
                    r, p, y = map(float, line_data.split(','))
                    
                    # 計算經過的時間
                    current_time = time.time() - start_time
                    
                    # 寫入一行資料到 CSV
                    writer.writerow([f"{current_time:.3f}", r, p, y])
                    
                    # 同時印在畫面上讓您確認有在跑
                    print(f"記錄: {current_time:.3f}s -> {r}, {p}, {y}")
                    
                except ValueError:
                    # 如果收到剛開機的文字或垃圾封包，就跳過這行
                    pass

    except KeyboardInterrupt:
        # 當使用者按下 Ctrl+C 時觸發
        print("\n收到停止指令，正在儲存檔案並關閉通訊埠...")

# 關閉通訊埠
ser.close()
print("記錄完成！檔案已儲存。")