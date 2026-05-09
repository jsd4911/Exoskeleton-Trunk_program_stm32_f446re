import serial
import time
import csv

# --- 設定通訊埠 (請修改為您的 COM Port) ---
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200

# 設定輸出的檔案名稱 (自動加上 Delta 標記與時間)
filename = time.strftime("BNO085_DeltaData_%Y%m%d_%H%M%S.csv")

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"成功連接 {SERIAL_PORT}")
except Exception as e:
    print(f"無法打開通訊埠: {e}")
    exit()

print(f"開始記錄資料至 {filename}...")
print("請隨時按 Ctrl+C 停止記錄。")

# 用來記憶上一筆資料的變數
last_r = None
last_p = None
last_y = None

# 打開一個 CSV 檔案準備寫入
with open(filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    
    # 寫入標題列 (純淨版：只記錄時間、絕對角度、變化量)
    writer.writerow(['Time(s)', 'Roll(deg)', 'Pitch(deg)', 'Yaw(deg)', 'Delta_Roll', 'Delta_Pitch', 'Delta_Yaw'])
    
    start_time = time.time()
    
    try:
        while True:
            if ser.in_waiting > 0:
                line_data = ser.readline().decode('utf-8').strip()
                
                try:
                    # 解析 STM32 傳來的純數字 (例如 "30.5,10.2,45.0")
                    r, p, y = map(float, line_data.split(','))
                    
                    # 計算 Delta 變化量
                    if last_r is None:
                        dr, dp, dy = 0.0, 0.0, 0.0
                    else:
                        dr = r - last_r
                        dp = p - last_p
                        dy = y - last_y
                    
                    last_r = r
                    last_p = p
                    last_y = y
                    
                    current_time = time.time() - start_time
                    
                    # 寫入資料 (移除 Label)
                    writer.writerow([f"{current_time:.3f}", r, p, y, round(dr, 4), round(dp, 4), round(dy, 4)])
                    
                    print(f"記錄: T:{current_time:.3f}s | Pitch:{p:6.1f} (dP:{dp:6.2f})")
                    
                except ValueError:
                    pass

    except KeyboardInterrupt:
        print("\n收到停止指令，正在儲存檔案並關閉通訊埠...")

ser.close()
print("記錄完成！檔案已儲存。")