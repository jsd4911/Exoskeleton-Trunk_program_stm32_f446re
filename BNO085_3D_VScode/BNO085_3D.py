import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

# --- 設定通訊埠 (請修改為您的 COM Port) ---
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200

# 初始化序列埠
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"成功連接 {SERIAL_PORT}")
except Exception as e:
    print(f"無法打開通訊埠: {e}")
    exit()

# 儲存數據的陣列 (最多保留 100 筆歷史軌跡)
max_points = 100
xs, ys, zs = [], [], []

# 設定 3D 畫布
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
line, = ax.plot([], [], [], lw=2, color='b')

ax.set_xlim(-180, 180)
ax.set_ylim(-90, 90)
ax.set_zlim(-180, 180)
ax.set_xlabel('Roll (deg)')
ax.set_ylabel('Pitch (deg)')
ax.set_zlabel('Yaw (deg)')
ax.set_title('BNO085 3D Orientation Trajectory')

def animate(i):
    global xs, ys, zs
    
    # 讀取一行 STM32 傳來的數據
    if ser.in_waiting > 0:
        line_data = ser.readline().decode('utf-8').strip()
        try:
            # 解析以逗號分隔的數字
            r, p, y = map(float, line_data.split(','))
            
            xs.append(r)
            ys.append(p)
            zs.append(y)
            
            # 限制軌跡長度，避免記憶體爆滿
            if len(xs) > max_points:
                xs.pop(0)
                ys.pop(0)
                zs.pop(0)
                
            # 更新 3D 曲線
            line.set_data(xs, ys)
            line.set_3d_properties(zs)
            
        except ValueError:
            pass # 忽略格式錯誤的垃圾封包

    return line,

# 啟動即時動畫，更新頻率約 20 毫秒 (50Hz)
ani = animation.FuncAnimation(fig, animate, interval=20, blit=False)

plt.show()
ser.close()