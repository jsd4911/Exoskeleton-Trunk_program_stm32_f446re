import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from scipy.spatial.transform import Rotation

# ==========================================
# 建立 3D 視窗與排版
# ==========================================
fig = plt.figure(figsize=(10, 8))
fig.suptitle('🚢 3D 姿態手動模擬器 (Interactive Attitude Simulator)', fontsize=16, fontweight='bold')

# 預留底部空間給滑桿
plt.subplots_adjust(bottom=0.35) 
ax_3d = fig.add_subplot(111, projection='3d')

# 設定座標軸範圍與標籤
ax_3d.set_xlim([-1, 1]); ax_3d.set_ylim([-1, 1]); ax_3d.set_zlim([-1, 1])
ax_3d.set_xlabel('X'); ax_3d.set_ylabel('Y'); ax_3d.set_zlabel('Z')

# 建立 3D 軸線物件 (紅:X/前方, 綠:Y/左側, 藍:Z/上方)
base_x = np.array([1, 0, 0])
base_y = np.array([0, 1, 0])
base_z = np.array([0, 0, 1])

line_x, = ax_3d.plot([0, 1], [0, 0], [0, 0], color='red', linewidth=5, label='Forward (X / Roll Axis)')
line_y, = ax_3d.plot([0, 0], [0, 1], [0, 0], color='green', linewidth=5, label='Left (Y / Pitch Axis)')
line_z, = ax_3d.plot([0, 0], [0, 0], [0, 1], color='blue', linewidth=5, label='Up (Z / Yaw Axis)')
ax_3d.legend(loc='upper left')

# ==========================================
# 建立 UI 控制元件 (滑桿)
# ==========================================
# 定義滑桿的放置位置 [左, 下, 寬, 高]
ax_yaw   = plt.axes([0.15, 0.20, 0.65, 0.03])
ax_pitch = plt.axes([0.15, 0.15, 0.65, 0.03])
ax_roll  = plt.axes([0.15, 0.10, 0.65, 0.03])

# 建立滑桿物件 (範圍 -180 到 180 度)
s_yaw   = Slider(ax_yaw, 'Yaw (Z)', -180.0, 180.0, valinit=0.0)
s_pitch = Slider(ax_pitch, 'Pitch (Y)', -180.0, 180.0, valinit=0.0)
s_roll  = Slider(ax_roll, 'Roll (X)', -180.0, 180.0, valinit=0.0)

# ==========================================
# 即時更新邏輯
# ==========================================
def update(val):
    # 讀取當前滑桿數值
    y = s_yaw.val
    p = s_pitch.val
    r = s_roll.val
    
    # 計算四元數旋轉 (ZYX 順序)
    rot = Rotation.from_euler('zyx', [y, p, r], degrees=True)
    
    # 套用旋轉矩陣到三根基礎軸
    rot_x = rot.apply(base_x)
    rot_y = rot.apply(base_y)
    rot_z = rot.apply(base_z)
    
    # 更新畫面上線條的 3D 座標
    line_x.set_data_3d([0, rot_x[0]], [0, rot_x[1]], [0, rot_x[2]])
    line_y.set_data_3d([0, rot_y[0]], [0, rot_y[1]], [0, rot_y[2]])
    line_z.set_data_3d([0, rot_z[0]], [0, rot_z[1]], [0, rot_z[2]])
    
    # 重新渲染畫面
    fig.canvas.draw_idle()

# 將滑桿綁定更新事件
s_yaw.on_changed(update)
s_pitch.on_changed(update)
s_roll.on_changed(update)

# 建立歸零按鈕
ax_reset = plt.axes([0.8, 0.025, 0.1, 0.04])
btn_reset = Button(ax_reset, 'Reset')

def reset(event):
    s_yaw.reset()
    s_pitch.reset()
    s_roll.reset()
btn_reset.on_clicked(reset)

plt.show()