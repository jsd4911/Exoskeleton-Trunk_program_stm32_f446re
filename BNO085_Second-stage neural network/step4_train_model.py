import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import pandas as pd
import numpy as np
import glob
import os

# ==========================================
# 參數設定
# ==========================================
DATA_PATH = "training_data/*.csv"  # 讀取所有標註好的 CSV
WINDOW_SIZE = 10                   # 0.5 秒的滑動視窗 (10 frames)
BATCH_SIZE = 64
EPOCHS = 100
LEARNING_RATE = 0.001

# ==========================================
# 1. 定義資料集 (Sliding Window Dataset)
# ==========================================
class ExoskeletonDataset(Dataset):
    def __init__(self, file_pattern, window_size):
        self.x_data = []
        self.y_move = []
        self.y_bend = []
        self.y_speed = []
        
        file_list = glob.glob(file_pattern)
        if not file_list:
            print(f"❌ 找不到任何資料！請確認 {file_pattern} 路徑是否正確。")
            exit()
            
        print(f"📂 找到 {len(file_list)} 個檔案，開始載入與切片...")
        
        for file in file_list:
            df = pd.read_csv(file)
            
            # 提取 6 個特徵
            features = df[['Pitch_Normalized', 'Delta_Pitch_Smoothed', 
                           'Roll_Normalized', 'Delta_Roll_Smoothed', 
                           'Yaw_Normalized', 'Delta_Yaw_Smoothed']].values
            
            # 提取 3 個標籤
            l1 = df['Label1_Move'].values
            l2 = df['Label2_Bend'].values
            l4 = df['Label4_Speed'].values
            
            # 滑動視窗切片
            for i in range(len(df) - window_size):
                window_x = features[i : i + window_size]
                
                # PyTorch 的 1D-CNN 需要的形狀是 (Channels, Length)
                # 所以要把 (10, 6) 轉置成 (6, 10) 🌟 完美對應 STM32 的 Channel-First!
                self.x_data.append(window_x.T) 
                
                # 取視窗最後一個時間點的標籤作為答案
                self.y_move.append(l1[i + window_size - 1])
                self.y_bend.append(l2[i + window_size - 1])
                self.y_speed.append(l4[i + window_size - 1])

        self.x_data = torch.tensor(np.array(self.x_data), dtype=torch.float32)
        self.y_move = torch.tensor(self.y_move, dtype=torch.long)
        self.y_bend = torch.tensor(self.y_bend, dtype=torch.long)
        self.y_speed = torch.tensor(self.y_speed, dtype=torch.long)
        
        print(f"✅ 載入完成！總共產生了 {len(self.x_data)} 筆訓練樣本。")

    def __len__(self):
        return len(self.x_data)

    def __getitem__(self, idx):
        return self.x_data[idx], self.y_move[idx], self.y_bend[idx], self.y_speed[idx]

# ==========================================
# 2. 定義輕量化多任務神經網路 (1D-CNN)
# ==========================================
class ExoskeletonMultiTaskCNN(nn.Module):
    def __init__(self):
        super(ExoskeletonMultiTaskCNN, self).__init__()
        
        # 特徵擷取層 (Feature Extractor)
        # 輸入: (Batch, 6, 10)
        self.conv1 = nn.Conv1d(in_channels=6, out_channels=16, kernel_size=3, padding=1)
        self.relu1 = nn.ReLU()
        self.pool1 = nn.MaxPool1d(kernel_size=2) # 長度變 5
        
        self.conv2 = nn.Conv1d(in_channels=16, out_channels=32, kernel_size=3, padding=1)
        self.relu2 = nn.ReLU()
        
        self.flatten = nn.Flatten()
        
        # 共享全連接層 (長度 5 * 32通道 = 160)
        self.fc_shared = nn.Linear(32 * 5, 64)
        self.relu_shared = nn.ReLU()
        
        # 🌟 三個獨立的決策輸出頭 (符合 V7.6 架構)
        self.head_move  = nn.Linear(64, 2)  # L1: 0(站), 1(移動)
        self.head_bend  = nn.Linear(64, 7)  # L2: 0~6 (直立, 前彎, 微彎, 側彎...)
        self.head_speed = nn.Linear(64, 3)  # L4: 0(慢), 1(中), 2(快)

    def forward(self, x):
        x = self.pool1(self.relu1(self.conv1(x)))
        x = self.relu2(self.conv2(x))
        x = self.flatten(x)
        shared_features = self.relu_shared(self.fc_shared(x))
        
        out_move  = self.head_move(shared_features)
        out_bend  = self.head_bend(shared_features)
        out_speed = self.head_speed(shared_features)
        
        return out_move, out_bend, out_speed

# ==========================================
# 3. 訓練流程
# ==========================================
def calculate_accuracy(preds, labels):
    _, predicted = torch.max(preds, 1)
    correct = (predicted == labels).sum().item()
    return correct / labels.size(0)

if __name__ == "__main__":
    dataset = ExoskeletonDataset(DATA_PATH, WINDOW_SIZE)
    dataloader = DataLoader(dataset, batch_size=BATCH_SIZE, shuffle=True)
    
    model = ExoskeletonMultiTaskCNN()
    optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE)
    
    # 使用交叉熵損失函數 (適合分類問題)
    criterion = nn.CrossEntropyLoss()
    
    print("\n🚀 開始訓練模型...")
    for epoch in range(EPOCHS):
        model.train()
        total_loss = 0
        acc_move, acc_bend, acc_speed = 0, 0, 0
        
        for batch_x, batch_m, batch_b, batch_s in dataloader:
            optimizer.zero_grad()
            
            # 正向傳播
            pred_m, pred_b, pred_s = model(batch_x)
            
            # 計算三個任務的 Loss 並相加
            loss_m = criterion(pred_m, batch_m)
            loss_b = criterion(pred_b, batch_b)
            loss_s = criterion(pred_s, batch_s)
            loss = loss_m + loss_b + loss_s
            
            # 反向傳播與優化
            loss.backward()
            optimizer.step()
            
            total_loss += loss.item()
            acc_move += calculate_accuracy(pred_m, batch_m)
            acc_bend += calculate_accuracy(pred_b, batch_b)
            acc_speed += calculate_accuracy(pred_s, batch_s)
            
        # 計算平均準確率
        batches = len(dataloader)
        print(f"Epoch [{epoch+1}/{EPOCHS}] | Loss: {total_loss/batches:.4f} | "
              f"Acc - Move: {acc_move/batches*100:.1f}% | "
              f"Bend: {acc_bend/batches*100:.1f}% | "
              f"Speed: {acc_speed/batches*100:.1f}%")
              
    # 儲存模型權重
    SAVE_PATH = "V7_MultiTask_Model.pth"
    torch.save(model.state_dict(), SAVE_PATH)
    print(f"\n🎉 訓練完成！模型已儲存至: {SAVE_PATH}")