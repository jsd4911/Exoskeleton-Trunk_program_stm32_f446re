import pandas as pd
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
from sklearn.model_selection import train_test_split

# ==========================================
# 1. 定義超參數 (Hyperparameters)
# ==========================================
WINDOW_SIZE = 10     # 滑動視窗大小 (假設 20Hz，10筆資料 = 看過去 0.5 秒的歷史)
BATCH_SIZE = 32      # 每次訓練抓幾組視窗
LEARNING_RATE = 0.001

# ==========================================
# 2. 建立資料集類別 (PyTorch Dataset)
# ==========================================
class IMUDataset(Dataset):
    def __init__(self, X, y):
        # 轉換為 PyTorch 專用的 Tensor 格式，並調整形狀給 1D-CNN 吃
        # CNN 輸入格式要求：(資料筆數, 頻道數(幾軸), 序列長度(Window Size))
        self.X = torch.tensor(X, dtype=torch.float32).transpose(1, 2)
        self.y = torch.tensor(y, dtype=torch.float32).unsqueeze(1)

    def __len__(self):
        return len(self.X)

    def __getitem__(self, idx):
        return self.X[idx], self.y[idx]

def create_sliding_windows(data, labels, window_size):
    """將連續的時間序列，切割成一個個的滑動視窗"""
    X, y = [], []
    for i in range(len(data) - window_size):
        # 抓取 window_size 長度的數據做為特徵
        window = data.iloc[i : i + window_size].values
        # 標籤取視窗最後一刻的動作狀態
        target = labels.iloc[i + window_size - 1]
        X.append(window)
        y.append(target)
    return np.array(X), np.array(y)

# ==========================================
# 3. 定義神經網路架構 (1D-CNN) - 針對 TinyML 最佳化
# ==========================================
class ExoskeletonCNN(nn.Module):
    def __init__(self, num_features=3, window_size=10):
        super(ExoskeletonCNN, self).__init__()
        
        # 特徵擷取層：尋找波形的特徵 (例如彎腰的 U 型)
        self.conv_layer = nn.Sequential(
            nn.Conv1d(in_channels=num_features, out_channels=16, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool1d(kernel_size=2)
        )
        
        # 計算經過卷積和池化後的資料長度
        flattened_length = 16 * (window_size // 2)
        
        # 分類層：根據特徵決定是 0 還是 1
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(flattened_length, 16),
            nn.ReLU(),
            nn.Linear(16, 1),
            nn.Sigmoid() # 輸出 0~1 之間的機率值
        )

    def forward(self, x):
        features = self.conv_layer(x)
        output = self.classifier(features)
        return output

# ==========================================
# 4. 主程式：載入資料並初始化
# ==========================================
if __name__ == '__main__':
    import glob

    print("正在尋找並合併所有資料集...")
    # 假設您把所有處理好的 CSV 都放在一個叫做 "dataset" 的資料夾裡
    # glob 會自動抓出裡面所有檔名結尾是 .csv 的檔案
    all_files = glob.glob("training_data/*.csv") 
    
    df_list = []
    for filename in all_files:
        print(f"讀取: {filename}")
        temp_df = pd.read_csv(filename)
        df_list.append(temp_df)
        
    # 將所有表格上下拼接起來，變成一份超級巨大的黃金訓練集！
    df = pd.concat(df_list, axis=0, ignore_index=True)
    
    print(f"✅ 合併完成！總共有 {len(df)} 筆數據準備進行切割。")
    
    # 擷取特徵 (X) 與標籤 (y)
    # 我們這裡先用 Roll, Pitch, Yaw 三軸，未來您可以隨意增減
    features_data = df[['Roll_Filtered', 'Pitch_Filtered', 'Yaw_Filtered']]
    labels_data = df['Label']
    
    print("正在執行滑動視窗切割...")
    X, y = create_sliding_windows(features_data, labels_data, WINDOW_SIZE)
    
    # 切割訓練集 (80%) 與測試集 (20%)
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)
    
    # 包裝成 PyTorch 的 DataLoader
    train_loader = DataLoader(IMUDataset(X_train, y_train), batch_size=BATCH_SIZE, shuffle=True)
    test_loader = DataLoader(IMUDataset(X_test, y_test), batch_size=BATCH_SIZE, shuffle=False)
    
    print(f"✅ 資料準備完成！")
    print(f"訓練集有 {len(X_train)} 個視窗樣本")
    print(f"測試集有 {len(X_test)} 個視窗樣本")
    
    # 建立模型實體
    model = ExoskeletonCNN(num_features=3, window_size=WINDOW_SIZE)
    print("\n神經網路架構建立成功！")
    print(model)
    
    # 下一步：撰寫訓練迴圈 (Training Loop)
    # ==========================================
    # 5. 設定訓練引擎 (Loss Function & Optimizer)
    # ==========================================
    # 誤差函數：因為我們只有 0 和 1 兩種答案，所以用「二元交叉熵 (Binary Cross Entropy)」
    criterion = nn.BCELoss() 
    
    # 優化器：選用最經典的 Adam，它會自動幫我們控制學習的步伐大小
    optimizer = torch.optim.Adam(model.parameters(), lr=LEARNING_RATE)
    
    EPOCHS = 100  # 設定要讓 AI 把整份資料集反覆看幾遍 (訓練回合數)

    # ==========================================
    # 6. 開始訓練迴圈 (Training Loop)
    # ==========================================
    print("\n🚀 開始訓練神經網路...")
    for epoch in range(EPOCHS):
        model.train() # 將大腦設定為「訓練模式」
        total_loss = 0
        
        for batch_X, batch_y in train_loader:
            # 步驟 A: 歸零梯度 (清空上一題的記憶，避免干擾)
            optimizer.zero_grad()
            
            # 步驟 B: 正向傳播 (讓大腦看波形並猜測機率)
            predictions = model(batch_X)
            
            # 步驟 C: 計算誤差 (拿猜測結果跟真實標籤比對)
            loss = criterion(predictions, batch_y)
            
            # 步驟 D: 反向傳播 (找出做錯的神經元)
            loss.backward()
            
            # 步驟 E: 更新權重 (微調參數，自我進化)
            optimizer.step()
            
            total_loss += loss.item()
            
        # 每 5 個回合，向您報告一次目前的學習狀況
        if (epoch + 1) % 5 == 0 or epoch == 0:
            avg_loss = total_loss / len(train_loader)
            print(f"Epoch [{epoch+1}/{EPOCHS}] | 平均誤差 (Loss): {avg_loss:.4f}")

    # ==========================================
    # 7. 驗證與期末考 (Testing)
    # ==========================================
    print("\n🎓 訓練完成！開始拿沒看過的數據進行期末考...")
    model.eval()  # 將大腦設定為「考試模式」(停止學習與修正)
    correct = 0
    total = 0
    
    with torch.no_grad(): # 考試時不需要計算修正梯度 (節省記憶體與加速)
        for batch_X, batch_y in test_loader:
            outputs = model(batch_X)
            
            # AI 輸出的是 0.0 ~ 1.0 的機率。我們規定大於等於 0.5 就算彎腰(1)，否則算站直(0)
            predicted = (outputs >= 0.5).float()
            
            total += batch_y.size(0)
            correct += (predicted == batch_y).sum().item()
            
    accuracy = 100 * correct / total
    print(f"🎯 模型在未看過的測試集上，預測準確率為: {accuracy:.2f}%")

    # ==========================================
    # 8. 儲存模型大腦 (準備未來部署給 STM32)
    # ==========================================
    torch.save(model.state_dict(), "exoskeleton_model.pth")
    print("\n💾 模型的智慧結晶已儲存為 'exoskeleton_model.pth'")