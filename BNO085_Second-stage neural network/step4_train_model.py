import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from sklearn.model_selection import train_test_split
import pandas as pd
import numpy as np
import glob

WINDOW_SIZE = 10
NUM_FEATURES = 2
NUM_CLASSES = 4
BATCH_SIZE = 32
EPOCHS = 100

class IMUDataset(Dataset):
    def __init__(self, X, y):
        self.X = torch.tensor(X, dtype=torch.float32)
        self.y = torch.tensor(y, dtype=torch.long)
    def __len__(self): return len(self.y)
    def __getitem__(self, idx): return self.X[idx], self.y[idx]

def create_sliding_windows(features, labels, window_size):
    X, y = [], []
    features_values = features.values
    labels_values = labels.values
    for i in range(len(features) - window_size):
        X.append(features_values[i : i + window_size])
        y.append(labels_values[i + window_size - 1])
    
    # 🌟 在 NumPy 階段就把 [數量, 10, 2] 翻轉成 PyTorch 愛的 [數量, 2, 10]
    X = np.array(X)
    X = np.transpose(X, (0, 2, 1))
    return X, np.array(y)

class ExoskeletonCNN(nn.Module):
    def __init__(self, num_features, window_size, num_classes):
        super(ExoskeletonCNN, self).__init__()
        self.conv1 = nn.Conv1d(in_channels=num_features, out_channels=16, kernel_size=3, padding=1)
        self.relu = nn.ReLU()
        self.pool = nn.MaxPool1d(kernel_size=2)
        self.flat_size = 16 * (window_size // 2) # 16 * 5 = 80
        self.fc1 = nn.Linear(self.flat_size, 32)
        self.fc2 = nn.Linear(32, num_classes)

    def forward(self, x):
        x = self.conv1(x)
        x = self.relu(x)
        x = self.pool(x)
        
        # 🌟 訓練階段保持動態 Batch Size (不會再報 2560 的錯誤了！)
        x = x.reshape(x.size(0), self.flat_size) 
        
        x = self.fc1(x)
        x = self.relu(x)
        x = self.fc2(x)
        return x

if __name__ == '__main__':
    all_files = glob.glob("training_data/*.csv")
    if len(all_files) == 0:
        print("找不到 CSV 檔，請確認 training_data 內有檔案")
        exit()
        
    df_list = [pd.read_csv(f) for f in all_files]
    df = pd.concat(df_list, axis=0, ignore_index=True).fillna(0)
    
    features_data = df[['Pitch_Normalized', 'Delta_Pitch_Smoothed']]
    labels_data = df['Label']

    X, y = create_sliding_windows(features_data, labels_data, WINDOW_SIZE)
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

    train_loader = DataLoader(IMUDataset(X_train, y_train), batch_size=BATCH_SIZE, shuffle=True)
    test_loader = DataLoader(IMUDataset(X_test, y_test), batch_size=BATCH_SIZE, shuffle=False)

    model = ExoskeletonCNN(num_features=NUM_FEATURES, window_size=WINDOW_SIZE, num_classes=NUM_CLASSES)
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=0.001)

    print("🚀 開始訓練大腦...")
    for epoch in range(EPOCHS):
        model.train()
        for batch_X, batch_y in train_loader:
            optimizer.zero_grad()
            outputs = model(batch_X)
            loss = criterion(outputs, batch_y)
            loss.backward()
            optimizer.step()

    model.eval()
    correct, total = 0, 0
    with torch.no_grad():
        for batch_X, batch_y in test_loader:
            outputs = model(batch_X) # 這次會順利通過！
            _, predicted = torch.max(outputs.data, 1)
            total += batch_y.size(0)
            correct += (predicted == batch_y).sum().item()

    print(f"🎯 測試集準確率: {100 * correct / total:.2f}%")
    torch.save(model.state_dict(), "exoskeleton_model_V4.pth")
    print("📦 已儲存 'exoskeleton_model_V4.pth'")