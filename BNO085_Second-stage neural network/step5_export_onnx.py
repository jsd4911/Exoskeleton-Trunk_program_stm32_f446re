import torch
import torch.nn as nn

# ==========================================
# 1. 重新定義我們的大腦架構 (必須跟訓練時一模一樣)
# ==========================================
class ExoskeletonCNN(nn.Module):
    def __init__(self, num_features=3, window_size=10):
        super(ExoskeletonCNN, self).__init__()
        self.conv_layer = nn.Sequential(
            nn.Conv1d(in_channels=num_features, out_channels=16, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool1d(kernel_size=2)
        )
        flattened_length = 16 * (window_size // 2)
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(flattened_length, 16),
            nn.ReLU(),
            nn.Linear(16, 1),
            nn.Sigmoid()
        )

    def forward(self, x):
        features = self.conv_layer(x)
        output = self.classifier(features)
        return output

if __name__ == '__main__':
    # ==========================================
    # 2. 喚醒模型並載入記憶 (Weights)
    # ==========================================
    print("正在喚醒 AI 大腦...")
    model = ExoskeletonCNN(num_features=3, window_size=10)
    
    # 讀取您辛苦訓練出來的 pth 檔案
    model.load_state_dict(torch.load("exoskeleton_model.pth"))
    
    # 設定為「推論/考試模式」 (這很重要，確保模型不會在轉換時發生隨機變化)
    model.eval()

    # ==========================================
    # 3. 建立「虛擬輸入」並打包為 ONNX
    # ==========================================
    # ONNX 需要知道輸入數據的「形狀」。
    # 我們的輸入是: (Batch Size=1, Channels=3軸, Window Size=10筆)
    dummy_input = torch.randn(1, 3, 10)

    onnx_filename = "exoskeleton_model.onnx"
    
    print("開始將模型翻譯為 ONNX 格式...")
    torch.onnx.export(
        model,                  # 要轉換的模型
        dummy_input,            # 虛擬輸入張量
        onnx_filename,          # 輸出的檔名
        export_params=True,     # 同時儲存模型架構與權重
        opset_version=11,       # ONNX 的版本 (11 是最穩定且支援 STM32 的版本)
        do_constant_folding=True, # 執行模型最佳化 (讓模型在晶片上跑更快)
        input_names=['input'],  # 定義輸入節點的名稱
        output_names=['output'] # 定義輸出節點的名稱
    )
    
    print(f"✅ 轉換成功！已產生跨平台模型檔案: {onnx_filename}")