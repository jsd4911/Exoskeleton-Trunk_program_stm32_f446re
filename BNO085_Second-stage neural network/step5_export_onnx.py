import torch
import torch.nn as nn

# ==========================================
# 1. 貼上與 Step 4 完全相同的模型架構
# ==========================================
class ExoskeletonMultiTaskCNN(nn.Module):
    def __init__(self):
        super(ExoskeletonMultiTaskCNN, self).__init__()
        self.conv1 = nn.Conv1d(in_channels=6, out_channels=16, kernel_size=3, padding=1)
        self.relu1 = nn.ReLU()
        self.pool1 = nn.MaxPool1d(kernel_size=2) 
        self.conv2 = nn.Conv1d(in_channels=16, out_channels=32, kernel_size=3, padding=1)
        self.relu2 = nn.ReLU()
        self.flatten = nn.Flatten()
        
        self.fc_shared = nn.Linear(32 * 5, 64)
        self.relu_shared = nn.ReLU()
        
        # 🌟 V7 專屬的三個獨立輸出頭
        self.head_move  = nn.Linear(64, 2)  # L1: 2 狀態
        self.head_bend  = nn.Linear(64, 7)  # L2: 7 狀態 (擴充版)
        self.head_speed = nn.Linear(64, 3)  # L4: 3 狀態

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
# 2. 載入訓練好的權重並準備匯出
# ==========================================
if __name__ == "__main__":
    print("🔄 正在載入 V7_MultiTask_Model.pth...")
    model = ExoskeletonMultiTaskCNN()
    
    # 讀取剛剛訓練好的模型檔案
    model.load_state_dict(torch.load("V7_MultiTask_Model.pth"))
    model.eval() # 設定為推論模式 (非常重要，關閉 Dropout 等訓練機制)

    # 🌟 建立假輸入 (Dummy Input) 
    # 維度必須是: (Batch=1, Channels=6, Length=10)
    # 這完美對應我們在 STM32 裡準備的 my_ai_in 陣列！
    dummy_input = torch.randn(1, 6, 10)

    # 匯出 ONNX
    onnx_file_path = "V7_MultiTask_Model.onnx"
    torch.onnx.export(
        model, 
        dummy_input, 
        onnx_file_path,
        export_params=True,
        opset_version=11,  # STM32 X-CUBE-AI 支援度最好的版本
        do_constant_folding=True,
        input_names=['input_6_features'],
        output_names=['out_move', 'out_bend', 'out_speed'] # 只有三個輸出！
    )
    
    print(f"🎉 大功告成！ONNX 模型已成功匯出至: {onnx_file_path}")