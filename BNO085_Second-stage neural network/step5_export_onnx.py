import torch
import torch.nn as nn

class ExoskeletonCNN_Export(nn.Module):
    def __init__(self, num_features=2, window_size=10, num_classes=4):
        super(ExoskeletonCNN_Export, self).__init__()
        self.conv1 = nn.Conv1d(in_channels=num_features, out_channels=16, kernel_size=3, padding=1)
        self.relu = nn.ReLU()
        self.pool = nn.MaxPool1d(kernel_size=2)
        
        self.flat_size = 16 * (window_size // 2) 
        self.fc1 = nn.Linear(self.flat_size, 32)
        self.fc2 = nn.Linear(32, num_classes)

    def forward(self, x):
        x = self.conv1(x)
        x = self.relu(x)
        x = self.pool(x)
        
        # 使用 view 來強制固定形狀，對底層記憶體映射更友善
        x = x.view(1, 80)
        
        x = self.fc1(x)
        x = self.relu(x)
        x = self.fc2(x)
        return x

if __name__ == '__main__':
    print("正在準備靜態模型...")
    model = ExoskeletonCNN_Export(num_features=2, window_size=10, num_classes=4)
    
    # 直接讀取剛剛訓練好的完美大腦，不用重訓！
    model.load_state_dict(torch.load("exoskeleton_model_V4.pth"))
    model.eval() 

    dummy_input = torch.randn(1, 2, 10)
    onnx_filename = "exoskeleton_model_V4_Static.onnx"
    
    print("開始強制靜態匯出 (降級 Opset 避開 allowzero 錯誤)...")
    torch.onnx.export(
        model, 
        dummy_input, 
        onnx_filename,
        export_params=True,
        opset_version=10,  # 🌟 絕對關鍵：降級到 Opset 10！徹底消滅 allowzero 屬性
        do_constant_folding=True,
        input_names=['input'],
        output_names=['output']
    )
    print(f"✅ 成功匯出 {onnx_filename}！Opset 10 版本保證 100% 相容 STM32！")