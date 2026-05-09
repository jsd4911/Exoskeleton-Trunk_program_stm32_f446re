import onnx

print("正在打開 ONNX 檔案進行強制修復...")

# 讀取您剛剛產生的 v2 模型
input_filename = "exoskeleton_model_V4_Static.onnx"
model = onnx.load(input_filename)

# 遍歷模型裡面的所有數學節點
for node in model.graph.node:
    # 只要看到是 Reshape (攤平) 節點
    if node.op_type == "Reshape":
        # 把裡面的 'allowzero' 屬性強行剔除
        cleaned_attributes = [attr for attr in node.attribute if attr.name != 'allowzero']
        del node.attribute[:]
        node.attribute.extend(cleaned_attributes)

# 將乾淨的模型另存新檔
fixed_filename = "exoskeleton_model_V4_Static_fixed.onnx"
onnx.save(model, fixed_filename)

print(f"✅ 違禁品已清除！請到 STM32CubeIDE 匯入這個全新檔案: {fixed_filename}")