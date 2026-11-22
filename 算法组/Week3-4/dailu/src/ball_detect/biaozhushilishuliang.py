import os

# 把你的 labels/train 路径填进去
label_dir = 'soccer_dataset/labels/test'
total_instances = 0

for file in os.listdir(label_dir):
    if file.endswith('.txt'):
        with open(os.path.join(label_dir, file), 'r') as f:
            lines = f.readlines()
            total_instances += len(lines) # 有几行，就是几个框

print(f"✅ 统计完成！train集总共有 {total_instances} 个标注实例（框）。")