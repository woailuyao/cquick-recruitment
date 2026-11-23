import os
import shutil

# 标签源目录（你之前全标在这里了）
src_label_dir = 'dataset/labels/train'
# 目标标签目录（验证集）
dst_label_dir = 'dataset/labels/val'
# 验证集的图片目录（以它为准，找对应的标签）
val_img_dir = 'dataset/images/val'

os.makedirs(dst_label_dir, exist_ok=True)

count = 0
# 遍历验证集里的所有图片
for img_name in os.listdir(val_img_dir):
    if img_name.endswith('.jpg'):
        # 算出对应的 .txt 和 .xml 文件名
        txt_name = img_name.replace('.jpg', '.txt')
        xml_name = img_name.replace('.jpg', '.xml')
        
        # 找对应的 txt
        src_txt_path = os.path.join(src_label_dir, txt_name)
        dst_txt_path = os.path.join(dst_label_dir, txt_name)
        if os.path.exists(src_txt_path):
            shutil.copy(src_txt_path, dst_txt_path)
            count += 1
            
        # 找对应的 xml (顺便也挪过去，以备不时之需)
        src_xml_path = os.path.join(src_label_dir, xml_name)
        dst_xml_path = os.path.join(dst_label_dir, xml_name)
        if os.path.exists(src_xml_path):
            shutil.copy(src_xml_path, dst_xml_path)

print(f"✅ 完成！成功从 train 文件夹里挑出了 {count} 个验证集标签，放进了 val 文件夹。")