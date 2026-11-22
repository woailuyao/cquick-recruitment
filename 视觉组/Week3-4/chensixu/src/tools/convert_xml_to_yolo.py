import os
import glob
import xml.etree.ElementTree as ET

# 你的 XML 标签文件夹路径
xml_dir = 'dataset/labels/train'

# 类别映射 (名字必须和你标注时填的一字不差)
class_mapping = {
    'LCross': 0,
    'TCross': 1,
    'XCross': 2
}

xml_files = glob.glob(os.path.join(xml_dir, '*.xml'))
print(f"找到 {len(xml_files)} 个 XML 文件，开始转换...")

for xml_file in xml_files:
    tree = ET.parse(xml_file)
    root = tree.getroot()
    
    size = root.find('size')
    w = int(size.find('width').text)
    h = int(size.find('height').text)
    
    txt_file = xml_file.replace('.xml', '.txt')
    
    with open(txt_file, 'w') as f:
        for obj in root.iter('object'):
            cls_name = obj.find('name').text
            if cls_name not in class_mapping:
                continue
                
            cls_id = class_mapping[cls_name]
            
            xmlbox = obj.find('bndbox')
            xmin = float(xmlbox.find('xmin').text)
            xmax = float(xmlbox.find('xmax').text)
            ymin = float(xmlbox.find('ymin').text)
            ymax = float(xmlbox.find('ymax').text)
            
            # 转换成 YOLO 格式的 中心点 x, 中心点 y, 宽度, 高度 (全部归一化到 0-1)
            x_center = (xmin + xmax) / (2.0 * w)
            y_center = (ymin + ymax) / (2.0 * h)
            width = (xmax - xmin) / w
            height = (ymax - ymin) / h
            
            f.write(f"{cls_id} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}\n")

print("✅ 转换完成！所有的 .xml 都已经生成了对应的 .txt 文件。")