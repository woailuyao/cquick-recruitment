from ultralytics import YOLO

if __name__ == '__main__':
    # 加载官方预训练模型
    model = YOLO('yolov8n.pt') 

    # 开始训练 (注意把下面这些代码整体往右缩进一下)
    model.train(
        data='ball_data.yaml', 
        epochs=50,           # 训练 50 轮
        imgsz=640,           # 图像尺寸
        batch=16,             # 如果显存小就改小（比如4），显存大可以改大（比如16）
        device=0,        # 如果没有显卡用 cpu，有显卡换成 0
        workers=0            # Windows 下有时设为 0 能避免多进程报错
    )