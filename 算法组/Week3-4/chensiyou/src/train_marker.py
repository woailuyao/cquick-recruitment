from ultralytics import YOLO

def main():

    model = YOLO("yolov8n.pt")

    # 2. 开始训练
    model.train(
        data=r"D:\Marker_Dataset\marker_data.yaml",
        epochs=50,
        imgsz=1280,
        batch=16,
        device=0,
        project=r"D:\Marker_Dataset\runs",
        name="train_marker"
    )

if __name__ == '__main__':
    main()