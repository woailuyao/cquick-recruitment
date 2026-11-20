# vision_interface · 任务一 足球检测

| 文件 | 说明 |
| --- | --- |
| `detect_ball.py` | 实时检测：相机/视频 → YOLOv8n → 框、类别、置信度、像素中心、FPS |
| `msg/BallDetection.msg` | 检测结果消息（像素中心、框、置信度） |

运行：

```bash
python src/vision_interface/detect_ball.py --weights ball_best.pt
python src/vision_interface/detect_ball.py --weights ball_best.pt --source demo.mp4 --save out.mp4
```

依赖：`pip install ultralytics opencv-python`
