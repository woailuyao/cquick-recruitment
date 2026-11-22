```
# 场地标志视觉定位实验
## 项目简介
本实验完成足球场L/T/X交点、点球点的数据采集，基于OpenCV实现相机标定与PnP位姿求解，实现相机相对场地的定位。

## 文件结构
```

├── calibration/
│   └── camera_extrinsics.yaml  # 相机外参配置（高度、俯仰角等）
├── config/
│   └── field.yaml              # L/T/X、点球点场地世界坐标
├── src/
│   ├── pnp_solve.py            # PnP 位姿求解主代码
│   ├── camera_calib.py         # 相机内参标定脚本
│   └── extract_frames.py       # 视频抽帧脚本
├── dataset_summary.md          # 数据集统计说明（图片、视频本地保存，不上传 GitHub）
└── README.md

> 说明：原始采集视频、抽帧图片数据集 football_dataset 体量较大，本地留存，**不提交至GitHub仓库**。

## 环境依赖
- Python 3.10
- OpenCV-python
- numpy
- PyYAML

安装依赖命令：
```bash
pip install opencv-python numpy pyyaml
## 运行步骤

1. 相机标定：`python src/camera_calib.py`，获取相机内参
2. 视频抽帧：`python src/extract_frames.py`，生成数据集图片并划分 train/val/test
3. PnP 位姿求解：`python src/pnp_solve.py`，输入标记点像素坐标，解算相机旋转、平移向量

## 实验说明

1. 在实验室采集 5 段包含 L/T/X 标志、点球点的场地视频，完成视频抽帧，样本总数满足≥300 张要求；数据集按照视频片段划分训练 / 验证 / 测试集，避免帧泄露。
2. 场地标记点世界坐标：L1 (3.73,2.79,0), L2 (-3.73,2.79,0), T1 (3.73,-2.79,0), X1 (-3.73,-2.79,0), 点球点 (0,2.79,0)。
