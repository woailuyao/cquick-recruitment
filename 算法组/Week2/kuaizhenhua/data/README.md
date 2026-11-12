# 测试输入

real_images：5张公开实拍照片，用于正式检测与ROS2输入。作者、原始页面、许可、尺寸和SHA256见其中的来源说明.md与sources.json。

images：6张标为SYNTHETIC的合成练习图，单张640×480，用于复现实操截图和单独检查面积、暗光等条件。它们不是本人拍摄。data/demo_manifest.json记录每张图的设定与哈希；生成代码在src/prepare_demo_images.py。

仅保留这些小型测试样例，不含训练数据集、权重或视频。
