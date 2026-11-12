# test_images 放什么

本目录里的 `blue_01.png` ~ `blue_05.png` 由 `tools/prepare_test_images.py` 自动生成，
分别对应下面 5 种情况（**目标颜色是蓝色球**）：

| 文件 | 场景 | 用途 |
| --- | --- | --- |
| `blue_01.png` | 球最大、最清楚 | 主结果图、参数对比基准 |
| `blue_02.png` | 中距球 | 常规检出样本 |
| `blue_03.png` | 远处小球（面积接近 `min_area`） | 验证 `min_area` 的影响、临界样本 |
| `blue_04.png` | **只有草地和天空，没有球** | 验证「未检测到」；放宽 S 下界时用它演示「蓝天误检」 |
| `blue_05.png` | `blue_01` 压暗到 40% | 光照对比实验、漏检案例 |

## 怎么生成的

```bash
# 在 houkehong/ 目录下，把桌面「图」文件夹里的 4 张照片整理成 blue_01~05
python3 tools/prepare_test_images.py
```

它会读源目录里所有图片、用默认蓝色阈值各跑一遍检测、**按检出面积从大到小重命名**
（没检出的自然排到最后，就是那张「无目标」图），再把最大的那张压暗生成 `blue_05`。

换自己的照片时，把照片丢进任意文件夹，然后：

```bash
python3 tools/prepare_test_images.py --src /path/to/your/photos
```

## 没有照片怎么办

```bash
python3 tools/make_test_images.py     # 生成 5 张蓝色球合成图（兜底，仅用于跑通流程）
```

合成图只能验证代码和 ROS2 通信，**正式提交前建议换成真实照片**，并在 README 的
「数据来源」里写清楚是自拍、AI 生成还是网图。
