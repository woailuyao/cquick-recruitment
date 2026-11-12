# 第 2 周考核 · 薛子仪

- 姓名：薛子仪（xueziyi）
- 完成日期：2026-11-01
- Ubuntu / ROS2 / Python / OpenCV：Ubuntu 22.04.5 LTS · ROS2 Humble · Python 3.10.12 · OpenCV 4.10.0

## 数据流

```
图片（1000px 宽的 PNG）
  │ cv2.imdecode 读入
  ▼
BGR 图
  │ cvtColor(BGR2HSV)
  ▼
HSV 图
  │ 对 5 种颜色分别 inRange，比"最大轮廓面积"选一种
  ▼
mask（白=该颜色区域）
  │ 开运算去小白点 + 闭运算补小洞
  ▼
干净 mask
  │ findContours
  ▼
轮廓列表 → 面积 ≥ 1600 才保留
  │ 按面积降序，取最大的一个
  ▼
中心点 (u, v) + 面积
  │ 填进 TargetPosition
  ▼
/target_position 话题 → 订阅端打印
```

## 任务 1：颜色目标检测

### HSV 阈值的作用

HSV 阈值可以用来排除颜色过淡或过浓的部分，让分割专注于物体本体，而不是背景中较淡或过浓的颜色。

这一点在 HSV 空间里尤其重要：H 管"是什么颜色"，S 管"颜色纯不纯"，V 管"亮不亮"。
把 S、V 的下界提高，就能把发灰、发白的像素（高光、雾面背景）排除掉。

### 轮廓面积筛选的作用

轮廓面积筛选可以去掉画面中明显过小、不可能是目标的物体，从而提高识别准确率。

面积阈值设得太小会留下一堆反光点和色块；设得太大则会把小目标整个漏掉。这两者的取舍见任务 2 的对比实验。

### 参数记录

| 项 | 取值 |
| --- | --- |
| 颜色表 | blue / green / orange / yellow / red（红跨 0 点，用两段 inRange 再按位或） |
| 选色规则 | 5 种颜色各生成 mask，取"最大轮廓面积"最大的那种颜色 |
| 最小轮廓面积 | 1600 px |
| 多候选规则 | 按 `cv2.contourArea` 降序，取面积最大的轮廓 |
| 中心点算法 | 轮廓矩 `u = m10/m00`，`v = m01/m00`；`m00 == 0` 时跳过该轮廓 |
| 测试图片 | `images/img1.png` ~ `img4.png`（Pexels 公开图片，来源见 SOURCES.md） |

### 实测结果

| 图片 | 选中的颜色 | u | v | 面积(px) | 说明 |
| --- | --- | ---: | ---: | ---: | --- |
| img1.png | green | 211 | 158 | 33,846 | **选错对象**：草地比足球大 |
| img2.png | blue | 477 | 288 | 159,504 | 正确（蓝色毛线球） |
| img3.png | red | 420 | 719 | 127,564 | 正确（红苹果） |
| img4.png | green | 298 | 503 | 189,012 | 部分正确（青苹果，但与红苹果粘连） |

## 任务 2：可视化与参数对比

### 可视化

标注内容：绿色轮廓线 + 黄色最小外接矩形 + 白色中心点 + 坐标文字。
结果保存为 `out/imgN_out.png`，不只依赖临时窗口。

坐标约定：左上角为 (0,0)，u 向右增大，v 向下增大。
**像素坐标不是场地坐标，也不是米制距离**；要换算需要相机标定与几何关系。

### 对比实验一：面积阈值 800 / 1600 / 3200

（1600 为原始值，800 和 3200 分别是"变小一倍"和"变大一倍"）

| 图片 | 800 | 1600 | 3200 |
| --- | ---: | ---: | ---: |
| img1.png | 29 | 13 | 9 |
| img2.png | 2 | 1 | 1 |
| img3.png | 3 | 3 | 1 |
| img4.png | 7 | 4 | 2 |

**结论**：

- 阈值增大 → 保留的轮廓明显减少（img1 从 29 降到 9）
- 阈值减小 → 保留的轮廓明显增多（img1 从 13 升到 29）
- 在 img1 上对比最明显：阈值增大后误检减少，但漏检增多
- **本质是"多留"和"少留"的权衡**：留得多，可能把背景当目标（误检）；留得少，小目标直接被筛掉（漏检）

### 对比实验二：开闭运算 有 / 无

| 图片 | 有开闭运算 | 无开闭运算 |
| --- | ---: | ---: |
| img1.png | 13 | 10 |
| img2.png | 1 | 2 |
| img3.png | 3 | 3 |
| img4.png | 4 | 5 |

**结论**：

- 进行开闭运算后，小轮廓有所减少，轮廓的位置也会改变
- 在识别出小轮廓较多的图片（img1）中，小轮廓减少明显，大轮廓相对增多
- 但轮廓中心点会有所偏移，稳定性降低，可能导致误检增多
- img2 这个单目标场景里，开闭运算把原本 2 个碎轮廓合并成 1 个，效果最好

### 失败案例

**案例一：img1 漏检足球，转而识别草地**

img1 是草地上的足球，但足球本身是白色 + 黑色花纹，**没有高饱和度的颜色**。
而绿色草地在整张图中占比最大，于是 `color_choose` 按"最大轮廓面积"选中了 green，
把草地当成了目标，足球被忽略。

**这是"选色规则只看面积"的直接后果**：目标无彩色、背景有大片纯色时，规则必然选错。

**案例二：img4 多个苹果被识别成单个物体**

img4 是一堆红绿苹果。苹果之间相互接触，`findContours` 把相邻苹果的连通区域
当成**一个整体轮廓**，于是一堆苹果只输出一个中心点。

这不是代码 bug，是"连通区域"的定义决定的：**只要像素连着，就是一个轮廓**。
要分开必须用分水岭、距离变换或轮廓层级（`RETR_TREE` + 父子关系）。

## 任务 3：ROS2 通信

### 消息定义 `mine_interfaces/msg/TargetPosition`

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `detected` | bool | 本帧是否检测到目标 |
| `u` | float32 | 中心横坐标（像素，向右增大） |
| `v` | float32 | 中心纵坐标（像素，向下增大） |
| `area` | float32 | 轮廓面积（像素平方，近似） |

未检测到目标时：`detected=false`，`u/v/area` 全为 0。
**下游必须先看 `detected`，不能把 0 当成"目标在左上角"。**

### 节点

| 节点 | 文件 | 作用 |
| --- | --- | --- |
| 发布 | `mine_detector/publish.py` | 每秒取一张图 → 检测 → 填消息 → 发到 `/target_position` |
| 订阅 | `mine_detector/subscriber.py` | 订阅同一话题 → 打印 detected/u/v/area |

话题名：**`/target_position`**

### 运行

```bash
cd ~/vision_ws/mine
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash

# 终端 1
ros2 run mine_detector subscriber

# 终端 2
ros2 run mine_detector publisher
```

### 实测输出

```
发布端：
[INFO] [publisher]: [img1.png] u=211.0 v=158.0 area=33846
[INFO] [publisher]: [img2.png] u=477.0 v=288.0 area=159504
[INFO] [publisher]: [img3.png] u=420.0 v=719.0 area=127564
[INFO] [publisher]: [img4.png] u=298.0 v=503.0 area=189012

订阅端：
[INFO] [subscriber]: #01 u=211.0 v=158.0 area=33846
[INFO] [subscriber]: #02 u=477.0 v=288.0 area=159504
[INFO] [subscriber]: #03 u=420.0 v=719.0 area=127564
[INFO] [subscriber]: #04 u=298.0 v=503.0 area=189012
```

**u/v 每秒变化，证明数据来自实际检测，不是写死的坐标。**

## 问题与解决过程

**问题 1：`ModuleNotFoundError: No module named 'mine_interfaces'`**

原因：没有 `source install/setup.bash`，Python 找不到编译出来的消息包。
解决：每个新终端都执行两条 source：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

**问题 2：`ros2 run` 报 `No module named 'mine_detector.publisher_node'`**

原因：我把文件从 `publisher_node.py` 改名为 `publish.py`，但 `setup.py` 的
`entry_points` 还写着旧名字。

解决：改成 `'publisher = mine_detector.publish:main'`，然后**重新 `colcon build`**
（entry_points 是在编译时写进启动脚本的，改完必须重编译）。

**问题 3：改了 `.py` 内容但不生效**

原因：改了 `.msg` 文件必须重编译；改了 `.py` 在 `--symlink-install` 下可免编译。
两者要分清。

## 仍未解决的问题

1. **选色规则太粗糙**：现在只按"最大轮廓面积"选颜色，遇到 img1 这种
   "目标无彩色、背景大片纯色"的情况必然选错。更合理的做法是限定 ROI、
   或者让用户显式指定目标颜色（做成 ROS2 参数）。
2. **粘连目标无法分开**：img4 多个苹果连在一起，被识别成一个轮廓。
   需要用距离变换 + 分水岭，或按轮廓层级做分割，本周没有实现。
3. **时间戳用哪个时钟**：`get_clock().now()` 在仿真（use_sim_time）下与系统时间混用
   会出现时间跳变，尚未验证。
