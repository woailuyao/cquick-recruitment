# 第一周作业 · 视觉算法（OpenCV 基础）

<!-- 「图片素材与来源」一节已按实际文件逐个核对填写；其余小节是结构占位，按提示补上你自己的数字与结论。 -->

## 1. 环境与依赖

- Python **3.10.22**（conda 环境 `cv_practice`）
- opencv-python **5.0.0**、numpy **2.2.6**、matplotlib **3.10.9**

安装：

```bash
conda create -n cv_practice python=3.10 -y
conda activate cv_practice
pip install opencv-python numpy matplotlib -i https://mirrors.aliyun.com/pypi/simple
```

> 说明：清华源当前拉不到 `opencv-python`（返回 `from versions: none`），换阿里源可装。

## 2. 目录结构

```
W1_薛子仪_视觉算法/
├── README.md
├── src/                  task1_basics.py · task2_processing.py · task3_color_detect.py
├── data/images/          img1–3（任务一）· img21–23（任务二）· img31–33（任务三）
└── outputs/              task1 / task2 / task3
```

## 3. 运行方式

```bash
cd /d <项目根目录>
conda activate cv_practice

python src/task1_basics.py
python src/task2_processing.py
python src/task3_color_detect.py
```

<!-- TODO：task1 当前输出目录名为 outputs\the_first_task，且脚本未建目录；
     要么先把目录建好写在这里，要么统一目录名后更新本节。 -->

## 4. 三个任务的做法与关键参数

### 任务一 基础操作

- 读取：`np.fromfile` + `cv2.imdecode`（兼容中文路径），打印高、宽、通道数
- 缩放：缩小 `fx=fy=0.5` 用 `INTER_AREA`；放大 2× 用 `INTER_CUBIC`
- 裁剪：中心 1/3–2/3 区域
- 灰度：`cv2.COLOR_BGR2GRAY`
- 保存后重新读取，确认可读
- 输出：`outputs/task1/`（12 张）

### 任务二 处理对比

- 高斯：(5,5) 与 (15,15)，sigma 由核大小自动推
- 阈值：固定 80 / 160；另做自适应 `blockSize=31, C=5`
- Canny：(30,90) 与 (100,200)
- 形态学：开运算去白点、闭运算补黑洞，核 5×5
- 输出：`outputs/task2/`（16 张）

### 任务三 颜色提取

- BGR→HSV，`inRange` 出掩膜，开 + 闭运算清理
- 轮廓：`RETR_TREE`，面积 ≥1600 过滤
- 标注：`minAreaRect` + `boxPoints` 画旋转外接框，`moments` 求质心
- 输出：`outputs/task3/`，mask 与 result 各一张

## 5. 关键参数对结果的影响

<!-- TODO：把真实数字填进来。没有数字的“参数影响”等于没写。 -->

| 参数 | 变化 | 观察到的变化 |
| --- | --- | --- |
| Canny 阈值 | (30,90) → (100,200) | 边缘像素由 ___ → ___ |
| 固定阈值 | 80 → 160 | 白像素由 ___ → ___ |
| 高斯核 | 5×5 → 15×15 | 细节 __（变糊/保留） |
| 开/闭运算 | — | 开：去孤立白点；闭：补内部空洞 |

## 6. 失败案例与改进

1. **中文路径读图返回 None**：`cv2.imread` 失败时只返回 None，报错信息却是“文件损坏”。
   原因：OpenCV 在 Windows 走窄字符接口。改进：`np.fromfile` + `cv2.imdecode` 读取，`imencode` + `tofile` 保存。
2. **保存失败不报错**：`cv2.imwrite` 失败只返回 False。
   改进：改用 `imencode` + `tofile`，并检查返回值。

## 7. 图片素材与来源

所有测试图均来自 **Wikimedia Commons**（公有领域或 CC 授权），已按实际文件逐个核对作者与许可。
使用 CC BY-SA 的图片**必须署名并以相同方式共享**；CC0 / 公有领域的图片可自由使用。

### 任务一（基础操作）

| 文件 | 尺寸 | 原图（Commons） | 作者 | 许可 | 链接 |
| --- | --- | --- | --- | --- | --- |
| `img1.jpg` | 1280×960 | Mandarin fruit on white background.jpg | Paolo Neo | Public domain | [文件页](https://commons.wikimedia.org/wiki/File%3AMandarin_fruit_on_white_background.jpg) |
| `img2.jpg` | 1280×853 | Orange -- 2022 -- 9713.jpg | Dietmar Rabich | CC BY-SA 4.0 | [文件页](https://commons.wikimedia.org/wiki/File%3AOrange_--_2022_--_9713.jpg) |
| `img3.jpg` | 1280×960 | 由 `img1.jpg` 缩放至 28% 贴白底**合成**，源图同 Paolo Neo（Public domain） | — | Public domain | [源文件页](https://commons.wikimedia.org/wiki/File%3AMandarin_fruit_on_white_background.jpg) |

### 任务二（处理对比）

| 文件 | 尺寸 | 原图（Commons） | 作者 | 许可 | 链接 |
| --- | --- | --- | --- | --- | --- |
| `img21.jpg` | 960×960 | Flat white coffee at Highdown Gardens, Worthing, West Sussex, England.jpg | Acabashi | CC BY-SA 4.0 | [文件页](https://commons.wikimedia.org/wiki/File%3AFlat_white_coffee_at_Highdown_Gardens%2C_Worthing%2C_West_Sussex%2C_England.jpg) |
| `img22.jpg` | 960×1334 | 05 - O Fluminense - 01 - Niterói.jpg | OrfeuCantador | CC BY-SA 4.0 | [文件页](https://commons.wikimedia.org/wiki/File%3A05_-_O_Fluminense_-_01_-_Niter%C3%B3i.jpg) |
| `img23.jpg` | 960×720 | - panoramio (2542).jpg | quexinhua | CC BY 3.0 | [文件页](https://commons.wikimedia.org/wiki/File%3A-_panoramio_%282542%29.jpg) |

### 任务三（颜色提取）

| 文件 | 尺寸 | 原图（Commons） | 作者 | 许可 | 链接 |
| --- | --- | --- | --- | --- | --- |
| `img31.jpg` | 617×694 | Aardbei dwrsdsn.jpg | Rasbak at Dutch Wikipedia | CC BY-SA 3.0 | [文件页](https://commons.wikimedia.org/wiki/File%3AAardbei_dwrsdsn.jpg) |
| `img32.jpg` | 960×640 | Banana (1).jpg | Renee Comet | Public domain | [文件页](https://commons.wikimedia.org/wiki/File%3ABanana_%281%29.jpg) |
| `img33.jpg` | 960×720 | 2 x Cherry Bambelo tomato 2017 A1.jpg | Fructibus | CC0 | [文件页](https://commons.wikimedia.org/wiki/File%3A2_x_Cherry_Bambelo_tomato_2017_A1.jpg) |

**署名说明**：CC BY-SA 4.0 / 3.0 的图片（`img2`、`img21`、`img22`、`img31`）使用时应保留作者姓名与许可声明；`img1`、`img3`、`img32` 为公有领域，`img33` 为 CC0，均无附加限制；`img23` 为 CC BY 3.0，需署名。

## 8. 已知问题 / 待改进

- 三个脚本均未 `os.makedirs` 输出目录，换机器运行时需先手动建目录
- 任务一脚本中裁剪复用了上一轮循环的 `h`、`w`，多图尺寸不同时 ROI 会按最后一张计算
- 任务三测试图片数量与 PDF 要求的“3 张不同背景/光照”是否一致，待确认

## 9. 验收自查

- [ ] 三个脚本按第 3 节命令可独立运行
- [ ] 每种操作至少一张输出图
- [ ] 原图未被覆盖（`data/images/` 修改时间未变）
- [ ] 代码内路径均为相对路径
- [ ] 参数影响有数字支撑
- [ ] 图片版权已注明（第 7 节）
