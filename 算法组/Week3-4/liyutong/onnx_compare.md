# ONNX 导出与 PyTorch / ONNXRuntime 推理对比

> 生成时间：2026-11-17 18:08　> 设备：CPU（torch 2.2.2+cpu / onnxruntime CPUExecutionProvider），输入 640×640，conf 0.25，预热 3 张后统计。

## 任务 1 · 足球检测

| 项 | PyTorch (.pt) | ONNXRuntime (.onnx) |
| --- | ---: | ---: |
| 文件体积 | 6.51 MB | 12.75 MB |
| 单图耗时 均值 | 111.3 ms | 32.3 ms |
| 单图耗时 中位 | 115.3 ms | 32.0 ms |
| 单图耗时 P95 | 129.6 ms | 34.3 ms |
| 等效 FPS | 9.0 | 31.0 |

检测结果一致性（IoU≥0.5 配对，20 张图）：PyTorch 共 15 框 / ONNX 共 14 框 / 配对成功 14 框（93.3%），配对框置信度平均差 0.0330

## 任务 2 · 场地标志

| 项 | PyTorch (.pt) | ONNXRuntime (.onnx) |
| --- | ---: | ---: |
| 文件体积 | 6.51 MB | 12.75 MB |
| 单图耗时 均值 | 198.9 ms | 68.1 ms |
| 单图耗时 中位 | 196.3 ms | 68.1 ms |
| 单图耗时 P95 | 242.4 ms | 72.1 ms |
| 等效 FPS | 5.0 | 14.7 |

检测结果一致性（IoU≥0.5 配对，20 张图）：PyTorch 共 28 框 / ONNX 共 28 框 / 配对成功 28 框（100.0%），配对框置信度平均差 0.0205

## GPU 推理对比（DirectML，RTX 5060 Laptop 8 GB）

> 生成时间：2026-11-17 23:16　> onnxruntime 1.23.0 + `onnxruntime-directml`，执行提供器 `DmlExecutionProvider`（DX12），输入 1×3×640×640，预热 5 次后统计 30 次，取中位。

**口径说明**：上面两节是 `ultralytics predict()` 的端到端单图耗时，含读图、letterbox、
前后处理与 NMS；本节只统计 `session.run()` 的前向，所以同一个模型在 CPU 上的数字比上面低。
两节的数字只在本节内部横向比较，不与上面两节交叉比较。

| 模型 | 体积 | CPU 前向 | GPU 前向 | CPU P95 | GPU P95 | 加速比 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 任务 1 · 足球 ball | 12.75 MB | 21.87 ms（45.7 FPS） | **2.79 ms（358.9 FPS）** | 28.42 ms | 3.00 ms | **7.8×** |
| 任务 2 · 标志 L/T/X | 12.75 MB | 27.47 ms（36.4 FPS） | **4.23 ms（236.6 FPS）** | 54.78 ms | 9.28 ms | **6.5×** |

- 任务 1 · 足球 ball：8 张真实测试图上 CPU 与 GPU 的原始输出张量最大绝对差 1.22e-03、平均 1.20e-05，可视为同一结果。
- 任务 2 · 标志 L/T/X：8 张真实测试图上 CPU 与 GPU 的原始输出张量最大绝对差 7.48e-04、平均 6.25e-06，可视为同一结果。

这一节用的是 **DirectML** 后端，不是 TensorRT。原因是本机 PyTorch 为 CPU 版，
TensorRT 链路需要 CUDA 版 torch + TensorRT runtime + CUDA Toolkit 三者同时到位，
在这台机器上装不上（新版 PyTorch 的 DLL 被系统策略拦截，见 README 的 GPU 说明）。
DirectML 不依赖 torch 的 CUDA 构建，只吃 ONNX，因此能直接用上这块显卡，
用来验证「同一个模型换后端能快多少」这件事。
