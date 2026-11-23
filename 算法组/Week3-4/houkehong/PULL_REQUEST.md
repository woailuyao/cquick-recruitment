# 第 3–4 周合并考核 · HouKehong

CQuick 战队算法组考核提交目录。

## 本目录内容

| 路径 | 说明 |
|---|---|
| `README.md` | **主文档**：必填项、指标、算法链路、踩坑记录、局限 |
| `dataset/` | 数据集说明与 yaml（按细则要求，不上传抽帧图与标注图） |
| `calibration/` | 相机内参与外参标定结果 |
| `config/` | 场地地图、坐标系定义、相机参数（见 `config/README_config.md`） |
| `src/` | 核心算法模块 |
| `scripts/` | 标定 / 建图 / 采集 / 训练 / 复测脚本 |
| `run.py` | 统一入口 |
| `weights/WEIGHTS.md` | 权重的 SHA256 与获取方式 |
| `RECORDING_CHECKLIST.md` | 录屏操作清单 |
| `TRAINING_HANDOFF.md` | 模型训练迭代交接记录 |

## 快速验证

```powershell
# 不需要相机：合成闭环自检
python run.py demo

# 需要相机：实时定位
python run.py live --camera 1 --marker-weights weights/marker_best.pt
```

## B 站演示

https://www.bilibili.com/video/BV1teyGzCEd3
