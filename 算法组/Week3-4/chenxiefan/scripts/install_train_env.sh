#!/bin/bash

set -e

echo "== 1/3 安装 CPU 版 torch/torchvision（SJTU 镜像） =="
python3 -m pip install --user \
  --index-url https://mirror.sjtu.edu.cn/pytorch-wheels/cpu/ \
  --extra-index-url https://pypi.tuna.tsinghua.edu.cn/simple \
  torch torchvision

echo "== 2/3 安装 ultralytics（清华 PyPI） =="
python3 -m pip install --user -i https://pypi.tuna.tsinghua.edu.cn/simple ultralytics

echo "== 3/3 固定 setuptools，避免和 colcon 冲突 =="
python3 -m pip install --user -i https://pypi.tuna.tsinghua.edu.cn/simple "setuptools<80"

echo "== 验证 =="
python3 -c "import torch, ultralytics; print('torch', torch.__version__, 'cuda', torch.cuda.is_available()); print('ultralytics', ultralytics.__version__)"
echo "完成"
