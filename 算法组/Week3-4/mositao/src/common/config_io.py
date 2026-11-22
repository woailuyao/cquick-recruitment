# -*- coding: utf-8 -*-
"""YAML 配置读写：统一报错信息、统一相对路径显示、统一 SHA256 记录。

考核细则要求 README 记录 field.yaml / field_geometry.yaml 的版本与 SHA256，
因此这里顺带把指纹计算也放在同一处。
"""

from __future__ import annotations

import hashlib
from pathlib import Path

import yaml

from .paths import ROOT, rel


class ConfigError(RuntimeError):
    """配置文件缺失或格式不对时抛出，消息里只带相对路径。"""


def load_yaml(path) -> dict:
    p = Path(path)
    if not p.is_absolute():
        p = ROOT / p
    if not p.is_file():
        raise ConfigError(f"配置文件不存在: {rel(p)}")
    try:
        with p.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
    except yaml.YAMLError as exc:
        raise ConfigError(f"YAML 解析失败: {rel(p)} -> {exc}") from exc
    if not isinstance(data, dict):
        raise ConfigError(f"配置文件顶层应为映射(mapping): {rel(p)}")
    return data


def save_yaml(path, data: dict) -> None:
    p = Path(path)
    if not p.is_absolute():
        p = ROOT / p
    p.parent.mkdir(parents=True, exist_ok=True)
    with p.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)


def sha256_of(path, chunk: int = 1 << 20) -> str:
    p = Path(path)
    if not p.is_absolute():
        p = ROOT / p
    h = hashlib.sha256()
    with p.open("rb") as f:
        while True:
            block = f.read(chunk)
            if not block:
                break
            h.update(block)
    return h.hexdigest()


def pick(d: dict, *names, default=None):
    """按候选键名依次取值，用于兼容官方/自定字段命名的差异。"""
    for n in names:
        if isinstance(d, dict) and n in d and d[n] is not None:
            return d[n]
    return default
