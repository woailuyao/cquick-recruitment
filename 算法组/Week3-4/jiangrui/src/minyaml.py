# -*- coding: utf-8 -*-
"""最小 YAML 读取器。

本项目只用到很少的 YAML 语法（标量、嵌套映射、一维列表、行尾注释）。
考核机房的 conda 环境不一定装了 PyYAML，所以这里给一个零依赖的兜底实现：
装了 PyYAML 就用 PyYAML，没装就用本文件。

支持的写法:
    key: 123
    key: 1.5
    key: [1.0, 2.0, 3.0]
    key: 'not_calibrated'
    key: null
    parent:
      child: 1
    list:
      - 1
      - name: T
        pixel: [640, 480]
"""

import json
import os

try:  # 优先使用官方实现
    import yaml as _pyyaml
except Exception:  # pragma: no cover - 兜底分支
    _pyyaml = None


def _strip_comment(line):
    """去掉引号外的行尾注释。"""
    out = []
    quote = None
    for ch in line:
        if quote is not None:
            out.append(ch)
            if ch == quote:
                quote = None
        else:
            if ch in "\"'":
                quote = ch
                out.append(ch)
            elif ch == "#":
                break
            else:
                out.append(ch)
    return "".join(out).rstrip()


def _split_commas(text):
    parts, buf, depth = [], [], 0
    quote = None
    for ch in text:
        if quote is not None:
            buf.append(ch)
            if ch == quote:
                quote = None
            continue
        if ch in "\"'":
            quote = ch
            buf.append(ch)
        elif ch in "[{(":
            depth += 1
            buf.append(ch)
        elif ch in "]})":
            depth -= 1
            buf.append(ch)
        elif ch == "," and depth == 0:
            parts.append("".join(buf))
            buf = []
        else:
            buf.append(ch)
    if buf:
        parts.append("".join(buf))
    return [p.strip() for p in parts if p.strip() != ""]


def _scalar(text):
    s = text.strip()
    if s == "":
        return ""
    if s.startswith("[") and s.endswith("]"):
        inner = s[1:-1].strip()
        return [] if not inner else [_scalar(x) for x in _split_commas(inner)]
    if len(s) >= 2 and s[0] == s[-1] and s[0] in "\"'":
        return s[1:-1]
    low = s.lower()
    if low in ("null", "~", "none"):
        return None
    if low in ("true", "yes"):
        return True
    if low in ("false", "no"):
        return False
    try:
        if low.startswith("0x"):
            return int(s, 16)
        if "." not in s and "e" not in low:
            return int(s)
        return float(s)
    except ValueError:
        return s


def _split_kv(line):
    idx = line.find(":")
    if idx < 0:
        return line.strip(), ""
    return line[:idx].strip(), line[idx + 1:].strip()


def _parse_block(lines, i, indent):
    # 列表
    if lines[i][1].startswith("-"):
        items = []
        while i < len(lines) and lines[i][0] == indent and lines[i][1].startswith("-"):
            body = lines[i][1][1:].strip()
            if body == "":
                i += 1
                if i < len(lines) and lines[i][0] > indent:
                    value, i = _parse_block(lines, i, lines[i][0])
                else:
                    value = None
                items.append(value)
                continue
            if ":" not in body or body.startswith(("[", "'", '"')):
                # 纯标量项
                items.append(_scalar(body))
                i += 1
                continue
            key, value = _split_kv(body)
            # 形如 "- key: value" 的映射项
            item = {}
            if value == "":
                i += 1
                if i < len(lines) and lines[i][0] > indent:
                    item[key], i = _parse_block(lines, i, lines[i][0])
                else:
                    item[key] = None
            else:
                item[key] = _scalar(value)
                i += 1
            # 同一项的后续键（缩进大于列表缩进）
            while i < len(lines) and lines[i][0] > indent and not lines[i][1].startswith("-"):
                k, v = _split_kv(lines[i][1])
                if v == "":
                    i += 1
                    if i < len(lines) and lines[i][0] > indent:
                        item[k], i = _parse_block(lines, i, lines[i][0])
                    else:
                        item[k] = None
                else:
                    item[k] = _scalar(v)
                    i += 1
            items.append(item)
        return items, i

    # 映射
    out = {}
    while i < len(lines) and lines[i][0] == indent:
        key, value = _split_kv(lines[i][1])
        if value == "":
            i += 1
            if i < len(lines) and lines[i][0] > indent:
                out[key], i = _parse_block(lines, i, lines[i][0])
            else:
                out[key] = None
        else:
            out[key] = _scalar(value)
            i += 1
    return out, i


def loads(text):
    if _pyyaml is not None:
        return _pyyaml.safe_load(text)
    lines = []
    for raw in text.splitlines():
        cleaned = _strip_comment(raw)
        if not cleaned.strip():
            continue
        indent = len(cleaned) - len(cleaned.lstrip(" "))
        lines.append((indent, cleaned.strip()))
    if not lines:
        return {}
    value, _ = _parse_block(lines, 0, lines[0][0])
    return value


def load(path):
    """读取 .yaml / .yml / .json 文件。"""
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    with open(path, "r", encoding="utf-8-sig") as f:
        text = f.read()
    if path.lower().endswith(".json"):
        return json.loads(text)
    return loads(text)
