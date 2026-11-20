"""极简 YAML 读写。

有 pyyaml 就用 pyyaml（推荐，`pip install pyyaml`）；
没有的话用内置小解析器，够读本工程的配置文件（嵌套字典 + 标量列表 + 内联列表）。
不支持 `- key: value` 这种列表里套字典的写法，遇到会提示你装 pyyaml。
"""

import os
from typing import Any


# ---------------------------------------------------------------- 读取

def load_yaml(path: str) -> Any:
    """读 yaml 文件，返回 dict / list / 标量。"""
    with open(path, "r", encoding="utf-8-sig") as f:
        text = f.read()
    try:
        import yaml  # type: ignore

        return yaml.safe_load(text)
    except ImportError:
        return _mini_load(text)


def _strip_comment(line: str) -> str:
    out = []
    in_s = in_d = False
    for i, ch in enumerate(line):
        if ch == "'" and not in_d:
            in_s = not in_s
        elif ch == '"' and not in_s:
            in_d = not in_d
        elif ch == "#" and not in_s and not in_d:
            if i == 0 or line[i - 1] in " \t":
                break
        out.append(ch)
    return "".join(out).rstrip()


def _tokenize(text: str):
    lines = []
    for raw in text.splitlines():
        line = _strip_comment(raw)
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip(" "))
        lines.append((indent, line.strip()))
    return lines


def _scalar(tok: str):
    tok = tok.strip()
    if not tok:
        return None
    if tok[0] in "'\"" and tok[-1] == tok[0] and len(tok) >= 2:
        return tok[1:-1]
    low = tok.lower()
    if low in ("true", "yes", "on"):
        return True
    if low in ("false", "no", "off"):
        return False
    if low in ("null", "none", "~"):
        return None
    if tok.startswith("[") and tok.endswith("]"):
        inner = tok[1:-1].strip()
        if not inner:
            return []
        return [_scalar(x) for x in inner.split(",")]
    try:
        return int(tok)
    except ValueError:
        pass
    try:
        return float(tok)
    except ValueError:
        pass
    return tok


def _mini_load(text: str):
    lines = _tokenize(text)
    if not lines:
        return None
    value, idx = _parse_block(lines, 0, lines[0][0])
    return value


def _parse_block(lines, idx, indent):
    if lines[idx][1].startswith("- "):
        return _parse_list(lines, idx, indent)
    return _parse_map(lines, idx, indent)


def _parse_list(lines, idx, indent):
    items = []
    while idx < len(lines) and lines[idx][0] == indent and lines[idx][1].startswith("-"):
        item = lines[idx][1][1:].strip()
        if not item:
            raise ValueError(
                "内置 YAML 解析器不支持空列表项 / 列表里嵌套结构，请安装 pyyaml：pip install pyyaml"
            )
        if ":" in item and not item.startswith("["):
            raise ValueError(
                "内置 YAML 解析器不支持「列表里放字典」的写法，请安装 pyyaml：pip install pyyaml"
            )
        items.append(_scalar(item))
        idx += 1
    return items, idx


def _parse_map(lines, idx, indent):
    out = {}
    while idx < len(lines) and lines[idx][0] == indent and not lines[idx][1].startswith("-"):
        text = lines[idx][1]
        if ":" not in text:
            raise ValueError("无法解析的 YAML 行: %r，请安装 pyyaml" % text)
        key, _, rest = text.partition(":")
        key = key.strip()
        rest = rest.strip()
        idx += 1
        if rest:
            out[key] = _scalar(rest)
            continue
        # 值在下一层
        if idx < len(lines) and lines[idx][0] > indent:
            out[key], idx = _parse_block(lines, idx, lines[idx][0])
        elif idx < len(lines) and lines[idx][0] == indent and lines[idx][1].startswith("-"):
            out[key], idx = _parse_list(lines, idx, indent)
        else:
            out[key] = None
    return out, idx


# ---------------------------------------------------------------- 写入

def _fmt(v) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, float):
        if v != v or v in (float("inf"), float("-inf")):
            return "nan"
        return repr(v)
    if v is None:
        return "null"
    if isinstance(v, (int,)):
        return str(v)
    if isinstance(v, str):
        return v if v and not any(c in v for c in ":#,[]{}\n") else '"%s"' % v
    if isinstance(v, (list, tuple)):
        return "[" + ", ".join(_fmt(x) for x in v) + "]"
    return str(v)


def _dump(obj, indent: int = 0) -> str:
    pad = "  " * indent
    lines = []
    if isinstance(obj, dict):
        for k, v in obj.items():
            if isinstance(v, dict):
                lines.append("%s%s:" % (pad, k))
                lines.append(_dump(v, indent + 1))
            elif isinstance(v, (list, tuple)) and v and not isinstance(v[0], (dict, list)):
                lines.append("%s%s:" % (pad, k))
                for item in v:
                    lines.append("%s  - %s" % (pad, _fmt(item)))
            else:
                lines.append("%s%s: %s" % (pad, k, _fmt(v)))
    elif isinstance(obj, (list, tuple)):
        for item in obj:
            lines.append("%s- %s" % (pad, _fmt(item)))
    else:
        lines.append("%s%s" % (pad, _fmt(obj)))
    return "\n".join(lines)


def save_yaml(path: str, data: Any) -> None:
    """写 yaml 文件（有 pyyaml 用它，保证格式规范）。"""
    try:
        import yaml  # type: ignore

        text = yaml.safe_dump(data, allow_unicode=True, sort_keys=False, default_flow_style=False)
    except ImportError:
        text = _dump(data) + "\n"
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
