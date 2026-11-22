# -*- coding: utf-8 -*-
"""
match_map.py
============
场地标志地图加载 + 观测特征匹配 + 载体二维位姿估计（单目，地面 Z=0 假设）。

坐标约定（全工程统一，务必与 README 一致）
------------------------------------------------
1) 载体/相机地面坐标系（rig frame，观测点使用的坐标系）
   原点 O_b = 相机光心在地面 (Z=0) 上的投影点；
   +X_b = 相机水平前向（光轴在水平面内的投影方向）；
   +Y_b = 相机左方（右手系，+Z_b 向上）。
   观测点 P_b = (x_b, y_b) 单位为米，由 main_app 的像素->地面投影给出。

2) 地图坐标系（map frame，field.yaml）
   +X_map 向右，+Y_map 向上，yaw 从 +X_map 逆时针为正，单位为度。

3) 位姿模型
   P_map = R(theta) * P_b + t
   其中 theta = yaw_camera_map（相机水平朝向相对地图 +X 的角度），
         t = (x_camera_map, y_camera_map)（相机光心的地面投影在地图中的位置）。
   本工程把"载体参考原点"直接定义为相机光心的地面投影（相机装在移动测试架上），
   因此 carrier 与 camera 位姿相同；若以后加机器人底座外参，只需在 main_app 里
   对 (x, y, yaw) 再做一次固定变换即可。

本文件不依赖 PyYAML（有就用，没有就用内置的极简 YAML 解析器），
以免实验室机器没装 pyyaml 直接跑不起来。
"""

from __future__ import annotations

import math
import os

import numpy as np

# ----------------------------------------------------------------------------
# 类别定义（与 marker_data.yaml / 模型 names 对应）
# ----------------------------------------------------------------------------
CLASS_NAMES = {
    0: "LCross",
    1: "TCross",
    2: "XCross",
    3: "PenaltyPoint",
}
CLASS_SHORT = {0: "L", 1: "T", 2: "X", 3: "P"}
CLASS_COLORS = {  # BGR，用于画图
    0: (255, 160, 0),    # L：橙
    1: (0, 220, 255),    # T：黄
    2: (255, 0, 255),    # X：品红
    3: (0, 255, 0),      # 点球点：绿
}

# ----------------------------------------------------------------------------
# 内置兜底地图（坐标与原来 match_map.py 的 map_markers 完全一致）。
# 正常情况下会优先加载 config/field.yaml（组织方统一发放的版本）。
# ----------------------------------------------------------------------------
DEFAULT_MAP = [
    {"cls": 2, "x": 0.0, "y": 0.75, "name": "x1"},
    {"cls": 2, "x": 0.0, "y": -0.75, "name": "x2"},
    {"cls": 1, "x": 0.0, "y": 3.0, "name": "t1"},
    {"cls": 1, "x": 0.0, "y": -3.0, "name": "t2"},
    {"cls": 1, "x": 3.96, "y": 1.95, "name": "t3"},
    {"cls": 1, "x": 3.96, "y": -1.95, "name": "t4"},
    {"cls": 1, "x": -3.96, "y": 1.95, "name": "t5"},
    {"cls": 1, "x": -3.96, "y": -1.95, "name": "t6"},
    {"cls": 1, "x": 3.96, "y": 1.5, "name": "t7"},
    {"cls": 1, "x": 3.96, "y": -1.5, "name": "t8"},
    {"cls": 1, "x": -3.96, "y": 1.5, "name": "t9"},
    {"cls": 1, "x": -3.96, "y": -1.5, "name": "t10"},
    {"cls": 0, "x": 2.05, "y": 2.0, "name": "l1"},
    {"cls": 0, "x": 2.05, "y": -2.0, "name": "l2"},
    {"cls": 0, "x": 3.0, "y": 1.49, "name": "l3"},
    {"cls": 0, "x": 3.0, "y": -1.49, "name": "l4"},
    {"cls": 0, "x": -2.05, "y": 2.0, "name": "l5"},
    {"cls": 0, "x": -2.05, "y": -2.0, "name": "l6"},
    {"cls": 0, "x": -3.0, "y": 1.49, "name": "l7"},
    {"cls": 0, "x": -3.0, "y": -1.49, "name": "l8"},
]

# 匹配参数默认值（可在 main_app 里覆盖）
DEFAULT_CFG = {
    "pair_dist_tol": 0.60,   # 两个观测的间距 与 两个地图点间距 的允许差(m)
    "inlier_tol": 0.35,      # 单个观测点到地图点的最大距离，小于则算内点(m)
    # 至少几个内点才认为位姿可信。1 个点只能定距离、定不出朝向（官方细则明确说不够）；
    # 2 个点能定出位姿但存在对称多解，所以只在"有上一帧先验"时才算有效。
    "min_inliers": 2,
    "min_spread": 0.25,      # 内点基线下限(m)：球门线上两个相邻 T 只差 0.45 m，太严会永远不出结果
    "max_residual": 0.40,    # 内点 RMS 残差上限(m)
    # 只有 2 个内点时残差≈两标志间距误差的一半，另加更严的上限
    "max_residual_2": 0.35,
    "max_candidates": 600,   # 候选位姿数量上限（按间距一致程度截断）
    "prior_gate": 1.50,      # 上一帧位姿作为先验的搜索半径(m)
    "tie_margin": 0.10,      # 判定"并列最优"的残差余量(m)
}

_BASE_DIR = os.path.dirname(os.path.abspath(__file__))
FIELD_FILE_CANDIDATES = [
    os.path.join(_BASE_DIR, "config", "field.yaml"),
    os.path.join(_BASE_DIR, "field.yaml"),
    # 提交目录结构：src/match_map.py -> ../config/field.yaml
    os.path.join(_BASE_DIR, os.pardir, "config", "field.yaml"),
    os.path.join(os.getcwd(), "config", "field.yaml"),
    os.path.join(os.getcwd(), "field.yaml"),
]


# ============================================================================
# 1. 极简 YAML 读取（有 PyYAML 时优先使用 PyYAML）
# ============================================================================
try:  # pragma: no cover
    import yaml as _pyyaml
except Exception:  # PyYAML 没装：用下面的内置解释器
    _pyyaml = None


def _strip_comment(line):
    out = []
    quote = None
    for i, ch in enumerate(line):
        if quote:
            out.append(ch)
            if ch == quote:
                quote = None
        elif ch in "'\"":
            quote = ch
            out.append(ch)
        elif ch == "#" and (i == 0 or line[i - 1] in " \t"):
            break
        else:
            out.append(ch)
    return "".join(out)


def _split_flow(text):
    """按顶层逗号切分 flow 列表/字典内部内容。"""
    parts, buf, depth, quote = [], [], 0, None
    for ch in text:
        if quote:
            buf.append(ch)
            if ch == quote:
                quote = None
            continue
        if ch in "'\"":
            quote = ch
            buf.append(ch)
        elif ch in "[{":
            depth += 1
            buf.append(ch)
        elif ch in "]}":
            depth -= 1
            buf.append(ch)
        elif ch == "," and depth == 0:
            parts.append("".join(buf))
            buf = []
        else:
            buf.append(ch)
    if "".join(buf).strip():
        parts.append("".join(buf))
    return parts


def _parse_scalar(text):
    s = text.strip()
    if not s:
        return None
    if len(s) >= 2 and s[0] == s[-1] and s[0] in "'\"":
        return s[1:-1]
    low = s.lower()
    if low in ("true", "yes", "on"):
        return True
    if low in ("false", "no", "off"):
        return False
    if low in ("null", "none", "~"):
        return None
    if s.startswith("[") and s.endswith("]"):
        return [_parse_scalar(p) for p in _split_flow(s[1:-1])]
    if s.startswith("{") and s.endswith("}"):
        d = {}
        for part in _split_flow(s[1:-1]):
            k, sep, v = part.partition(":")
            if not sep:
                continue
            d[_parse_scalar(k)] = _parse_scalar(v)
        return d
    try:
        return int(s)
    except ValueError:
        pass
    try:
        return float(s)
    except ValueError:
        return s


def _split_key(text):
    """把 'key: value' 拆成 (key, value_text)，跳过引号/括号内的冒号。"""
    depth, quote = 0, None
    for i, ch in enumerate(text):
        if quote:
            if ch == quote:
                quote = None
            continue
        if ch in "'\"":
            quote = ch
        elif ch in "[{":
            depth += 1
        elif ch in "]}":
            depth -= 1
        elif ch == ":" and depth == 0:
            return text[:i].strip(), text[i + 1:].strip()
    return text.strip(), ""


def _looks_like_mapping(text):
    if text[:1] in "[{":
        return False
    key, sep = _split_key(text)
    return bool(sep) and bool(key) and " " not in key.strip("\"'")


def _simple_yaml_load(text):
    """够用的 YAML 子集解析：缩进嵌套、'- ' 列表、flow 列表/字典、注释。"""
    lines = []
    for raw in text.splitlines():
        line = _strip_comment(raw).rstrip()
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip(" "))
        lines.append((indent, line.strip()))
    if not lines:
        return None

    def parse_block(i, indent):
        if lines[i][1].startswith("- "):
            out = []
            while i < len(lines) and lines[i][0] == indent and lines[i][1].startswith("-"):
                rest = lines[i][1][1:].strip()
                i += 1
                if not rest:
                    if i < len(lines) and lines[i][0] > indent:
                        val, i = parse_block(i, lines[i][0])
                    else:
                        val = None
                    out.append(val)
                elif _looks_like_mapping(rest):
                    key, val_txt = _split_key(rest)
                    item = {}
                    if val_txt:
                        item[key] = _parse_scalar(val_txt)
                    elif i < len(lines) and lines[i][0] > indent:
                        item[key], i = parse_block(i, lines[i][0])
                    else:
                        item[key] = None
                    while i < len(lines) and lines[i][0] > indent and not lines[i][1].startswith("-"):
                        k2, v2 = _split_key(lines[i][1])
                        cur_indent = lines[i][0]
                        i += 1
                        if v2:
                            item[k2] = _parse_scalar(v2)
                        elif i < len(lines) and lines[i][0] > cur_indent:
                            item[k2], i = parse_block(i, lines[i][0])
                        else:
                            item[k2] = None
                    out.append(item)
                else:
                    out.append(_parse_scalar(rest))
            return out, i
        out = {}
        while i < len(lines) and lines[i][0] == indent and not lines[i][1].startswith("-"):
            key, val_txt = _split_key(lines[i][1])
            i += 1
            if val_txt:
                out[key] = _parse_scalar(val_txt)
            elif i < len(lines) and lines[i][0] > indent:
                out[key], i = parse_block(i, lines[i][0])
            else:
                out[key] = None
        return out, i

    start = lines[0][0]
    value, _ = parse_block(0, start)
    return value


def load_yaml_file(path):
    with open(path, "r", encoding="utf-8-sig") as f:
        text = f.read()
    if _pyyaml is not None:
        try:
            return _pyyaml.safe_load(text)
        except Exception:
            pass
    return _simple_yaml_load(text)


# ============================================================================
# 2. 标定文件 / 地图文件加载
# ============================================================================
def _matrix_from_node(node, rows, cols):
    """兼容 OpenCV FileStorage 风格 {rows,cols,data} 与嵌套列表两种写法。"""
    if isinstance(node, dict):
        data = node.get("data")
        if data is None:
            return None
        rows = int(node.get("rows", rows) or rows)
        cols = int(node.get("cols", cols) or cols)
        return np.array(data, dtype=float).reshape(rows, cols)
    if isinstance(node, (list, tuple)):
        arr = np.array(node, dtype=float)
        if arr.shape == (rows, cols):
            return arr
        if arr.size == rows * cols:
            return arr.reshape(rows, cols)
    return None


def load_intrinsics(path):
    """
    读取 camera_intrinsics.yaml。
    返回 dict(image_width, image_height, K, dist, source)；文件不存在/解析失败返回 None。
    """
    if not path or not os.path.isfile(path):
        return None
    try:
        cfg = load_yaml_file(path)
    except Exception:
        return None
    if not isinstance(cfg, dict):
        return None

    K = _matrix_from_node(cfg.get("camera_matrix"), 3, 3)
    dist = _matrix_from_node(cfg.get("distortion_coefficients") or cfg.get("dist_coeffs"), 1, 5)
    if K is None:
        return None
    if dist is None:
        dist = np.zeros((1, 5), dtype=float)
    dist = np.asarray(dist, dtype=float).reshape(-1)

    w = cfg.get("image_width") or cfg.get("width") or 0
    h = cfg.get("image_height") or cfg.get("height") or 0
    try:
        w, h = int(w), int(h)
    except (TypeError, ValueError):
        w, h = 0, 0
    return {"image_width": w, "image_height": h, "K": K, "dist": dist,
            "source": os.path.basename(path)}


def load_extrinsics(path):
    """
    读取相机安装外参 camera_extrinsics.yaml。

    返回 dict(height, pitch_deg, roll_deg, source)；找不到/解析失败返回 None，
    由调用方回退到默认常量。兼容常见键名，方便手写或组长发放的模板。
    注意：height/pitch 只有在相机按固定安装架好之后测的值才有意义，
    训练集是手持随手拍的，不能用训练图片反推这两个值。
    """
    if not path or not os.path.isfile(path):
        return None
    try:
        cfg = load_yaml_file(path)
    except Exception:
        return None
    if not isinstance(cfg, dict):
        return None

    def pick(*keys):
        for k in keys:
            for kk in (k, k.lower(), k.upper()):
                if kk in cfg:
                    try:
                        return float(cfg[kk])
                    except (TypeError, ValueError):
                        continue
        return None

    height = pick("camera_height_m", "camera_height", "height_m", "height", "z_camera_m")
    pitch = pick("pitch_deg", "pitch", "camera_pitch_deg", "tilt_deg")
    roll = pick("roll_deg", "roll", "camera_roll_deg")
    if height is None and pitch is None:
        return None
    return {"height": height, "pitch_deg": pitch,
            "roll_deg": roll if roll is not None else 0.0,
            "source": os.path.abspath(path)}


def _norm_name(name):
    """'LCross' / 'l_cross' / 'L' / 'PenalityPoint' 统一成小写字母串。"""
    if name is None:
        return ""
    s = str(name).strip().lower()
    return "".join(ch for ch in s if ch.isalnum())


_NAME_TO_CLS = {
    "l": 0, "lcross": 0, "lcrossing": 0, "lmark": 0, "corner": 0, "lcorner": 0,
    "t": 1, "tcross": 1, "tcrossing": 1, "tmark": 1,
    "x": 2, "xcross": 2, "xcrossing": 2, "xmark": 2, "cross": 2,
    "p": 3, "penalty": 3, "penaltypoint": 3,
    "penality": 3, "penalitypoint": 3,  # 官方 field.yaml 的拼写
    "penaltymark": 3, "point": 3,
}


def class_id_from_name(name):
    """类别名 -> 类别 id；认不出返回 None。"""
    key = _norm_name(name)
    if key in _NAME_TO_CLS:
        return _NAME_TO_CLS[key]
    for token, cid in _NAME_TO_CLS.items():
        if len(token) > 2 and token in key:
            return cid
    return None


def _is_number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _collect_points(node, inherited_name, out):
    """在任意嵌套结构里找出所有 {x, y, 类别名} 点。"""
    if isinstance(node, dict):
        x = node.get("x", node.get("X"))
        y = node.get("y", node.get("Y"))
        name = (node.get("type") or node.get("class") or node.get("cls")
                or node.get("name") or node.get("label") or node.get("kind")
                or inherited_name)
        if x is not None and y is not None and _is_number(x) and _is_number(y):
            cid = class_id_from_name(name)
            if cid is not None:
                out.append({"cls": cid, "x": float(x), "y": float(y),
                            "name": str(name).strip() or CLASS_NAMES[cid]})
                return
        for key, value in node.items():
            _collect_points(value, key, out)
    elif isinstance(node, (list, tuple)):
        if (len(node) == 2 and all(_is_number(v) for v in node)
                and class_id_from_name(inherited_name) is not None):
            cid = class_id_from_name(inherited_name)
            out.append({"cls": cid, "x": float(node[0]), "y": float(node[1]),
                        "name": CLASS_NAMES[cid]})
            return
        for item in node:
            _collect_points(item, inherited_name, out)


def load_field(paths=None, verbose=True):
    """
    加载 field.yaml 中的标志点。
    返回 (markers, source)。
    markers = [{'cls': int, 'x': float, 'y': float, 'name': str}, ...]

    找不到可用地图时返回内置 DEFAULT_MAP（并给出提示），保证程序不会因为缺文件崩掉。
    """
    if paths is None:
        paths = FIELD_FILE_CANDIDATES
    elif isinstance(paths, str):
        paths = [paths]

    for path in paths:
        if not path or not os.path.isfile(path):
            continue
        try:
            data = load_yaml_file(path)
        except Exception as exc:
            if verbose:
                print(f"[map] 读取 {path} 失败：{exc}")
            continue
        raw = []
        _collect_points(data, None, raw)
        if not raw:
            if verbose:
                print(f"[map] {path} 里没有找到可用的 (x, y, 类别) 标志点，跳过。")
            continue

        # 去重（同一坐标同一类别只留一个）
        seen, markers = set(), []
        for m in raw:
            key = (m["cls"], round(m["x"], 4), round(m["y"], 4))
            if key in seen:
                continue
            seen.add(key)
            markers.append(m)

        # 统一生成稳定的显示名（field.yaml 里的 name 常常只是类别名）
        counters = {}
        for m in markers:
            short = CLASS_SHORT[m["cls"]]
            if not m["name"] or _norm_name(m["name"]) in _NAME_TO_CLS:
                counters[short] = counters.get(short, 0) + 1
                m["name"] = f"{short}{counters[short]}"
        if verbose:
            stat = ", ".join(f"{CLASS_NAMES[c]}:{sum(1 for m in markers if m['cls'] == c)}"
                             for c in sorted({m["cls"] for m in markers}))
            print(f"[map] 已加载地图：{os.path.abspath(path)}  共 {len(markers)} 个标志点 ({stat})")
        return markers, os.path.abspath(path)

    if verbose:
        print("[map] 未找到 field.yaml，使用 match_map.py 内置兜底地图（坐标与原来一致）。")
    return [dict(m) for m in DEFAULT_MAP], "<built-in>"


# ============================================================================
# 3. 二维位姿估计
# ============================================================================
def _wrap_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def _rot2(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=float)


def _empty_result(status, reason=""):
    return {
        "valid": False,
        "x": 0.0,
        "y": 0.0,
        "yaw_deg": 0.0,
        "residual": float("inf"),
        "marker_count": 0,
        "status": status,
        "matches": [],       # [{'obs': i, 'map': idx, 'dist': d}]
        "obs_map": [],       # 每个观测点投影到地图后的坐标（用于画图）
        "groups": [],        # 所有候选位姿（多解时给 main_app 展示 / 人工消歧）
        "prior_used": False,
        "tie_resolved": False,
        "ambiguous": False,
        "degraded": False,
        "reason": reason or status,
    }


def _assign(obs, th, t, cls_ids, cls_pts, inlier_tol):
    """给定位姿，对每条观测贪心分配最近的同类地图点，返回 (matched, rms)。"""
    R = _rot2(th)
    matched = {}
    total = 0.0
    for k, o in enumerate(obs):
        P = R @ np.array([o["x"], o["y"]], dtype=float) + t
        pts = cls_pts.get(o["cls"])
        if pts is None or len(pts) == 0:
            continue
        d = np.hypot(pts[:, 0] - P[0], pts[:, 1] - P[1])
        order = np.argsort(d)[:3]  # 最多试最近 3 个，避免"抢点"
        for j in order:
            if d[j] > inlier_tol:
                break
            mid = int(cls_ids[o["cls"]][j])
            if mid in matched:
                continue
            matched[mid] = (k, float(d[j]))
            total += float(d[j]) ** 2
            break
    if not matched:
        return matched, float("inf")
    return matched, math.sqrt(total / len(matched))


def _refine(obs, matched, markers):
    """用内点做 2D 刚体最小二乘（Umeyama，不含尺度），得到更精确的 (theta, t)。"""
    P = np.array([[obs[k]["x"], obs[k]["y"]] for k, _ in matched.values()], dtype=float)
    M = np.array([[markers[mid]["x"], markers[mid]["y"]] for mid in matched], dtype=float)
    pc, mc = P.mean(axis=0), M.mean(axis=0)
    P0, M0 = P - pc, M - mc
    num = float(np.sum(P0[:, 0] * M0[:, 1] - P0[:, 1] * M0[:, 0]))
    den = float(np.sum(P0[:, 0] * M0[:, 0] + P0[:, 1] * M0[:, 1]))
    if abs(num) < 1e-12 and abs(den) < 1e-12:
        return None, None
    th = math.atan2(num, den)
    R = _rot2(th)
    return th, mc - R @ pc


def estimate_pose(observed, markers=None, last_pose=None, cfg=None):
    """
    观测特征 -> 地图位姿。

    参数
    ----
    observed : [{'cls': int, 'x': float, 'y': float, 'conf': float}, ...]
               载体坐标系下的地面坐标（米），来自像素->地面投影。
    markers  : 地图标志点列表（load_field 的返回值）。
    last_pose: 上一帧 (x, y, yaw_deg) 或 None；仅用于在"多解并列"时消歧（先验）。
    cfg      : 覆盖 DEFAULT_CFG 的字典。

    返回
    ----
    dict: valid / x / y / yaw_deg / residual / marker_count / status /
          matches / obs_map / groups / prior_used
    """
    cfg = dict(DEFAULT_CFG, **(cfg or {}))
    markers = DEFAULT_MAP if markers is None else markers

    # ---- 观测过滤 ----
    obs = []
    for o in observed:
        try:
            x, y = float(o["x"]), float(o["y"])
            cid = int(o["cls"])
        except (KeyError, TypeError, ValueError):
            continue
        if not (math.isfinite(x) and math.isfinite(y)):
            continue
        if cid not in CLASS_NAMES:
            continue
        obs.append({"cls": cid, "x": x, "y": y, "conf": float(o.get("conf", 1.0))})

    if len(obs) < 2:
        return _empty_result("insufficient_markers",
                             f"only {len(obs)} marker(s) with ground point, need 2")

    # ---- 地图按类别索引 ----
    by_cls = {}
    for idx, m in enumerate(markers):
        by_cls.setdefault(m["cls"], []).append(idx)
    if not by_cls:
        return _empty_result("lost")

    # T 与 X 都是"线交点"标志，模型经常互相认错（特别是 X 被认成 T），
    # 所以匹配时允许 T/X 互换；L 的几何形态差别大，不放开。
    def _compat(c):
        return (1, 2) if c in (1, 2) else (c,)

    compat_ids, compat_pts = {}, {}
    for c in set(list(by_cls.keys()) + [0, 1, 2, 3]):
        ids = [i for i, m in enumerate(markers) if m["cls"] in _compat(c)]
        if ids:
            compat_ids[c] = np.array(ids, dtype=int)
            compat_pts[c] = np.array([[markers[i]["x"], markers[i]["y"]] for i in ids],
                                     dtype=float)

    inlier_tol = float(cfg["inlier_tol"])
    pair_tol = float(cfg["pair_dist_tol"])

    # ---- 1) 两两组合生成候选位姿 ----
    cands = []
    nearest_mismatch = float("inf")
    n = len(obs)
    best_pair = None
    best_mis = float("inf")
    for i in range(n):
        for j in range(i + 1, n):
            pi = np.array([obs[i]["x"], obs[i]["y"]], dtype=float)
            pj = np.array([obs[j]["x"], obs[j]["y"]], dtype=float)
            dp = pj - pi
            d_obs = float(np.hypot(dp[0], dp[1]))
            if d_obs < 0.15:  # 两个观测几乎重合，定不出角度
                continue
            ang_obs = math.atan2(dp[1], dp[0])
            for a in compat_ids.get(obs[i]["cls"], []):
                ma = markers[int(a)]
                for b in compat_ids.get(obs[j]["cls"], []):
                    if a == b:
                        continue
                    mb = markers[int(b)]
                    d_map = math.hypot(mb["x"] - ma["x"], mb["y"] - ma["y"])
                    mismatch = abs(d_obs - d_map)
                    if mismatch < nearest_mismatch:
                        nearest_mismatch = mismatch
                    ang_map = math.atan2(mb["y"] - ma["y"], mb["x"] - ma["x"])
                    th = ang_map - ang_obs
                    R = _rot2(th)
                    t = np.array([ma["x"], ma["y"]], dtype=float) - R @ pi
                    if mismatch < best_mis:
                        best_mis = mismatch
                        best_pair = (mismatch, th, t)
                    if mismatch > pair_tol:
                        continue
                    cands.append((mismatch, th, t))

    # 没有任何一对地图点与观测间距吻合：退而求其次，用"最接近的那一对"生成一个
    # 最可能解（degraded）。宁可给一个标注为"低可信"的坐标，也不要在有三个点时
    # 完全没有任何输出。
    degraded = False
    if not cands:
        if best_pair is None:
            return _empty_result("not_converged", "no compatible map marker pair")
        cands = [best_pair]
        degraded = True

    # 间距吻合度越好越可能是真解，限制数量保证实时
    cands.sort(key=lambda c: c[0])
    cands = cands[:int(cfg["max_candidates"])]

    # ---- 2) 逐候选评估内点，按姿态格子去重 ----
    groups = {}
    for mismatch, th, t in cands:
        matched, rms = _assign(obs, th, t, compat_ids, compat_pts, inlier_tol)
        cnt = len(matched)
        if cnt < 2:
            continue
        key = (int(round(t[0] / 0.4)), int(round(t[1] / 0.4)),
               int(round(math.degrees(th) / 15.0)))
        cur = groups.get(key)
        if cur is None or (cnt, -rms) > (cur["inliers"], -cur["residual"]):
            groups[key] = {"th": th, "t": t, "inliers": cnt, "residual": rms,
                           "matched": matched}

    if not groups:
        return _empty_result("not_converged", "candidates exist but no consistent assignment")

    # ---- 3) 内点精化 + 重新评估（两轮） ----
    refined = []
    for g in groups.values():
        th, t = g["th"], g["t"]
        matched = g["matched"]
        rms = g["residual"]
        for _ in range(2):
            th_new, t_new = _refine(obs, matched, markers)
            if th_new is None:
                break
            m2, rms2 = _assign(obs, th_new, t_new, compat_ids, compat_pts, inlier_tol)
            if len(m2) < len(matched):
                break  # 精化后内点变少，回退
            th, t, matched, rms = th_new, t_new, m2, rms2
        g["th"], g["t"], g["matched"], g["residual"] = th, t, matched, rms
        g["inliers"] = len(matched)
        refined.append(g)

    # ---- 4) 基线检查：内点不能挤成一团 ----
    valid_groups = []
    too_short = 0
    best_short_spread = 0.0
    for g in refined:
        if g["inliers"] < 2:
            continue
        idxs = [k for k, _ in g["matched"].values()]
        pts = np.array([[obs[k]["x"], obs[k]["y"]] for k in idxs])
        spread = float(np.max(np.hypot(pts[:, None, 0] - pts[None, :, 0],
                                       pts[:, None, 1] - pts[None, :, 1])))
        g["spread"] = spread
        if g["inliers"] >= int(cfg["min_inliers"]) and spread < float(cfg["min_spread"]):
            too_short += 1
            best_short_spread = max(best_short_spread, spread)
            continue  # 基线太短，角度不可信
        valid_groups.append(g)

    if not valid_groups:
        if too_short:
            return _empty_result(
                "baseline_too_short",
                f"baseline {best_short_spread:.2f} m < {float(cfg['min_spread']):.2f} m")
        return _empty_result("insufficient_markers")

    valid_groups.sort(key=lambda g: (-g["inliers"], g["residual"]))

    def _apart(g1, g2):
        dt = math.hypot(g1["t"][0] - g2["t"][0], g1["t"][1] - g2["t"][1])
        dyaw = abs(_wrap_deg(math.degrees(g1["th"] - g2["th"])))
        return dt > 0.6 or dyaw > 15.0

    best = valid_groups[0]
    chosen = best
    prior_used = False

    # ---- 5) 先验消歧（连续帧跟踪）：在合理候选里挑最接近上一帧位姿的那个 ----
    if last_pose is not None:
        try:
            px, py = float(last_pose[0]), float(last_pose[1])
            pyaw = float(last_pose[2])
        except (TypeError, IndexError, ValueError):
            px = py = pyaw = None
        if px is not None:
            gate = float(cfg["prior_gate"])

            def _prior_cost(g):
                d = math.hypot(g["t"][0] - px, g["t"][1] - py)
                dyaw = abs(_wrap_deg(math.degrees(g["th"]) - pyaw))
                return (d / gate) ** 2 + (dyaw / 30.0) ** 2, d, dyaw

            near = []
            for g in valid_groups:
                if g["inliers"] < max(int(cfg["min_inliers"]), best["inliers"] - 1):
                    continue
                if g["residual"] > best["residual"] + float(cfg["tie_margin"]):
                    continue
                cost, d, dyaw = _prior_cost(g)
                if d <= gate and dyaw <= 45.0:
                    near.append((cost, -g["inliers"], g["residual"], g))
            if near:
                near.sort(key=lambda item: item[:3])
                chosen = near[0][3]
                prior_used = chosen is not best

    # ---- 6) 歧义判定：还有没有同样好、但位置明显不同的解 ----
    tie = None
    for g in valid_groups:
        if g is chosen:
            continue
        if g["inliers"] < chosen["inliers"]:
            continue
        if g["residual"] > chosen["residual"] + float(cfg["tie_margin"]):
            continue
        if _apart(g, chosen):
            tie = g
            break

    # 有上一帧先验、且选中的解就在先验附近时：对称多解由连续帧一致性消解。
    prior_resolved = False
    if last_pose is not None and tie is not None:
        try:
            px, py = float(last_pose[0]), float(last_pose[1])
            d_prior = math.hypot(float(chosen["t"][0]) - px, float(chosen["t"][1]) - py)
            prior_resolved = d_prior <= float(cfg["prior_gate"])
        except (TypeError, IndexError, ValueError):
            prior_resolved = False

    # 输出候选（按好坏排序，给 main_app 按键循环消歧用；同一解只留一个）
    groups_out = []
    for g in valid_groups:
        gx, gy = float(g["t"][0]), float(g["t"][1])
        gyaw = _wrap_deg(math.degrees(g["th"]))
        dup = False
        for o in groups_out:
            if (math.hypot(o["x"] - gx, o["y"] - gy) < 0.35
                    and abs(_wrap_deg(o["yaw_deg"] - gyaw)) < 10.0):
                dup = True
                break
        if dup:
            continue
        groups_out.append({
            "x": gx,
            "y": gy,
            "yaw_deg": gyaw,
            "marker_count": int(g["inliers"]),
            "residual": float(g["residual"]),
        })
        if len(groups_out) >= 6:
            break

    yaw_deg = _wrap_deg(math.degrees(chosen["th"]))
    x_cam, y_cam = float(chosen["t"][0]), float(chosen["t"][1])

    obs_map = []
    R = _rot2(chosen["th"])
    for o in obs:
        P = R @ np.array([o["x"], o["y"]], dtype=float) + chosen["t"]
        obs_map.append([float(P[0]), float(P[1])])

    matches = [{"obs": int(k), "map": int(mid), "dist": float(d)}
               for mid, (k, d) in chosen["matched"].items()]

    result = {
        "valid": False,
        "x": x_cam,
        "y": y_cam,
        "yaw_deg": yaw_deg,
        "residual": float(chosen["residual"]),
        "marker_count": int(chosen["inliers"]),
        "status": "ok",
        "matches": matches,
        "obs_map": obs_map,
        "groups": groups_out,
        "prior_used": bool(prior_used),
        "tie_resolved": bool(prior_resolved),
        "ambiguous": bool(tie is not None and not prior_resolved),
        "degraded": bool(degraded),
        "reason": "",
    }

    # ---- 7) 有效性判定（绝不"为了画面连续"伪造有效位姿）----
    # 2 个内点时残差门槛更严：错误但"间距相近"的标志对通常残差偏大
    res_limit = float(cfg["max_residual"])
    if chosen["inliers"] <= 2:
        res_limit = min(res_limit, float(cfg.get("max_residual_2", 0.35)))
    if chosen["inliers"] < int(cfg["min_inliers"]):
        result["status"] = "insufficient_markers"
        result["reason"] = f"only {chosen['inliers']} inlier(s)"
    elif tie is not None and not prior_resolved:
        result["status"] = "ambiguous"
        result["reason"] = (f"{len(groups_out)} equally good hypotheses "
                            f"(mirror/symmetry) - press c to lock one")
    elif chosen["residual"] > res_limit:
        result["status"] = "high_residual"
        result["reason"] = (f"residual {chosen['residual']:.2f} m > {res_limit:.2f} m")
    else:
        result["valid"] = True
        result["reason"] = "best-effort (no map pair within tol)" if degraded else ""
    if degraded:
        result["reason"] = (result.get("reason") or
                            "best-effort (no map pair within tol)")
    return result


# ============================================================================
# 4. 自测：不接相机、不用模型，直接验证"像素->地面->匹配"几何是否正确
# ============================================================================
def _project_body_point(px, py, theta, cx, cy, height, pitch_deg, K, dist,
                        width=640, height_img=480):
    """
    正向投影（仅自测用）：地图点 -> 相机像素。
    (cx, cy, theta) 为相机位姿，返回 (u, v)；点不可见（在相机后方或画面外）返回 None。
    """
    import cv2

    c, s = math.cos(math.radians(theta)), math.sin(math.radians(theta))
    # 地图 -> 载体（地面系）
    dx = px - cx
    dy = py - cy
    xb = c * dx + s * dy
    yb = -s * dx + c * dy
    # 载体(地面系) -> 相机坐标（与 main_app 的 R_bc 互为转置）
    th = math.radians(pitch_deg)
    R_bc = np.array([
        [0.0, -math.sin(th), math.cos(th)],
        [-1.0, 0.0, 0.0],
        [0.0, -math.cos(th), -math.sin(th)],
    ])
    d_b = np.array([xb, yb, -height], dtype=float)
    P_cam = R_bc.T @ d_b
    if P_cam[2] <= 1e-6:
        return None
    # 可见性判据必须用"理想针孔像素"判断：
    # 本相机 k3=-20 的畸变模型在归一化半径较大时会把画面外的点折叠回画面内，
    # 直接用 projectPoints 的结果会误判成"可见"。
    xn, yn = P_cam[0] / P_cam[2], P_cam[1] / P_cam[2]
    u_ideal = K[0, 0] * xn + K[0, 2]
    v_ideal = K[1, 1] * yn + K[1, 2]
    if not (0.0 <= u_ideal < width and 0.0 <= v_ideal < height_img):
        return None
    obj = P_cam.reshape(1, 1, 3).astype(np.float64)
    uv, _ = cv2.projectPoints(obj, np.zeros((3, 1)), np.zeros((3, 1)), K, dist)
    return float(uv[0, 0, 0]), float(uv[0, 0, 1])


def _pixel_to_ground_inline(u, v, K, dist, height, pitch_deg):
    """与 main_app.GroundProjector 完全相同的公式（自测用）。"""
    import cv2

    pt = np.array([[[u, v]]], dtype=np.float64)
    xn, yn = cv2.undistortPoints(pt, K, dist)[0][0]
    th = math.radians(pitch_deg)
    denom = math.sin(th) + math.cos(th) * yn
    if denom <= 1e-3:
        return None
    lam = height / denom
    xb = lam * (math.cos(th) - math.sin(th) * yn)
    yb = lam * (-xn)
    return xb, yb


def _demo():
    """
    不接相机、不用 YOLO 的自测：
      地图点 -> 正向投影成像素 -> 像素反算地面 -> 多特征匹配 -> 位姿
    用来验证"内参/外参/投影/匹配"这条几何链路本身没有符号或公式错误。
    """
    import sys

    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, here)
    markers, source = load_field()
    print(f"地图来源：{source}，标志数：{len(markers)}")
    for cid in sorted({m["cls"] for m in markers}):
        print(f"  {CLASS_NAMES[cid]:<13} {sum(1 for m in markers if m['cls'] == cid)} 个")

    # 兼容开发目录（同层）与提交目录（src/ + ../calibration/）
    intr_path = None
    for cand in (os.path.join(here, "camera_intrinsics.yaml"),
                 os.path.join(here, "config", "camera_intrinsics.yaml"),
                 os.path.join(here, os.pardir, "calibration", "camera_intrinsics.yaml"),
                 os.path.join(os.getcwd(), "camera_intrinsics.yaml")):
        if os.path.isfile(cand):
            intr_path = cand
            break
    intr = load_intrinsics(intr_path) if intr_path else None
    if intr is None:
        print("没读到 camera_intrinsics.yaml，跳过几何自测。")
        return
    print(f"内参来源：{os.path.abspath(intr_path)}")
    K, dist = intr["K"], intr["dist"]
    height, pitch = 0.82, 8.0
    rng = np.random.default_rng(0)

    def observe(cx, cy, yaw, noise=0.0):
        obs = []
        for m in markers:
            uv = _project_body_point(m["x"], m["y"], yaw, cx, cy, height, pitch, K, dist)
            if uv is None:
                continue
            g = _pixel_to_ground_inline(uv[0], uv[1], K, dist, height, pitch)
            if g is None:
                continue
            xb, yb = g
            if noise:
                xb += rng.normal(0.0, noise)
                yb += rng.normal(0.0, noise)
            obs.append({"cls": m["cls"], "x": xb, "y": yb, "conf": 1.0})
        return obs

    # 找一个本相机 FOV 下能同时看到 3 个以上标志的位姿
    good = []
    for cx in np.arange(-3.0, 3.01, 0.5):
        for cy in np.arange(-2.5, 2.51, 0.5):
            for yaw in range(0, 360, 30):
                obs = observe(float(cx), float(cy), float(yaw))
                if len(obs) >= 3:
                    good.append((len(obs), float(cx), float(cy), float(yaw), obs))
    good.sort(key=lambda t: -t[0])
    print(f"\n视场自检：{len(good)} 个位姿可同时看到 >=3 个标志"
          f"（本相机水平视场约 {2 * math.degrees(math.atan(320 / K[0, 0])):.1f}°）")
    if not good:
        print("没有可用位姿，跳过。")
        return

    print("\n--- 几何自测：加入 3 cm 地面噪声后的位姿误差 ---")
    for n, cx, cy, yaw, _ in good[:5]:
        obs = observe(cx, cy, yaw, noise=0.03)
        res = estimate_pose(obs, markers, last_pose=(cx, cy, yaw))
        ex = math.hypot(res["x"] - cx, res["y"] - cy)
        ey = abs(_wrap_deg(res["yaw_deg"] - yaw))
        print(f"  真值({cx:5.2f},{cy:5.2f},{yaw:6.1f}) 观测{n} -> "
              f"估计({res['x']:5.2f},{res['y']:5.2f},{res['yaw_deg']:6.1f}) "
              f"内点{res['marker_count']} 残差{res['residual']:.4f}m "
              f"误差{ex:.3f}m/{ey:.2f}° {res['status']} valid={res['valid']}")

    print("\n--- 无先验时的多解判定（能唯一确定输出 ok；有并列解输出 ambiguous）---")
    n, cx, cy, yaw, obs = good[0]
    res = estimate_pose(obs, markers)
    print(f"  真值({cx:.2f},{cy:.2f},{yaw:.1f}) -> valid={res['valid']} "
          f"status={res['status']} 候选数={len(res['groups'])} "
          f"候选位置={[(round(g['x'], 2), round(g['y'], 2)) for g in res['groups'][:4]]}")

    print("\n--- 带 5 cm 地面噪声 + 上一帧先验（模拟连续帧跟踪）---")
    obs = observe(cx, cy, yaw, noise=0.05)
    res = estimate_pose(obs, markers, last_pose=(cx + 0.1, cy - 0.1, yaw + 3.0))
    ex = math.hypot(res["x"] - cx, res["y"] - cy)
    ey = abs(_wrap_deg(res["yaw_deg"] - yaw))
    print(f"  估计({res['x']:5.2f},{res['y']:5.2f},{res['yaw_deg']:6.1f}) "
          f"误差{ex:.3f}m/{ey:.2f}° 内点{res['marker_count']} "
          f"残差{res['residual']:.4f}m {res['status']} valid={res['valid']}")


if __name__ == "__main__":
    _demo()
