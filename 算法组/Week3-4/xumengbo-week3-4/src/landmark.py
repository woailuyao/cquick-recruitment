"""
几何细化：检测框 -> 标志像素特征点 (u, v)
==========================================
细则 2.4：不能直接拿检测框中心当交点；每类标志在框内按标线走向取真实特征点。

取点规则：
  LCross : L 型两条白线的交会顶点（外角/内角顶点）
  TCross : T 型横线与竖线的交点
  XCross : 两条线交叉中心

当前版本（骨架）：先用检测框几何给出参考候选点；线段拟合细化接口
`refine_by_lines` 预留，后续用 Canny + Hough 拟合两条白线再求交点。
"""
import cv2
import numpy as np


def box_center(box):
    """box = [x1, y1, x2, y2]，返回框中心 (cx, cy)。"""
    x1, y1, x2, y2 = box
    return (x1 + x2) / 2.0, (y1 + y2) / 2.0


def extract_landmark(cls_name, box, frame=None):
    """按类别从检测框提取像素特征点。

    参数：
      cls_name : 类别名 LCross / TCross / XCross
      box      : [x1,y1,x2,y2] 像素框
      frame    : 原始 BGR 帧（线段拟合细化时使用，当前骨架可传 None）
    返回：
      {"u", "v", "rule", "refined"}
      refined=False 表示尚未做线段拟合，点为框几何候选，精度待细化。
    """
    x1, y1, x2, y2 = box
    cx, cy = box_center(box)

    if cls_name == "XCross":
        # X 型两条线交叉中心：交叉点落在框中心附近
        u, v, rule = cx, cy, "X 交叉中心（框中心候选）"
    elif cls_name == "LCross":
        # L 型顶点：两条边线交会的角点。完整实现需判断两条白线走向，
        # 再选框的对应角点；骨架先取框中心，refine_by_lines 出真实顶点。
        u, v, rule = cx, cy, "L 交会顶点（待线段拟合选角点）"
    elif cls_name == "TCross":
        # T 型交点：竖线与横线相交处。骨架先取框中心候选。
        u, v, rule = cx, cy, "T 横竖交点（待线段拟合）"
    else:
        # 未知类别：保守取框中心并标记不可靠
        u, v, rule = cx, cy, "未知类别（框中心候选，不可靠）"

    return {"u": float(u), "v": float(v), "rule": rule, "refined": False}


def refine_by_lines(frame, box):
    """预留：在框内拟合两条白线并求交点。

    后续实现步骤：
      1. 裁出 box 区域，转灰度 + 阈值二值化（白线高亮）；
      2. Canny 边缘 + HoughLinesP 拟合直线，按角度分成两组主方向；
      3. 两组直线各自取平均代表线，求两线交点；
      4. 交点落在框内才采用，否则回退框中心。
    当前先返回 None，由 extract_landmark 的候选点兜底。
    """
    return None


def landmark_list(results, names):
    """把 ultralytics 检测结果转成标志特征点列表。

    输入：results[0]（YOLO predict 结果），names={id: 类别名}
    输出：[{"type", "box", "u", "v", "conf", "rule", "refined"}, ...]
    """
    out = []
    r = results[0]
    for b in r.boxes:
        x1, y1, x2, y2 = b.xyxy[0].tolist()
        box = [int(x1), int(y1), int(x2), int(y2)]
        conf = float(b.conf[0])
        cls_id = int(b.cls[0])
        cls_name = names[cls_id]
        lm = extract_landmark(cls_name, box)
        out.append({
            "type": cls_name,
            "box": box,
            "u": lm["u"], "v": lm["v"],
            "conf": round(conf, 3),
            "rule": lm["rule"],
            "refined": lm["refined"],
        })
    return out
