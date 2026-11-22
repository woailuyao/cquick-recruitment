# -*- coding: utf-8 -*-
"""场地标志（L / T 两类）标注专用 labelImg 启动器。

与原 launch_labelimg.py 的区别：
  1. 不强制单类 ball。原来画完框直接写 "ball"，一旦保存就会把 ball 追加进
     classes.txt 末尾（libs/yolo_io.py 的 bnd_box_to_yolo_line 里会 append），
     导致 L=0 / T=1 的类别号整体错位，训练数据全废。
     改为弹出「L / T」两个大按钮自绘小窗，规避 WSLg 下原生 LabelDialog
     崩溃的问题（原启动器正是为此屏蔽了弹窗）。
  2. 显式把保存格式设为 YOLO，确保 Ctrl+S 写的是 .txt 而不是 .xml。

用法：
  python launch_labelimg_landmark.py <图片目录> [classes.txt] [保存目录]

说明：labelImg 的 YOLO 读取路径是「保存目录/图名.txt」，
      所以图片与 txt 必须同目录（本次已整理为同一个 labelimg_work 目录）。
"""
import sys

from PyQt5.QtWidgets import (QDialog, QHBoxLayout, QLabel, QPushButton,
                             QVBoxLayout)

import labelImg.labelImg as L

CLASSES = ["L", "T"]


class _ClassDialog:
    """LabelDialog 替身：两个大按钮直接选 L / T。"""

    def __init__(self, *args, **kwargs):
        pass

    def pop_up(self, text=None):
        dlg = QDialog()
        dlg.setWindowTitle("选择标志类别")
        lay = QVBoxLayout(dlg)
        lay.addWidget(QLabel("这个框是哪种标志？"))
        row = QHBoxLayout()
        picked = {"v": None}

        def make(name):
            def _():
                picked["v"] = name
                dlg.accept()
            return _

        for n in CLASSES:
            btn = QPushButton(n)
            btn.setMinimumSize(96, 46)
            btn.setStyleSheet("font-size:20px; font-weight:600;")
            btn.clicked.connect(make(n))
            row.addWidget(btn)
        lay.addLayout(row)
        lay.addWidget(QLabel("关窗 / Esc = 放弃这个框"))
        dlg.exec_()
        return picked["v"]          # None → labelImg 视为取消，不建框


L.LabelDialog = _ClassDialog

argv = ["labelImg"] + sys.argv[1:]
data_dir = sys.argv[1] if len(sys.argv) > 1 else None
app, win = L.get_main_app(argv)

# labelImg 会从设置里恢复"上次打开的目录"，无视命令行传的路径
# （实测打开的是 soccer_dataset/images/val）。必须显式覆盖，
# 否则你会对着足球数据集标注。
if data_dir:
    win.default_save_dir = data_dir
    win.last_open_dir = data_dir
    try:
        win.open_dir_dialog(False, data_dir, True)
    except Exception as exc:                 # pragma: no cover
        print("打开目录失败: %s" % exc)

# 保存格式强制 YOLO（首次加载已有 txt 时 labelImg 自己也会切，这里显式兜底）
try:
    win.set_format(L.FORMAT_YOLO)
except Exception as exc:                      # pragma: no cover
    print("设置 YOLO 格式失败: %s" % exc)

# 关掉单类模式与默认标签，确保每次都走两按钮选择
for attr, value in (("single_class_mode", False),
                    ("use_default_label_checkbox", False)):
    try:
        getattr(win, attr).setChecked(value)
    except Exception:
        pass
try:
    win.default_label_text_line.setText("")
except Exception:
    pass

win.show()
sys.exit(app.exec_())
