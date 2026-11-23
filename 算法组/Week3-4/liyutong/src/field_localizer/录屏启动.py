# -*- coding: utf-8 -*-
"""
合规录屏启动器（任务 1 / 任务 2 通用）

作用：把细则第六节要求的「姓名 + 实时时间」打在终端最前面，然后**在同一个终端里**
直接拉起 demo.py，让屏录从第一秒到最后一秒都是同一段连续画面。

用法
    python 录屏启动.py                       # 摄像头 0，默认姓名 liyutong
    python 录屏启动.py --record demo_实时.mp4   # 顺便让程序也存一份合成画面
    python 录屏启动.py --name zhangsan        # 临时换名字
    python 录屏启动.py -- --marker-model none  # 双横线之后的参数原样传给 demo.py

为什么不用手敲 echo：
  Windows PowerShell 里 `date "+%Y-%m-%d"` 会报「无法绑定参数 Date」，
  细则给的是 bash 写法。这里直接由程序打印等价的两行，避免录屏时因为命令报错重来。
"""
import sys
import time

NAME = "liyutong"          # ← 名字只在这里改一处


def main():
    argv = sys.argv[1:]
    name = NAME
    passthrough = []

    i = 0
    while i < len(argv):
        a = argv[i]
        if a == "--name" and i + 1 < len(argv):
            name = argv[i + 1]
            i += 2
        elif a == "--":
            passthrough.extend(argv[i + 1:])
            break
        else:
            passthrough.append(a)
            i += 1

    # ---- 细则第六节要求出现在画面里的两行，等价于：
    #      echo "姓名: <姓名拼音>"; date "+%Y-%m-%d %H:%M:%S"
    print("=" * 60)
    print(f"姓名: {name}")
    print(time.strftime("%Y-%m-%d %H:%M:%S"))
    print("=" * 60, flush=True)

    # ---- 同一进程、同一终端，紧接着启动 demo ----
    # --splash 6：程序窗口先显示 6 秒「启动证明页」（姓名/时间/环境/模型路径）。
    # 实验室那台机器没有 OBS，Win+G 只能录单个窗口 —— 有这一页，只录 demo 窗口也能自证。
    sys.argv = (["demo.py", "--cam", "0", "--whoami", name, "--splash", "6"]
                + passthrough)
    import demo
    demo.main()


if __name__ == "__main__":
    main()
