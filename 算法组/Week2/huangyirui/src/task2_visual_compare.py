# ======== 任务二：可视化与参数对比（30 分）========
#
# 留 言：
#   1. 本文件一行代码都没有，从当前这行之后开始写就行。
#   2. 运行：选中这个文件按 F5，或终端 python src/task2_visual_compare.py
#   3. 输入 "data/images/"，对比图、csv、md 都存 "out/task2/"。
#   4. 可以直接 import 任务一写好的检测函数，别复制两套。
#
# 建议覆盖：至少 2 组参数（阈值宽/紧、核大小、面积阈值）、
#           每组出「掩膜 + 标注」、横向拼成大图、结果写成表格、
#           并且能讲清「改了什么 → 现象为什么变」。
#
# —— 每改一次参数存一张图，README 的参数表才有东西写。(￣ω￣)
# ======================================================
import csv
import os
import sys

import cv2
import numpy as np

from task1_color_detect import detect_color,draw_result#借用task1的函数

Images = [
    "data/ball/sample_4.png",
    "data/ball/sample_1.png",
    "data/ball/sample_3.png",
    "data/ball/sample_5.png",
]
Out_Dir = "out/task2"
SHOW_WINDOW = "--no-window" not in sys.argv

Variants = [
    # 基准组：任务一的那一套，其它五组当参照物。
    {"name": "base", "lower": (36, 85, 60), "upper": (60, 255, 255),
     "kernel": 5, "min_area": 300},

    #H 往黄、S 往灰、V 往暗：
    {"name": "loosen", "lower": (30, 60, 40), "upper": (70, 255, 255),
     "kernel": 5, "min_area": 300},

    #H收40~52、V下限110：
    {"name": "tighten", "lower": (40, 100, 110), "upper": (52, 255, 255),
     "kernel": 5, "min_area": 300},

    #核1×1。
    {"name": "kernel1", "lower": (36, 85, 60), "upper": (60, 255, 255),
     "kernel": 1, "min_area": 300},

    #核9×9
    {"name": "kernel9", "lower": (36, 85, 60), "upper": (60, 255, 255),
     "kernel": 9, "min_area": 300},

    #面积阈值从300到2000
    {"name": "area2000", "lower": (36, 85, 60), "upper": (60, 255, 255),
     "kernel": 5, "min_area": 2000},
]

def label_box(img,text):#说明
    pad,scale,thick=6,0.55,2#pad留白 scale字号 thick粗细

    (text_w,text_h),_ = cv2.getTextSize(
        text,
        cv2.FONT_HERSHEY_SIMPLEX,#字体
        scale,
        thick,
    )

    cv2.rectangle(img,(0,0),(text_w + pad*2,text_h + pad*2),(255,255,255),-1)#加一个白色底框

    cv2.putText(img,text,(pad,text_h + pad),cv2.FONT_HERSHEY_SIMPLEX,scale,(0,0,0),thick)#写黑字

    return img


def variant_text(v):#把参数压成短文字贴在做上边
    return (f"{v['name']}"
            f"L{v['lower']}"
            f"U{v['upper']}"
            f"k{v['kernel']}"
            f"a{v['min_area']}")


#输出成品图
def build_compare(images):
    os.makedirs(Out_Dir,exist_ok=True)#创建目录，已存在则跳过
    csv_Path = os.path.join(Out_Dir,"report.csv")#读路径

    #利用withopen打开建立的csV，用newline让每行先归零，编码utf8，模式为write
    with open(csv_Path,"w",newline="",encoding="utf-8") as fcsv:
        w = csv.writer(fcsv)#向文件写数据
        w.writerow(["image","variant","detected","u","v","area"])#写表头


        for path in images:
            img = cv2.imread(path)
            if img is None:
                print("Fail to read:",path)
                continue
            name = os.path.splitext(os.path.basename(path))[0]#去掉路径、后缀，用文件名命名变量

            rows = []#建立一个空托盘

            for v in Variants:
                result,mask = detect_color(
                    img, v["lower"], v["upper"], v["min_area"], v["kernel"]
                )#和task1一样直接调用然后检测，输出

                vis,mask_BGR = draw_result(img,result,mask)#画框和中心点
                cell = np.hstack([vis,mask_BGR])#入栈拼接
                rows.append(label_box(cell.copy(),variant_text(v)))#复制后填白、写字，label过后存进rows

                r = result or {}
                w.writerow([
                        name,
                        v["name"],
                        r.get("detected", False),
                        r.get("cnt3", ""),
                        r.get("cnt4", ""),
                        r.get("area", 0),
                    ])#填记录表,没检测到就返回空值

                print(f"{name:16s}{v['name']:10s}"#宽度对齐
                    f"detected={bool(result)}"#返回一下检查结果，看看有没有内容在result里
                    f"uv={(r.get('cnt3'),r.get('cnt4'))}"
                    f"area={r.get('area',0)}")
                #图上字和表格的数互证


            sheet = np.vstack(rows)#压栈，合成大图
            sheet = cv2.resize(sheet,(sheet.shape[1]//2,sheet.shape[0]//2))#把原图缩小一点，让图片放得下

            Out_Path = os.path.join(Out_Dir,f"compare_{name}.png")#拼接输出文件夹
            cv2.imwrite(Out_Path,sheet)
            print("Saved:",Out_Path)


            if SHOW_WINDOW:
                cv2.imshow(f"compare_{name}",sheet)
                cv2.waitKey(0)
                cv2.destroyAllWindows()

#生成markdown表格
def write_markdown_table():
    csv_path = os.path.join(Out_Dir,"report.csv")

    with open(csv_path,newline="",encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    lines = [
        "| Pictures | Nums | Ifread | Center(u,v) | Area | Other |",
        "| --- | --- | --- | --- | --- | --- |"
    ]

    note = {
        "base": "标准参数，即原来的处理",
        "loosen": "S/V下线放款：多暗的、淡的目标，更容易误判背景",
        "tighten": "S/V下限收紧：只留最纯的，暗点的可能漏",
        "kernel1": "不滤波处理：噪点多，有的面积偏大",
        "kernel9": "核9×9：噪点干净但被误消除了，特别是足球的边缘一圈可能被认为是噪点，然后就消除了，最后识别出来面积偏小",
        "area2000": "面积下限提到2000：小目标直接不识别",
    }

    for r in rows:#遍历rows
        if str(r["detected"]).lower() == "true":#确认detected列是否为true
            uv = f"({r['u']},{r['v']})"#输出u、v
        else:
            uv = "-"#占位

        lines.append(#往markdown加字符串
                f"| {r['image']} | {r['variant']} | {r['detected']} | {uv} "
                f"| {r['area']} | {note.get(r['variant'], '')} |"
        )#按列加，每列用r输入

    Out_Path= os.path.join(Out_Dir, "report.md")

    with open(Out_Path,"w",encoding="utf-8") as f:#依旧with open
        f.write("The compared pictures\n\n" +  "\n".join(lines) + "\n")#打印lines表格行
    print("Saved:",Out_Path)


#main函数
if __name__ == "__main__":
    build_compare(Images)    
    write_markdown_table() 