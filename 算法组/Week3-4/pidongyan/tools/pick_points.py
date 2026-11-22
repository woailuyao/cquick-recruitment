"""任务二「人工点选真实交点」交互工具（替代不可信的理论投影参考图）。

为什么需要它
------------
`tools/gen_pose_guides.py` 用 CSV 真值位姿正向投影出来的红圈与实拍白线交点
对不上（真值本身是拿有争议的内参反解的，误差二次放大）。**投影点不可用作观测。**

本工具只做一件事：打开实拍帧，让你用鼠标点**真实白线交点**，记录原图像素，
输出 `run_task2.py clicks` 能直接吃的 yaml。所有观测都来自你本人目视判断，
不掺任何理论值——这也是考核要求的「图像特征提取」真实证据。

【点不在照片上怎么办】—— 不用管，直接跳过
------------------------------------------
field.yaml 有 15 个点，但一张窄视场实拍帧通常只拍得到其中几个。
**PnP 只需要 >=4 个可见点**，yaml 里只写你实际点到的点，没点的点根本不参与解算
（`run_task2.py clicks` 里 `items = [(n, field[n]) for n in clicks]`）。
所以：看不见 / 被裁掉 / 被挡住 / 认不准编号的点，按 `s` 标记为「本帧不可见」跳过，
绝不要凭猜测补一个坐标，也绝不要拿理论投影位置凑数。

红线
----
- 必须点画面里**实际看得见的白线交点**，不要点任何理论/推测位置。
- 至少 4 个点，且尽量分散、不要近似共线（共线会让 PnP 病态）。
- 点名必须对应 `field.yaml` 里真实存在的那个交点，**认不准编号就别点它**。
  例如 LCross5 是场地角 [4.175,-3.0]，LCross1 是禁区角 [2.295,-1.94]，认错会直接解出错位姿。
- **不要跨帧凑点**：所有点必须来自同一张图（同一相机位姿）。可见点不足 4 个就整张换帧重标。
- 生成的 yaml 只是中间文件；误差表由 `run_task2.py clicks` 解算并对比真值产生。

操作
----
  左键      给「当前选中」的点名落点，并自动跳到下一个可用点名
  n / p     切换当前待点的点名（侧栏 `>` 高亮）
  s         【本帧看不见这个点】跳过它，不写入 yaml，自动前进
  u         撤销上一个点
  c         清空全部（含跳过标记）
  q         保存并退出（打印可直接运行的命令）

用法
----
  python tools/pick_points.py --image frames_pool_markers/pose_B_20261112_173941_f000000.jpg --tag pose_B_20261112_173941
  # 想只盯住某几个点名，可加 --points LCross1,TCross3,XCross2,PenalityPoint
  # 之后按保存时打印的提示跑（诊断 → 候选副本 → 解算，均为 PowerShell 单行，别折行）：
  python tools/diagnose_clicks.py --clicks check_out/clicks_pose_B_20261112_173941.yaml --tag pose_B_20261112_173941
  python tools/run_task2.py clicks --clicks check_out/clicks_pose_B_20261112_173941_candidate.yaml --tag pose_B_20261112_173941 --image <同一张图>
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from task2.geometry import load_field  # noqa: E402


class Picker:
    def __init__(self, img_path, names, out_path, tag, disp_w, disp_h):
        self.img = cv2.imread(str(img_path))
        if self.img is None:
            raise SystemExit(f"读不了图片：{img_path}")
        self.H, self.W = self.img.shape[:2]
        self.names = names
        self.out_path = out_path
        self.tag = tag
        self.picked = {}            # name -> (u, v) 原图坐标
        self.skipped = set()        # 本帧看不见、主动跳过的点名
        self.cur = 0
        self.scale = min(disp_w / self.W, disp_h / self.H, 1.0)
        self.dw, self.dh = int(self.W * self.scale), int(self.H * self.scale)

    def _next_available(self, start=None):
        """从 start 起找下一个「未点且未跳过」的索引；全占满就返回 start。"""
        base = self.cur if start is None else start
        for i in range(len(self.names)):
            idx = (base + i) % len(self.names)
            n = self.names[idx]
            if n not in self.picked and n not in self.skipped:
                return idx
        return base

    def _current_name(self):
        """当前选中索引。尊重 n/p 手动切换，不再自动覆盖成「第一个未点」。"""
        return self.cur

    def _on_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        u, v = x / self.scale, y / self.scale
        idx = self.cur
        name = self.names[idx]
        self.picked[name] = (round(u, 1), round(v, 1))
        self.skipped.discard(name)
        self.cur = self._next_available((idx + 1) % len(self.names))

    def render(self):
        vis = cv2.resize(self.img.copy(), (self.dw, self.dh))
        # 已点的点
        for name, (u, v) in self.picked.items():
            p = (int(u * self.scale), int(v * self.scale))
            cv2.circle(vis, p, 7, (0, 200, 0), 2)
            cv2.circle(vis, p, 2, (0, 255, 0), -1)
            cv2.putText(vis, name, (p[0] + 10, p[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 0), 1, cv2.LINE_AA)
        # 侧栏：点名列表
        panel_w = 230
        canvas = np.full((max(self.dh, 40 + 22 * len(self.names)),
                          self.dw + panel_w, 3), 32, dtype=np.uint8)
        canvas[:self.dh, :self.dw] = vis
        cur_idx = self._current_name()
        cv2.putText(canvas, f"tag: {self.tag}", (self.dw + 8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(canvas,
                    f"picked {len(self.picked)} / need >=4   skip {len(self.skipped)}",
                    (self.dw + 8, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.40,
                    (0, 200, 255), 1, cv2.LINE_AA)
        for i, n in enumerate(self.names):
            y = 62 + 22 * i
            done = n in self.picked
            skip = n in self.skipped
            color = ((0, 200, 0) if done else
                     (90, 90, 90) if skip else
                     (0, 0, 255) if i == cur_idx else (150, 150, 150))
            mark = "+" if done else ("-" if skip else (">" if i == cur_idx else " "))
            cv2.putText(canvas, f"{mark} {n}", (self.dw + 8, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.44, color, 1, cv2.LINE_AA)
        cv2.putText(canvas,
                    "LMB=pick n/p=switch s=skip(not visible) u=undo c=clear q=save",
                    (8, self.dh - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    (0, 200, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, "CLICK REAL WHITE-LINE INTERSECTIONS ONLY (no theory points)",
                    (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0, 0, 255), 1, cv2.LINE_AA)
        return canvas

    def save(self):
        self.out_path.parent.mkdir(parents=True, exist_ok=True)
        data = {n: [u, v] for n, (u, v) in self.picked.items()}
        skipped = [n for n in self.names if n in self.skipped]
        header = (
            "# 人工在实拍帧上点选的真实白线交点（原图像素）。\n"
            "# 非理论投影值；由 tools/pick_points.py 生成。\n"
            f"# 本帧不可见、已跳过：{', '.join(skipped) if skipped else '（无）'}\n"
            "# 只写入实际看见并点选的点；没点的点不参与 PnP，无需凑齐 15 个。\n"
        )
        self.out_path.write_text(
            header + yaml.safe_dump(data, allow_unicode=True, sort_keys=False),
            encoding="utf-8")
        return len(data)

    def run(self):
        win = "pick_points"
        cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(win, self._on_mouse)
        print(f"图片 {self.W}x{self.H}  显示 {self.dw}x{self.dh}  "
              f"候选点名 {len(self.names)} 个")
        print("点真实白线交点；至少 4 个且分散。")
        print("看不见 / 认不准的点按 s 跳过（不写入文件）；n/p 选点名，左键落点，q 保存退出。\n")
        while True:
            cv2.imshow(win, self.render())
            key = cv2.waitKey(30) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("n"):
                self.cur = (self.cur + 1) % len(self.names)
            elif key == ord("p"):
                self.cur = (self.cur - 1) % len(self.names)
            elif key == ord("s"):
                name = self.names[self.cur]
                self.picked.pop(name, None)
                self.skipped.add(name)
                self.cur = self._next_available((self.cur + 1) % len(self.names))
            elif key == ord("u"):
                if self.picked:
                    name, _ = self.picked.popitem()
                    self.cur = self.names.index(name)
            elif key == ord("c"):
                self.picked.clear()
                self.skipped.clear()
                self.cur = 0
        cv2.destroyAllWindows()
        n = self.save()
        skipped = [x for x in self.names if x in self.skipped]
        print(f"\n已保存 {n} 个点 -> {self.out_path}")
        print(f"本帧跳过（不可见）{len(skipped)} 个：{', '.join(skipped) if skipped else '无'}")
        orig = self.out_path.relative_to(ROOT).as_posix()
        if n < 4:
            print(f"\n[无效状态] 仅 {n} 个可见交点：平面 PnP 至少需 4 个不共线点。")
            print("这是细则要求的「标志不足 → 无效状态」案例，不要跨帧凑点、不要猜坐标、")
            print("不要拿理论投影凑数。直接用原点选文件打印规范状态：")
            print(f"  python tools/run_task2.py clicks --clicks {orig} "
                  f"--tag {self.tag} --image <同一张图>")
            return
        cand = self.out_path.with_name(
            self.out_path.stem + "_candidate.yaml").relative_to(ROOT).as_posix()
        print("\n下一步（两条，整行复制粘贴）：")
        print(f"  python tools/diagnose_clicks.py --clicks {orig} --tag {self.tag}")
        print(f"  python tools/run_task2.py clicks --clicks {cand} "
              f"--tag {self.tag} --image <同一张图>")
        print("  （diagnose 无论编号是否贴反都会写出候选副本；若报告「编号贴反」，")
        print("   先对照 docs/field_measured_map.png 目视确认，再跑第二条）")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--image", required=True, help="实拍帧路径")
    ap.add_argument("--tag", required=True, help="位姿真值 CSV 里的视频名（不带扩展名）")
    ap.add_argument("--field", default="config/field.yaml")
    ap.add_argument("--points", default=None,
                    help="只列出这些点名（逗号分隔）；默认 field.yaml 全部 15 个")
    ap.add_argument("--out", default=None, help="输出 yaml，默认 check_out/clicks_<tag>.yaml")
    ap.add_argument("--disp-w", type=int, default=1180)
    ap.add_argument("--disp-h", type=int, default=660)
    args = ap.parse_args()

    field = load_field(ROOT / args.field)
    names = ([s.strip() for s in args.points.split(",") if s.strip()]
             if args.points else list(field))
    unknown = [n for n in names if n not in field]
    if unknown:
        raise SystemExit(f"field.yaml 里没有：{unknown}")
    out = (ROOT / args.out) if args.out else (ROOT / "check_out" / f"clicks_{args.tag}.yaml")
    img = ROOT / args.image if not Path(args.image).is_absolute() else Path(args.image)

    Picker(img, names, out, args.tag, args.disp_w, args.disp_h).run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
