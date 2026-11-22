"""把一组现场实测位姿真值追加到 logs/field_pose_samples.csv。

只追加，不改、不删任何历史行（红线：脚本不删文件）。

yaw 由「相机光心地面投影」与「光轴落地点」两点连线算出，全部来自卷尺实测，
与任何标志点像素无关，因此不会污染后续解算（不循环自证）。

用法（PowerShell 单行，末尾不要加反斜杠）：
  python tools/add_pose_sample.py --tag pose_D --camx -2.90 --camy -2.10 --aimx -0.10 --aimy 0.35 --h 0.55 --note "机位1 左场角朝对角"

参数含义：
  --tag    视频名前缀（不含时间戳），如 pose_D；工具自动在 raw_videos 里找最新的
           pose_D_*.mp4，并把完整视频名写进 CSV
  --camx   相机光心地面投影的 x（米，C0 系）
  --camy   相机光心地面投影的 y（米，C0 系）
  --aimx   光轴落地点（画面正中心对准的地面点）的 x
  --aimy   光轴落地点的 y
  --h      镜头光心离地高度（米），一般 0.55
  --note   备注；建议写明站位、朝向、视野里有哪些标志

坐标系（与 config/field.yaml 一致）：原点 = 场地中心，+X 指向唯一球门，
+Y 朝左（面朝球门时），地面 Z=0，单位米；yaw 逆时针为正，朝球门 = 0°。
"""
import argparse
import csv
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RAW_DIR = ROOT / "raw_videos"
DEFAULT_CSV = ROOT / "logs" / "field_pose_samples.csv"
HEADER = ["视频文件", "x", "y", "yaw_deg", "相机高度m", "备注"]


def norm_deg(a):
    """归一化到 (-180, 180]。"""
    a = math.fmod(a + 180.0, 360.0)
    if a <= 0:
        a += 360.0
    return a - 180.0


def resolve_video(tag, video_arg):
    if video_arg:
        p = Path(video_arg)
        if not p.is_absolute():
            p = ROOT / p
        if not p.exists():
            sys.exit(f"错误：找不到视频文件 {p}")
        return p
    hits = sorted(RAW_DIR.glob(f"{tag}_*.mp4"), key=lambda q: q.stat().st_mtime)
    if not hits:
        sys.exit(f"错误：raw_videos 里没有 {tag}_*.mp4\n"
                 f"  先录制：python tools/capture_video.py --index 0 --tag {tag} --duration 15")
    if len(hits) > 1:
        print(f"[提示] 找到 {len(hits)} 个 {tag}_*.mp4，取最新的：{hits[-1].name}")
    return hits[-1]


def short(p):
    """尽量显示相对根目录的路径，跨盘/临时路径则原样显示。"""
    try:
        return p.relative_to(ROOT)
    except ValueError:
        return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", required=True, help="视频名前缀，如 pose_D")
    ap.add_argument("--video", default="", help="直接指定视频路径（默认按 --tag 找最新）")
    ap.add_argument("--camx", type=float, required=True, help="相机光心投影 x（米）")
    ap.add_argument("--camy", type=float, required=True, help="相机光心投影 y（米）")
    ap.add_argument("--aimx", type=float, required=True, help="光轴落地点 x（米）")
    ap.add_argument("--aimy", type=float, required=True, help="光轴落地点 y（米）")
    ap.add_argument("--h", type=float, default=0.55, help="镜头光心离地高度（米）")
    ap.add_argument("--note", default="", help="备注：站位/朝向/视野含哪些标志")
    ap.add_argument("--csv", default=str(DEFAULT_CSV), help="真值 CSV 路径（测试用）")
    args = ap.parse_args()

    vid = resolve_video(args.tag, args.video)
    stem = vid.stem

    dx = args.aimx - args.camx
    dy = args.aimy - args.camy
    base = math.hypot(dx, dy)
    if base < 1.0:
        sys.exit(f"错误：光轴落地点离相机只有 {base:.2f} m，太近，yaw 角度会很不准。\n"
                 f"  请把正中心对准前方 2 m 以上的地面点重新量。")
    yaw = norm_deg(math.degrees(math.atan2(dy, dx)))

    if not (0.05 <= args.h <= 2.0):
        sys.exit(f"错误：相机高度 {args.h} m 不合理，检查是不是量成厘米了。")

    csv_path = Path(args.csv)
    if not csv_path.is_absolute():
        csv_path = ROOT / csv_path

    existing = set()
    if csv_path.exists():
        with csv_path.open("r", encoding="utf-8-sig", newline="") as f:
            for row in csv.DictReader(f):
                key = (row.get("视频文件") or "").strip()
                if key:
                    existing.add(Path(key).stem)
    if stem in existing:
        sys.exit(f"错误：{stem} 已经在 CSV 里了，不重复追加。\n"
                f"  要改数值请先手工编辑 {short(csv_path)}。")

    new_file = not csv_path.exists()
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("a", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        if new_file:
            w.writerow(HEADER)
        w.writerow([vid.name, f"{args.camx:.3f}", f"{args.camy:.3f}",
                    f"{yaw:.1f}", f"{args.h:.2f}", args.note])

    print("已追加 1 行真值（历史行未改动）")
    print(f"  视频      {vid.name}")
    print(f"  相机位置  x={args.camx:+.3f}  y={args.camy:+.3f}  h={args.h:.2f}")
    print(f"  光轴落点  x={args.aimx:+.3f}  y={args.aimy:+.3f}   距离 {base:.2f} m")
    print(f"  yaw       {yaw:+.1f}°  （由两点连线算出，未使用任何标志像素）")
    print(f"  CSV       {short(csv_path)}")
    print("")
    print("后续点选/解算直接用这个 tag（整行复制，不加反斜杠）：")
    print(f"  python tools/pick_points.py --image <选定帧路径> --tag {stem}")
    print(f"  python tools/diagnose_clicks.py --clicks check_out/clicks_{stem}.yaml --tag {stem}")
    print(f"  python tools/run_task2.py clicks --clicks check_out/clicks_{stem}_candidate.yaml "
          f"--tag {stem} --image <选定帧路径>")


if __name__ == "__main__":
    main()
