import sys, time, os, cv2
from pathlib import Path

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
sys.stderr.reconfigure(encoding="utf-8", errors="replace")

CAM, SIZE, BOARD = 1, (1280, 720), (9, 9)
NEED_CALIB = 15
KEEP_FPS = 2.0
MIN_FPS = 20.0
BACKENDS = [("MSMF", cv2.CAP_MSMF), ("DSHOW", cv2.CAP_DSHOW), ("ANY", cv2.CAP_ANY)]
WIN = "capture  [c] record  [s] calib shot  [q] quit"
REPO = Path(__file__).resolve().parent.parent
DATA = Path(os.environ.get("W34_DATA", REPO.parents[4] / "week34_data"))
RAW = DATA / "raw_videos"
CALIB = DATA / "calib"
RAW.mkdir(parents=True, exist_ok=True)
CALIB.mkdir(parents=True, exist_ok=True)


def label(img, text, y, color, scale=0.8):
    cv2.putText(img, text, (20, y), 0, scale, (0, 0, 0), 6, cv2.LINE_AA)
    cv2.putText(img, text, (20, y), 0, scale, color, 2, cv2.LINE_AA)


def fourcc_of(cap):
    v = int(cap.get(cv2.CAP_PROP_FOURCC))
    s = "".join(chr((v >> (8 * i)) & 0xFF) for i in range(4))
    s = "".join(ch for ch in s if 32 <= ord(ch) < 127).strip()
    return s if s else "UNKNOWN"


def tune(cap):
    MJPG = cv2.VideoWriter_fourcc(*"MJPG")
    res = {}
    res["fourcc_1"] = cap.set(cv2.CAP_PROP_FOURCC, MJPG)
    res["width"] = cap.set(cv2.CAP_PROP_FRAME_WIDTH, SIZE[0])
    res["height"] = cap.set(cv2.CAP_PROP_FRAME_HEIGHT, SIZE[1])
    res["fps30"] = cap.set(cv2.CAP_PROP_FPS, 30)
    res["fourcc_2"] = cap.set(cv2.CAP_PROP_FOURCC, MJPG)
    res["width_2"] = cap.set(cv2.CAP_PROP_FRAME_WIDTH, SIZE[0])
    res["height_2"] = cap.set(cv2.CAP_PROP_FRAME_HEIGHT, SIZE[1])
    res["fps_2"] = cap.set(cv2.CAP_PROP_FPS, 30)
    res["autofocus_off"] = cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    res["zoom_lock"] = cap.set(cv2.CAP_PROP_ZOOM, 100)
    res["focus_fixed"] = cap.set(cv2.CAP_PROP_FOCUS, 1024)
    res["autowb_off"] = cap.set(cv2.CAP_PROP_AUTO_WB, 0)
    return res


def open_tuned(api):
    c = cv2.VideoCapture(CAM, api)
    if c.isOpened():
        tune(c)
    return c


def measure(cap, frames=40):
    ok, first = cap.read()
    if not ok:
        return None, None, 0.0
    n, t0 = 0, time.time()
    while n < frames:
        ok, _ = cap.read()
        if not ok:
            break
        n += 1
    return first.shape, fourcc_of(cap), n / max(time.time() - t0, 1e-6)


print(f"=== 探测相机索引 {CAM} 的三个后端 ===")
trials = []
for name, api in BACKENDS:
    trial = open_tuned(api)
    if not trial.isOpened():
        print(f"[{name}] 打不开")
        trial.release()
        continue
    shape, fc, fps = measure(trial)
    print(f"[{name}] FOURCC={fc} 分辨率={shape} 实测={fps:.1f} fps")
    trials.append((name, api, fc, fps, shape))
    trial.release()
time.sleep(1.0)

WANT_HW = (SIZE[1], SIZE[0])
right_size = [t for t in trials if t[4] and t[4][:2] == WANT_HW]
usable = [t for t in right_size if t[3] >= MIN_FPS]
pool = usable or right_size or trials
if not pool:
    raise SystemExit("三个后端都取不到帧，检查 USB / 设备管理器里相机是否被占用")
chosen = max(pool, key=lambda t: t[3])
name, api, fourcc_now, real_fps, _ = chosen
print(f"=== 选定 {name}（{real_fps:.1f} fps, FOURCC={fourcc_now}）并重新打开 ===")

cap = open_tuned(api)
if not cap.isOpened():
    raise SystemExit("选定后端后重新打开失败")
ok, check = cap.read()
actual_hw = check.shape[:2] if ok else None
fourcc_now = fourcc_of(cap)
if actual_hw != WANT_HW:
    cap.release()
    raise SystemExit(f"实际帧尺寸 {actual_hw}，要求 {WANT_HW}。"
                     f"不要拍标定图，先把分辨率调对。各后端: {[(t[0], t[4]) for t in trials]}")
print(f"分辨率校验通过: {actual_hw[1]}x{actual_hw[0]}")

if fourcc_now == "YUY2":
    print("!! 明确是 YUY2，帧率受 USB2 带宽限制")
elif fourcc_now == "UNKNOWN" and real_fps >= 25:
    print("MSMF 读不回 FOURCC，但 25+ fps 只可能是 MJPG，视为已吃到 MJPG")

focus_ok = cap.get(cv2.CAP_PROP_FOCUS)
print(f"焦距控制: 请求值 1024 -> 读回 {focus_ok}；自动对焦开关不被后端支持，需厂商软件或物理锁焦")

existing = [int(p.stem[4:]) for p in RAW.glob("clip*.avi") if p.stem[4:].isdigit()]
clip = max(existing) if existing else 0
print(f"已有片段最大编号 clip{clip:02d}，本次从 clip{clip+1:02d} 开始（不会覆盖旧文件）")
writer, n, t0, log = None, 0, 0.0, []
calib_ok, calib_bad = 0, 0
last, shown_fps = time.time(), 0.0
started = time.strftime("%Y-%m-%d %H:%M:%S")
retries, fails = 0, 0

while True:
    ok, f = cap.read()
    if not ok:
        fails += 1
        retries += 1
        print(f"取帧失败（第 {retries} 次），重连后端 {name} ...")
        cap.release()
        time.sleep(1.0)
        cap = open_tuned(api)
        if not cap.isOpened() or retries > 4:
            print("重连失败，结束采集")
            break
        last = time.time()
        continue
    if fails:
        print(f"重连成功，累计失败 {fails} 次")

    raw = f.copy()
    now = time.time()
    shown_fps += 0.1 * (1.0 / max(now - last, 1e-6) - shown_fps)
    last = now
    if writer:
        writer.write(raw)
        n += 1

    label(f, f"FPS {shown_fps:.1f}", 45, (0, 255, 0), 1.0)
    label(f, f"calib OK={calib_ok}/{NEED_CALIB}  BAD={calib_bad}", 90, (255, 255, 255), 0.8)
    label(f, f"{fourcc_now} {actual_hw[1]}x{actual_hw[0]} {name}", 130, (255, 255, 255), 0.7)
    if writer:
        label(f, f"REC clip{clip:02d}  {n}f  {now - t0:.1f}s", 175, (0, 0, 255), 1.0)
    cv2.imshow(WIN, f)

    k = cv2.waitKey(1) & 0xFF
    if k == ord("c"):
        if writer:
            dur = time.time() - t0
            writer.release()
            writer = None
            line = f"clip{clip:02d} 帧数={n} 时长={dur:.1f}s 写入帧率={real_fps:.1f}"
            print(line)
            log.append(line)
            n = 0
        else:
            clip += 1
            t0 = time.time()
            path = str(RAW / f"clip{clip:02d}.avi")
            writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"MJPG"), real_fps, SIZE)
            if not writer.isOpened():
                writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"XVID"), real_fps, SIZE)
            print(f"开始录制 clip{clip:02d}，再按 c 结束")
    elif k == ord("s"):
        gray = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCornersSB(gray, BOARD, cv2.CALIB_CB_ADAPTIVE_THRESH)
        if found:
            cv2.drawChessboardCorners(f, BOARD, corners, found)
        tag = "OK" if found else "BAD"
        if found:
            calib_ok += 1
        else:
            calib_bad += 1
        p = CALIB / f"calib_{clip:02d}_{time.strftime('%H%M%S')}_{tag}.png"
        cv2.imwrite(str(p), raw)
        print(f"标定图 {p.name}  进度 {calib_ok}/{NEED_CALIB}")
    elif k == ord("q"):
        break

if writer:
    writer.release()
prop_fps = cap.get(cv2.CAP_PROP_FPS)
cap.release()
cv2.destroyAllWindows()

head = [
    f"采集时间: {started} ~ {time.strftime('%Y-%m-%d %H:%M:%S')}",
    f"相机索引: {CAM}  选定后端: {name}  取帧重连次数: {retries}",
    f"分辨率: {actual_hw[1]}x{actual_hw[0]}  生效 FOURCC: {fourcc_now}  实测采集帧率: {real_fps:.1f} fps  CAP_PROP_FPS: {prop_fps:.1f}",
    f"探测结果: {trials}",
    f"标定图: OK={calib_ok}  BAD={calib_bad}  棋盘格内角点: {BOARD}",
    f"抽帧目标间隔: {KEEP_FPS} fps",
    "",
]
(RAW / "capture_log.md").write_text("\n".join(head) + "\n".join(log) + "\n", encoding="utf-8")
print("日志写入", RAW / "capture_log.md")
