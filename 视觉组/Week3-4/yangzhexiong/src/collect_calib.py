"""标定图采集：对着棋盘格按空格拍照，自动编号存到 calib/。

用法：
    python3 collect_calib.py
    python3 collect_calib.py --out calib --prefix calib

操作：
    空格  拍一张（自动存 calib/calib_01.jpg）
    u     删掉最后一张
    q     退出

拍照要求：
    · 棋盘占画面 1/3 ~ 2/3
    · 至少 15 张，覆盖：正中、左上、右上、左下、右下、倾斜、远近
    · 棋盘必须平整（贴在板上），手别抖
"""
import argparse
import glob
import os

import cv2

import camera


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="calib")
    ap.add_argument("--prefix", default="calib")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    cap, frame, info = camera.auto_open()
    h, w = frame.shape[:2]
    print(f"相机 {w}x{h}")
    print(f"按 空格 拍照，u 删除上一张，q 退出。保存到 {args.out}/")

    win = "collect calib  [space]=shot  u=undo  q=quit"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    saved = 0
    last = None

    while True:
        ok, frame = cap.read()
        if not ok:
            print("取帧失败")
            break
        canvas = frame
        cv2.putText(canvas, f"{w}x{h}   saved={saved}", (12, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(canvas, "SPACE=shot   u=undo   q=quit", (12, h - 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow(win, canvas)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord(" ") or key == 13:
            saved += 1
            path = os.path.join(args.out, f"{args.prefix}_{saved:02d}.jpg")
            cv2.imwrite(path, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            last = path
            print(f"  已保存 {path}  ({saved} 张)")
        if key == ord("u") and last and os.path.exists(last):
            os.remove(last)
            print(f"  已删除 {last}")
            saved -= 1
            last = None

    cap.release()
    cv2.destroyAllWindows()
    total = len(glob.glob(os.path.join(args.out, "*.jpg")))
    print(f"\n{args.out}/ 里共 {total} 张")
    if total < 15:
        print("少于 15 张，建议再拍一些")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
