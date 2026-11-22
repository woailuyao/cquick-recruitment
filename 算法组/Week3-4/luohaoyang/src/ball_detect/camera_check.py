import cv2
import argparse


def check_camera(index: int):
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print(f"[camera {index}] 打不开")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)

    ok, frame = cap.read()
    if not ok:
        print(f"[camera {index}] 打开成功，但读不到画面")
        cap.release()
        return

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"[camera {index}] 分辨率={width}x{height}, 帧率={fps:.1f}")
    print("按任意键关闭窗口。")

    cv2.putText(
        frame,
        f"camera index={index}, {width}x{height}, {fps:.1f} fps",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        2,
    )
    cv2.imshow(f"camera {index}", frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    cap.release()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--index", type=int, default=None,
                        help="只检查一个摄像头序号；不填则检查 0、1")
    args = parser.parse_args()

    if args.index is None:
        for i in (0, 1):
            check_camera(i)
    else:
        check_camera(args.index)