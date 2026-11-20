import argparse
from pathlib import Path

import cv2
import numpy as np


PATTERN_SIZE = (9, 9)       # 内部角点：横向 9 个，纵向 9 个
SQUARE_SIZE_M = 0.027       # 每格实测 27 mm = 0.027 m
def main():
    parser = argparse.ArgumentParser(description="9x9 棋盘格相机内参标定")
    parser.add_argument("--images", type=Path, required=True, help="标定照片目录")
    parser.add_argument("--output", type=Path, help="输出目录，默认为照片目录下 calibration_output")
    parser.add_argument("--max-view-rms", type=float, default=1.5,
                        help="剔除单张 RMS 超过此值的照片")
    args = parser.parse_args()
    image_dir = args.images.resolve()
    output_dir = (args.output or image_dir / "calibration_output").resolve()
    image_paths = sorted(
        p for p in image_dir.iterdir()
        if p.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp"}
    )
    if not image_paths:
        raise SystemExit(f"未找到标定照片：{image_dir}")

    output_dir.mkdir(parents=True, exist_ok=True)

    # 标定板平面上的三维点，单位为米，Z 坐标均为 0。
    object_template = np.zeros((PATTERN_SIZE[0] * PATTERN_SIZE[1], 3), np.float32)
    object_template[:, :2] = (
        np.mgrid[0:PATTERN_SIZE[0], 0:PATTERN_SIZE[1]].T.reshape(-1, 2)
        * SQUARE_SIZE_M
    )

    object_points = []
    image_points = []
    image_size = None
    accepted = []
    rejected = []

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        40,
        0.001,
    )

    for path in image_paths:
        image = cv2.imread(str(path))
        if image is None:
            rejected.append((path.name, "无法读取"))
            continue

        current_size = (image.shape[1], image.shape[0])
        if image_size is None:
            image_size = current_size
        elif current_size != image_size:
            rejected.append((path.name, f"分辨率不一致：{current_size}"))
            continue

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCornersSB(
            gray,
            PATTERN_SIZE,
            flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE,
        )

        if not found:
            rejected.append((path.name, "未识别到完整 9x9 内角点"))
            continue

        corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
        object_points.append(object_template.copy())
        image_points.append(corners)
        accepted.append(path.name)

        preview = image.copy()
        cv2.drawChessboardCorners(preview, PATTERN_SIZE, corners, True)
        cv2.imwrite(str(output_dir / f"corners_{path.name}"), preview)

    print(f"照片总数：{len(image_paths)}")
    print(f"成功识别：{len(accepted)}")
    print(f"识别失败：{len(rejected)}")
    for name, reason in rejected:
        print(f"  - {name}: {reason}")

    if len(accepted) < 8:
        raise SystemExit("有效照片少于 8 张，暂不建议计算标定参数。请补拍不同角度的照片。")

    active = list(range(len(accepted)))
    dropped = []
    while True:
        obj_active = [object_points[i] for i in active]
        img_active = [image_points[i] for i in active]
        rms, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
            obj_active, img_active, image_size, None, None)
        per_image_errors = []
        for obj, img, rvec, tvec in zip(obj_active, img_active, rvecs, tvecs):
            projected, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, distortion)
            delta = projected.reshape(-1, 2) - img.reshape(-1, 2)
            per_image_errors.append(float(np.sqrt(np.mean(np.sum(delta * delta, axis=1)))))
        worst = int(np.argmax(per_image_errors))
        if per_image_errors[worst] <= args.max_view_rms or len(active) <= 10:
            break
        dropped.append((accepted[active[worst]], per_image_errors[worst]))
        del active[worst]

    output_file = output_dir / "camera_calibration.yaml"
    storage = cv2.FileStorage(str(output_file), cv2.FILE_STORAGE_WRITE)
    storage.write("image_width", image_size[0])
    storage.write("image_height", image_size[1])
    storage.write("pattern_columns", PATTERN_SIZE[0])
    storage.write("pattern_rows", PATTERN_SIZE[1])
    storage.write("square_size_m", SQUARE_SIZE_M)
    storage.write("rms_reprojection_error", float(rms))
    storage.write("camera_matrix", camera_matrix)
    storage.write("distortion_coefficients", distortion)
    storage.release()

    print("\n===== 标定完成 =====")
    print(f"图像分辨率：{image_size[0]} x {image_size[1]}")
    print(f"RMS 重投影误差：{rms:.6f} 像素")
    print(f"平均单图误差：{np.mean(per_image_errors):.6f} 像素")
    print(f"最大单图误差：{np.max(per_image_errors):.6f} 像素")
    print(f"最终使用照片：{len(active)} 张")
    for name, error in dropped:
        print(f"  - 剔除 {name}: {error:.6f} 像素")
    print("相机内参矩阵 camera_matrix：")
    print(camera_matrix)
    print("畸变参数 distortion_coefficients：")
    print(distortion.ravel())
    print(f"结果文件：{output_file}")
    print(f"角点预览：{output_dir}")


if __name__ == "__main__":
    main()
