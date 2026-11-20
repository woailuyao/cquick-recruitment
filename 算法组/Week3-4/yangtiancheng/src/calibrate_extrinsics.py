from pathlib import Path

import cv2
import numpy as np


IMAGE_PATH = Path("/mnt/hgfs/codex_share/ground_chessboard.jpg")
OUTPUT_PATH = Path("/mnt/hgfs/codex_share/camera_extrinsics.yaml")

N = 9
SQUARE = 0.027
EXPECTED_CAMERA_HEIGHT = 0.685

# A 是靠近机器人一排的中央内角点，机器人坐标为 (0.750, 0, 0)。
A_X = 0.750
A_Y = 0.0

K = np.array(
    [
        [1490.3291634516265, 0.0, 690.3055808809594],
        [0.0, 1501.1247891348844, 327.3470954106697],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)
DIST = np.array(
    [0.15986872738605942, -0.9979708282542827,
     -0.002721193959994071, -0.0021791026978977722,
     1.3694314276100565],
    dtype=np.float64,
)


def grid_symmetries(grid):
    variants = []
    for k in range(4):
        rotated = np.rot90(grid, k, axes=(0, 1))
        variants.append((f"rot{k * 90}", rotated))
        variants.append((f"rot{k * 90}_flip", np.fliplr(rotated)))
    return variants


def main():
    image = cv2.imread(str(IMAGE_PATH))
    if image is None:
        raise SystemExit(f"无法读取照片：{IMAGE_PATH}")
    if (image.shape[1], image.shape[0]) != (1280, 720):
        raise SystemExit(f"照片必须为 1280x720，当前为 {image.shape[1]}x{image.shape[0]}")

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCornersSB(
        gray,
        (N, N),
        flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE,
    )
    if not found:
        raise SystemExit("未识别到完整 9x9 内角点")

    image_grid = corners.reshape(N, N, 2)

    # 物理网格：第 0 行靠近机器人，第 8 行远离机器人；
    # 第 0 列在机器人左侧，第 8 列在机器人右侧。
    object_grid = np.zeros((N, N, 3), dtype=np.float64)
    for row in range(N):
        for col in range(N):
            object_grid[row, col] = [
                A_X + row * SQUARE,
                A_Y + (N // 2 - col) * SQUARE,
                0.0,
            ]
    object_points = object_grid.reshape(-1, 3)

    candidates = []
    for name, candidate_grid in grid_symmetries(image_grid):
        image_points = candidate_grid.reshape(-1, 2).astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            K,
            DIST,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            continue

        r_robot_to_camera, _ = cv2.Rodrigues(rvec)
        r_camera_to_robot = r_robot_to_camera.T
        camera_position_robot = (-r_camera_to_robot @ tvec).reshape(3)

        projected, _ = cv2.projectPoints(object_points, rvec, tvec, K, DIST)
        rms = float(np.sqrt(np.mean(np.sum(
            (projected.reshape(-1, 2) - image_points) ** 2, axis=1
        ))))

        # 相机安装在机器人中心附近，且高度应接近实测的 0.685 m。
        position_score = (
            abs(camera_position_robot[2] - EXPECTED_CAMERA_HEIGHT) * 10.0
            + abs(camera_position_robot[1]) * 2.0
            + max(0.0, abs(camera_position_robot[0]) - 0.40)
        )
        if camera_position_robot[2] <= 0:
            position_score += 100.0

        candidates.append((
            position_score + rms * 0.01,
            name,
            rms,
            r_camera_to_robot,
            camera_position_robot,
        ))

    if not candidates:
        raise SystemExit("solvePnP 未得到有效结果")

    candidates.sort(key=lambda item: item[0])
    _, symmetry, rms, rotation, translation = candidates[0]

    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation

    lines = [
        "# camera(x right, y down, z forward) -> robot(x forward, y left, z up)",
        "source_image: /mnt/hgfs/codex_share/ground_chessboard.jpg",
        f"reprojection_rms_px: {rms:.9f}",
        f"selected_grid_symmetry: {symmetry}",
        "camera_position_in_robot_m:",
        f"  x: {translation[0]:.9f}",
        f"  y: {translation[1]:.9f}",
        f"  z: {translation[2]:.9f}",
        "extrin:",
    ]
    for row in transform:
        lines.append("  - [" + ", ".join(f"{value:.12f}" for value in row) + "]")
    OUTPUT_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")

    preview = image.copy()
    cv2.drawChessboardCorners(preview, (N, N), corners, True)
    preview_path = OUTPUT_PATH.with_name("external_corners_preview.jpg")
    cv2.imwrite(str(preview_path), preview)

    print("===== 外参计算完成 =====")
    print(f"网格方向：{symmetry}")
    print(f"重投影 RMS：{rms:.6f} px")
    print("相机在机器人坐标系中的位置（m）：")
    print(f"x={translation[0]:.6f}, y={translation[1]:.6f}, z={translation[2]:.6f}")
    print("相机到机器人 4x4 外参：")
    print(transform)
    print(f"结果：{OUTPUT_PATH}")
    print(f"角点预览：{preview_path}")


if __name__ == "__main__":
    main()
