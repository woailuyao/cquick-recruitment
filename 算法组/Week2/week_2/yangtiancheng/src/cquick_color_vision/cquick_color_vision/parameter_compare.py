import argparse

import cv2
import numpy as np

from cquick_color_vision.core import (
    DetectorConfig,
    detect_target,
    draw_detection,
    read_image,
    write_image,
)


def build_experiments():
    return [
        (
            "A: baseline",
            DetectorConfig(),
        ),
        (
            "B: wider HSV (S>=40, V>=50)",
            DetectorConfig(s_min=40, v_min=50),
        ),
        (
            "C: min_area=5000",
            DetectorConfig(min_area=5000),
        ),
    ]


def make_tile(
    label: str,
    image: np.ndarray,
    detected: bool,
    u: float,
    v: float,
) -> np.ndarray:
    resized = cv2.resize(
        image,
        (480, 360),
        interpolation=cv2.INTER_AREA,
    )

    header = np.full(
        (76, 480, 3),
        245,
        dtype=np.uint8,
    )

    status = (
        f"detected={detected}, "
        f"u={u:.1f}, v={v:.1f}"
    )

    cv2.putText(
        header,
        label,
        (10, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (20, 20, 20),
        2,
        cv2.LINE_AA,
    )

    cv2.putText(
        header,
        status,
        (10, 58),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (20, 20, 20),
        1,
        cv2.LINE_AA,
    )

    return np.vstack([header, resized])


def main() -> None:
    parser = argparse.ArgumentParser(
        description="一次只改变一个主要参数的对比实验",
    )

    parser.add_argument(
        "--image",
        default="test_images/similar_background.png",
    )
    parser.add_argument(
        "--save",
        default="02_参数对比.png",
    )

    args = parser.parse_args()
    image = read_image(args.image)

    if image is None:
        print(f"[错误] 图片读取失败: {args.image}")
        raise SystemExit(2)

    tiles = []

    for label, config in build_experiments():
        detection = detect_target(image, config)

        annotated = draw_detection(
            image,
            detection,
            config,
        )

        tiles.append(
            make_tile(
                label,
                annotated,
                detection.detected,
                detection.u,
                detection.v,
            )
        )

        print(
            f"{label}: detected={detection.detected}, "
            f"u={detection.u:.2f}, "
            f"v={detection.v:.2f}, "
            f"area={detection.area:.2f}"
        )

    canvas = np.hstack(tiles)

    if not write_image(args.save, canvas):
        print(f"[错误] 对比图保存失败: {args.save}")
        raise SystemExit(3)

    print(f"参数对比图已保存: {args.save}")


if __name__ == "__main__":
    main()
