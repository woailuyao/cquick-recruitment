import argparse
import os

import cv2
import numpy as np

from cquick_color_vision.core import write_image


def orange_layer(shape):
    hsv = np.zeros(
        (shape[0], shape[1], 3),
        dtype=np.uint8,
    )
    hsv[:, :] = (12, 210, 230)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def make_images(output_dir: str) -> None:
    os.makedirs(output_dir, exist_ok=True)

    width, height = 640, 480
    orange = orange_layer((height, width))
    orange_bgr = tuple(
        int(value) for value in orange[0, 0]
    )

    clear = np.full(
        (height, width, 3),
        (30, 35, 40),
        dtype=np.uint8,
    )
    cv2.circle(
        clear,
        (320, 240),
        70,
        orange_bgr,
        -1,
    )
    cv2.circle(
        clear,
        (305, 220),
        16,
        (120, 120, 120),
        -1,
    )

    small = np.full(
        (height, width, 3),
        (45, 50, 55),
        dtype=np.uint8,
    )
    cv2.circle(
        small,
        (505, 125),
        30,
        orange_bgr,
        -1,
    )

    similar = np.full(
        (height, width, 3),
        (100, 130, 140),
        dtype=np.uint8,
    )

    for x in range(0, width, 40):
        cv2.line(
            similar,
            (x, 0),
            (x, height),
            (112, 142, 152),
            2,
        )

    cv2.rectangle(
        similar,
        (0, 0),
        (width - 1, height - 1),
        (125, 155, 165),
        18,
    )

    cv2.circle(
        similar,
        (410, 300),
        35,
        orange_bgr,
        -1,
    )

    no_target = np.full(
        (height, width, 3),
        (180, 80, 30),
        dtype=np.uint8,
    )

    cv2.putText(
        no_target,
        "NO ORANGE TARGET",
        (150, 250),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.2,
        (255, 255, 255),
        3,
        cv2.LINE_AA,
    )

    models = {
        "clear.png": clear,
        "small.png": small,
        "similar_background.png": similar,
        "no_target.png": no_target,
    }

    for name, image in models.items():
        path = os.path.join(output_dir, name)

        if not write_image(path, image):
            raise RuntimeError(
                f"无法写入测试图片: {path}"
            )

        print(f"已生成: {path}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="生成 Week 2 四张测试图片",
    )
    parser.add_argument(
        "--output-dir",
        default="test_images",
    )

    args = parser.parse_args()
    make_images(args.output_dir)


if __name__ == "__main__":
    main()
