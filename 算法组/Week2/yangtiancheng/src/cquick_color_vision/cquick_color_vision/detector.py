import argparse
import sys

import cv2

from cquick_color_vision.core import (
    DetectorConfig,
    detect_target,
    draw_detection,
    read_image,
    write_image,
)


def add_detector_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--image", required=True, help="输入图片路径")

    parser.add_argument("--h-min", type=int, default=5)
    parser.add_argument("--h-max", type=int, default=25)
    parser.add_argument("--s-min", type=int, default=100)
    parser.add_argument("--s-max", type=int, default=255)
    parser.add_argument("--v-min", type=int, default=70)
    parser.add_argument("--v-max", type=int, default=255)

    parser.add_argument("--min-area", type=int, default=800)
    parser.add_argument("--kernel-size", type=int, default=5)

    parser.add_argument(
        "--morph-mode",
        choices=[
            "none",
            "open",
            "close",
            "open_close",
            "close_open",
        ],
        default="open_close",
    )

    parser.add_argument(
        "--save",
        default="",
        help="标注结果保存路径",
    )

    parser.add_argument(
        "--show",
        action="store_true",
        help="运行后打开结果窗口",
    )


def config_from_args(args: argparse.Namespace) -> DetectorConfig:
    config = DetectorConfig(
        h_min=args.h_min,
        h_max=args.h_max,
        s_min=args.s_min,
        s_max=args.s_max,
        v_min=args.v_min,
        v_max=args.v_max,
        min_area=args.min_area,
        kernel_size=args.kernel_size,
        morph_mode=args.morph_mode,
    )

    config.validate()
    return config


def main() -> None:
    parser = argparse.ArgumentParser(
        description="HSV 颜色目标检测",
    )

    add_detector_arguments(parser)
    args = parser.parse_args()

    image = read_image(args.image)

    if image is None:
        print(
            f"[错误] 图片读取失败: {args.image}",
            file=sys.stderr,
        )
        raise SystemExit(2)

    config = config_from_args(args)
    detection = detect_target(image, config)

    height, width = image.shape[:2]

    print(f"图片: {args.image}")
    print(f"图片尺寸: width={width}, height={height}")
    print(
        "HSV 阈值: "
        f"lower=({config.h_min}, {config.s_min}, {config.v_min}), "
        f"upper=({config.h_max}, {config.s_max}, {config.v_max})"
    )
    print(f"最小轮廓面积: {config.min_area}")
    print(
        f"形态学处理: {config.morph_mode}, "
        f"kernel={config.kernel_size}"
    )

    if detection.detected:
        print("检测状态: 成功")
        print(
            f"u={detection.u:.2f}, v={detection.v:.2f}, "
            f"area={detection.area:.2f}"
        )
    else:
        print("检测状态: 未检测到目标")
        print("u=0, v=0, area=0 (detected=false)")

    if args.save:
        output = draw_detection(
            image,
            detection,
            config,
        )

        if not write_image(args.save, output):
            print(
                f"[错误] 标注图保存失败: {args.save}",
                file=sys.stderr,
            )
            raise SystemExit(3)

        print(f"标注图已保存: {args.save}")

    if args.show:
        cv2.imshow(
            "Detection",
            draw_detection(image, detection, config),
        )
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
