import argparse
import sys

from cquick_color_vision.core import (
    detect_target,
    draw_detection,
    read_image,
    write_image,
)
from cquick_color_vision.detector import (
    add_detector_arguments,
    config_from_args,
)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="生成带轮廓、中心点与坐标的标注图",
    )

    add_detector_arguments(parser)
    parser.set_defaults(save="02_结果可视化.png")

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
    output = draw_detection(image, detection, config)

    if not write_image(args.save, output):
        print(
            f"[错误] 标注图保存失败: {args.save}",
            file=sys.stderr,
        )
        raise SystemExit(3)

    if detection.detected:
        print(
            f"检测成功: u={detection.u:.2f}, "
            f"v={detection.v:.2f}, "
            f"area={detection.area:.2f}"
        )
    else:
        print("未检测到目标，已在图像中显示提示")

    print(f"已保存: {args.save}")


if __name__ == "__main__":
    main()
