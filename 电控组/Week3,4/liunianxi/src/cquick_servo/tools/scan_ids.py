"""兼容入口：真正的实现在 cquick_servo/tools/scan_ids.py。

保留这个文件是为了老的直接运行方式（python3 tools/scan_ids.py）还能用。
"""
from cquick_servo.tools.scan_ids import main

if __name__ == "__main__":
    main()
