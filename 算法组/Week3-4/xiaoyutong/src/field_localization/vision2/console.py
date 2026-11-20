"""控制台编码兼容。

Windows 的 cmd 默认是 GBK（代码页 936），直接 print 中文或特殊符号可能报错；
Linux / macOS 默认是 UTF-8。这里统一处理：

* 如果外部已经用 PYTHONIOENCODING 指定了编码，就沿用该编码，只补一个 errors 兜底；
* 否则切到 UTF-8。
"""

import os
import sys


def setup_console():
    forced = os.environ.get("PYTHONIOENCODING", "").strip()
    for stream in (sys.stdout, sys.stderr):
        try:
            enc = (forced.split(":")[0] if forced else "utf-8") or (stream.encoding or "utf-8")
            stream.reconfigure(encoding=enc, errors="replace")
        except Exception:
            pass
