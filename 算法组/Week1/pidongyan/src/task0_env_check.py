
import platform
import sys
from datetime import datetime
import cv2
import numpy as np

#  记录环境信息

print(f"操作系统: {platform.system()} {platform.release()}  ({platform.platform()})")
print(f"Python 版本: {sys.version.split()[0]}")
print(f"解释器路径: {sys.executable}")
print(f"OpenCV 版本: {cv2.__version__}")
print(f"NumPy 版本: {np.__version__}")
print(datetime.now())
print(datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
print(f"{datetime.now():%H:%M:%S}")
print("Pi Dongyan")



