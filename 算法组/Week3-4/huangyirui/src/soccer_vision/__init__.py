#可以跟着一张照片读这个包。
#detect 找出目标框，refine 在框里找白线特征点，geometry 计算地面位置，field_map 再用已知场地图判断位姿。
#视觉定位包，configio读配置，video_source读相机或视频
#localizer把检测、投影和地图匹配串起来，visualize画结果，run_demo启动程序

#双下划线开头和结尾的名字是 Python 的特殊约定，__version__ 不是强制的，
#但很多工具（pip、日志、--version 参数）都会去读它。
__version__ = "0.1.0"#包版本号，供 --version 与日志打印
