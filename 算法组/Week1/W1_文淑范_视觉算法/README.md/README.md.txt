hsv lower / upper HSV颜色筛选上下限，调整该范围来匹配目标颜色
kernel_size 形态学运算核大小，用于开运算降噪；核越大降噪能力越强，容易抹除小目标
min_area 轮廓最小面积阈值，过滤面积小于该值的噪声轮廓
scale_x, scale_y 图像缩放系数，对原图进行缩放预处理


初次运行 task3_color_detect.py 时，生成的掩膜 mask_clean 整张空白全黑，程序没有报错，但检测不到任何目标轮廓。
使用取色工具在原图采样目标颜色，重新标定 lower、upper；