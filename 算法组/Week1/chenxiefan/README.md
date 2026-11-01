开发环境：Python 3.10
安装依赖：OpenCV、NumPy、Matplotlib。

依赖安装方式：
命令：
pip install opencv-python numpy matplotlib

运行命令：
请先确认目录为纯英文
启动
src/task1_basics.py
src/task2_processing.py
src/task3_color_detect.py
路径均采用项目内相对路径
task2&3的中间结果已经保存在outputs文件夹内的task2&3内"中间结果"文件夹内

关键参数：
cv2.imread()————flags,1读彩色，0读灰度，-1读原图
cv2.resize()————dsize，宽在前高在后，与img.shape相反；fx,fy，按比例缩放；interpolation，INTER_AREA缩小用, INTER_LINEAR默认, INTER_CUBIC放大用
cv2.inRange()————lowerb, upperb，下限与上限，格式：np.array([H,S,V])
cv2.GaussianBlur()————ksize，(n,n)必须是奇数，越大越模糊
cv2.threshold()————thresh，阈值，用了THRESH_BINARY + THRESH_OTSU会自己算
cv2.Canny()————threshold1,threshold2，控制弱、强边缘
np.ones()————shape,dtype，形态学处理的核，越大越夸张

失败案例：
任务3中，一张图片拍摄了物体的阴面，导致不论怎么调节HSV参数也无法生成正确的掩膜。

改进思路：
认识到了对于过于明亮或黑暗的图片不能统一采用同一个函数来处理，于是更换了一张能够与既有的函数兼容的素材图片。

问题记录：
能否先将图片的暗部亮度提高，再参与掩膜生成？