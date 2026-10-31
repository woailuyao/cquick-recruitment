import cv2
import os

current_script_path=os.path.abspath(__file__)
src_dir=os.path.dirname(current_script_path)
project_root=os.path.dirname(src_dir)
image_path=os.path.join(project_root, "../data", "images", "test2.jpg")
output_dir = os.path.join(project_root, "../outputs", "task2")

image_path= "../data/images/test2.jpg"
img=cv2.imread(image_path)
if img is None:
    print("读取失败")
else:
    print("读取成功")
    gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    cv2.imwrite("../outputs/task2/gray.jpg", gray)

    blur=cv2.GaussianBlur(gray,(5,5),0)
    cv2.imwrite("../outputs/task2/blur.jpg", blur)

    ret,thresh=cv2.threshold(blur,127,255,cv2.THRESH_BINARY)
    cv2.imwrite("../outputs/task2/thresh(127).jpg", thresh)

    ret,thresh=cv2.threshold(blur,90,255,cv2.THRESH_BINARY)
    cv2.imwrite("../outputs/task2/thresh(90).jpg", thresh)

    edge=cv2.Canny(blur,50,150,)
    cv2.imwrite("../outputs/task2/edge.jpg", edge)

    kernel=cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
    closed=cv2.morphologyEx(edge,cv2.MORPH_CLOSE,kernel)
    cv2.imwrite("../outputs/task2/closed1.jpg", closed)

    kernel=cv2.getStructuringElement(cv2.MORPH_RECT,(10,10))
    closed=cv2.morphologyEx(closed,cv2.MORPH_CLOSE,kernel)
    cv2.imwrite("../outputs/task2/closed2.jpg", closed)

    print("任务完成")