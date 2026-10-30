\#第二周考核 · <王禹萱>



姓名：王禹萱

完成日期：10.30

版本：Ubuntu22.04, ROS2    ,Python3.13, OpenCV5.0.0.93

图片来源：豆包AI生成



\## 任务 1 ：颜色目标检测

目标颜色：黄色

HSV阈值：**lower\_yellow = np.array(\[20, 100, 100])**

&#x20;                 **upper\_yellow = np.array(\[35, 255, 255])**

最小面积：200

中心点计算方法：通过画出外接矩形轮廓，找到左上角坐标点、宽和高，进而计算中心点坐标**cx=int(x+w/2)； cy=int(y+h/2)**



\##任务 2 ：可视化与参数对比

HSV阈值：左  lower\_yellow = np.array(\[21, 180, 150])            右 lower\_yellow = np.array(\[22, 200, 160])

&#x20;                     upper\_yellow = np.array(\[23, 255, 250])                upper\_yellow = np.array(\[25, 255, 250])

&#x20;                筛选结果发生变化。但此为误检案例，背景其他物体与目标小球颜色相近重合，通过反复改变区间测试也无法找到合适HSV阈值

最小面积：左 200   右16000                   

&#x20;                     结果：较小的无关物体被排除在外

是否使用形态学闭运算：左：否；右：是



\##任务 3：ROS2 通信

消息字段：bool detected

* &#x20;          float32 u

&#x20;                float32 v

&#x20;                float32 area

话题名：target\_position

发布  订阅流程：

编辑消息包 target\_interfaces，使 ROS2 系统识别该消息类型；发布端向话题target\_position 封装并推送；订阅端订阅同名话题，等待接收消息



\##仍未解决的问题

1\.如何找到较为准确的HSV数值以识别对应目标物体，而不会因被背景相近而识别出多个

2\.未了解清楚ROS2的使用方法

&#x20; 



