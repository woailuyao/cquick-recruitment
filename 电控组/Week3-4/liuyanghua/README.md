# 电控第 3-4 周提交说明 · <刘洋华>

注：本文档由第二周README模板改造而来，纯手写（

## 一、基本信息

* 姓名拼音：<liuyanghua>
* 姓名：<刘洋华>
* 完成时间：<2026-11-20>
* 运行环境：Ubuntu 22.04 + ROS2 Humble

## 二、四项任务完成情况

### 任务 1 · 点亮一个舵机

* 已正常完成，关于装舵臂，非我之责也
* 关于接线：USB转TTL接到24路控制板的接线就是RX对TX，TX对RX，GND对GND，上面都有标注；舵机连控制板就是用螺旋缠绕的那个线，有防呆不怕插反，总之对应颜色就行；电池的TX30头也有防呆，总之红对红、黑对黑就行
- 附：[舵机转动录制](https://www.bilibili.com/video/av117653196838851/?vd_source=623a2b698bc65ad40223652abf39a7e0)

### 任务 2 · 让 ROS2 接管这个舵机

* 包名：<cquick\_servo> <cquick\_servo\_interfaces>
* 程序流程：发送goal -> 服务端校验goal合法性 -> 接受或拒绝 -> 开始执行 -> 【独立进程运行execute\_callback -> 写入上力指令、读位置指令、运动指令 -> 设置定时器发布feedback，同时设置wait防超时 -> feedback反复读位置并发布 -> 检测到运动到位或超时就结束action】 -> 客户端得到result
* 关于硬件：USB -> 转TTL -> 24路控制板 -> 舵机
* 关于bag：录制了2个bag，一个是按照步骤6布设置参数，一个是正反转加回中位，均有录屏
* [按照步骤6布设置参数](https://www.bilibili.com/video/BV18CsB69ECp/?vd_source=623a2b698bc65ad40223652abf39a7e0)
* [正反转加回中位](https://www.bilibili.com/video/BV1t2sB6QEXA/?vd_source=623a2b698bc65ad40223652abf39a7e0)

### 任务 3 · 复制到整机 + 多舵机动作组

* 关于代码：我用AI写了一个扫描ID<search\_id.py>和一个发送命令<test.py>的脚本，可以一键扫描、一键卸力、一键读位置，非常方便
* 关于yaml：使用方法： sequence\_player节点会默认读取config/actions/execute.yaml动作文件，如果需要更改动作组，需要将新的动作组粘贴到execute.yaml中，而且colcon build时也只会install那个execute.yaml
yaml结构： 本yaml采用一维数组+分隔符的形式，每一步动作由<-114,-114,-114,>隔开，**注意最后一行也必须写分隔符且末尾不能有逗号，否则不会运行最后一步或yaml出错**，我们组共同设计了两套yaml，<squat>是站直->蹲下->站直，<step>是模拟迈一次左脚
* 其他说明：之所以采用一维数组，是因为多维数组会报错，我不清楚yaml格式到底支不支持多维数组，但是换成一维数组确实不会报错了，所以就这样了；此外，yaml中还有一个<repeat>参数，代表动作组重复的次数，默认为3；
附：一键启动命令：<ros2 launch cquick\_servo servo.launch.py servo\_ids:=\[0,1,2,3,4,5,6,7,8,9]>
附：[yaml1动作组录制](https://www.bilibili.com/video/BV1hmypzyE4p/?vd_source=623a2b698bc65ad40223652abf39a7e0)  |  [yaml2动作组录制](https://www.bilibili.com/video/BV13YyxzQEA8/?vd_source=623a2b698bc65ad40223652abf39a7e0)

### 任务 4 · 安全下电

* 步骤：用我编辑过的test.py一键卸力，之后关控制板电源，拔电池，最后sudo shutdown now关闭虚拟机
* 误操作后果：USB转TTL插错的话回显和指令会出现异常，但是舵机不会坏；如果和电源相关的插错...总之舵机命不久矣，但是这么多防呆还能插错那是真的有了
* 其他操作：电池量电压是用白头接到那个小仪表（叫啥？）的最左边的3个针
* 附：[安全下电录制](https://www.bilibili.com/video/BV1hYyxzQErS/?vd_source=623a2b698bc65ad40223652abf39a7e0)

## 三、踩过的坑 \& 怎么解决的

1. <串口成功打开但是无法联系舵机> → <大概率是没开电> → <开电>
2. <能控制舵机运动但是无回显> → <USB转TTL有一根线断了> → <打螺丝>
3. <无法打开串口> → <要么是串口名字变了，要么是未知原因> → <ls -l 查串口名，虚拟机里重连，重新插拔>
4. <write指令后无反应> → <没flush(为什么AI会看不出来啊！？)> → <改代码>
5. <能连接舵机但是运动指令无反应> → <说不定是因为本来就在目标位置导致看不出来呢> → <换个指令再试>
6. <播放动作序列时有两个舵机不同步> → <发现是最大进程数量写少了> → <进程数量改大>
7. <一个动作组结束后服务端程序崩溃> → <feedback函数调用了已销毁的timer导致的> → <不销毁timer只取消（怎么感觉越来越屎山了）>
8. <为了bag播放导致程序出现一些其他问题> → <我真累了哥> → <bag摆了，能动就行>
9. <太多了忘完了>
10. ......

