# 第 1 周考核 · WangZiai

- 姓名：王子艾
- 姓名拼音：WangZiai
- 学号 / 联系方式：20254624 / 2263798225@qq.com
- 完成日期：2026-10-22

---

## 任务 1：系统安装与 ROS2 环境自检

| 项目 | 内容 |
| --- | --- |
| 环境方案 | 虚拟机（VMware Workstation） |
| 虚拟机版本 | VMware Workstation 111 Pro 26H1u1 |
| 操作系统 | Ubuntu 22.04 |
| ROS2 版本 | Humble |

**遇到的问题：** 装 ROS2 时未发现问题，脚本一次跑通。

---

## 任务 2：小乌龟

### 我对「节点」和「话题」的理解

1. **节点**是我们运行的程序；
2. **话题**是节点之间交换数据的通道；
3. 每个话题可以连接着多个节点，
   可将数据信息从一个节点中传递到其他节点中。

### 具体到我在实际操作中的理解

**两个节点：**

| 节点 | 职责 |
| --- | --- |
| `/teleop_turtle` | 接收键盘响应（外部激励）的程序 |
| `/turtlesim` | 处理乌龟具体参数并渲染图像的程序 |

**三条话题：**

| 话题 | 方向 | 传输的内容 |
| --- | --- | --- |
| `/turtle1/cmd_vel` | `/teleop_turtle` → `/turtlesim` | 运动数据 |
| `/turtle1/pose` | `/turtlesim` → `/teleop_turtle` | 乌龟的坐标与朝向 |
| `/turtle1/color_sensor` | `/turtlesim` → `/teleop_turtle` | 图像内容 |

### 疑问与解答：rqt 图里的话题名和 `ros2 topic list` 对不上？

**现象**

我一开始没能理解，为什么 rqt 分析程序（rqt_graph）里节点之间那座"桥"上显示的话题名，
和 `ros2 topic list` 列出来的话题名不完全一样。

*（在我询问 DSH 后获得了解答）*

**原因**

rqt_graph 默认只显示"当前正在通信"的话题，而 `ros2 topic list` 会把系统里
**所有已注册的话题**都列出来。

**所以：**

我在 rqt 图里看到的 `/turtle1/cmd_vel` 加上两条
`/turtle1/rotate_absolute/_action/feedback`、`.../status`，就是勾选了 `Actions` 之后
多出来的 —— **`rotate_absolute` 这个功能在 ROS2 Humble 里是用 action 实现的，
不是普通话题**，所以 `ros2 topic list` 和 rqt 图的显示范围天生就不一样。

**结论**

> `ros2 topic list` 是"系统里有哪些话题名"，rqt_graph 是"现在谁在跟谁通信"。

---

## 任务 3：Git 提交

**学到的流程**

```
Fork 主仓库 → Clone 自己的 Fork → 按目录规范放截图和 README
→ git add / git commit / git push → 向主仓库发起 Pull Request
```

**遇到的困难：** 第一次走 Fork + PR 流程，对 Git 的基础终端指令还不是很熟悉，对流程内容也一知半解。

**怎么解决的：** 把考核细则文档喂给了 DSH（DeepSeek Harness），在它的讲解下理清了具体流程。

**感想：** 终端指令比较简单，对这套协作规则的理解则比较重要。

---

> 说明：本 README 的文本内容均由本人手写完成，排版与格式仅借助 DSH 协助优化。
