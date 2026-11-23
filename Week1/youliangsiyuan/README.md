# 第 1 周考核 · youliangsiyuan

- 姓名：游梁思远
- 联系方式：2562024916@qq.com
- 姓名拼音：youliangsiyuan
- 完成日期：2026-10-21

## 任务 1：系统安装与 ROS2 环境自检

用 VirtualBox 7.2.20 创建虚拟机（4 核 CPU / 8GB 内存 / 40GB 动态磁盘），安装 Ubuntu 22.04.5 桌面版；
随后用鱼香 ROS 一键脚本安装 ROS2 Humble（换国内源、安装桌面版、安装 rosdep）。

自检结果：

- `printenv ROS_DISTRO` 输出 `humble`；
- `ros2 --help` 正常输出帮助信息；
- 已安装 `ros-humble-rclcpp` 16.0.21、`ros-humble-ros2cli` 0.18.20（见截图）。

遇到的坑：

- 双击 `VBoxLinuxAdditions.run` 时 Ubuntu 默认用文本编辑器打开，根本没安装，经过 AI 指导后在终端里用 `sudo bash` 执行才行；
- 手打光盘路径时把 `media` 打成了 `meida`，报"没有那个文件或目录"；改用 `cd /media/$USER/VBox_GAs_*` 加 Tab 补全后解决；
- `wget http://fishros.com/install -O fishros` 里把大写字母 `O` 打成了数字 `0`，wget 报"无效的选项"，改正后正常；
- 装增强功能之前要先 `apt install build-essential dkms linux-headers-$(uname -r)`，否则内核模块编译不过。

## 任务 2：小乌龟

节点感觉就是两个互相不干扰的进程,比如这里面turtlesim_node负责显示乌龟模拟运动;teleop_key负责读取键盘,互相不干扰.但是节点之间没办法直接互相调用或者影响,需要一个通道来进行数据的交换,话题就承担了数据传输的作用,一个节点发布数据到话题,其他的节点可以通过这个话题来读取相应的指令和数据,从而实现协作.

## 任务 3：Git 提交

这次是我第一次完整走 Fork + Pull Request 流程：

一开始我以为直接在本地 `git push` 到考核仓库就行，clone 下来才发现自己对仓库只有读权限，我想改只能"请求合并"。后来通过ai理解这个设计：Fork 相当于把仓库复制一份到我自己的 GitHub 账号下我才有写的权限；Pull Request 则是把我的提交和主仓库 `main` 做对比，把差异交给评审决定要不要合入。

踩过的坑：

- 第一次 commit 前没配 `user.name` / `user.email`，git 提示 `Please tell me who you are`，配完才提交成功；
- PR 的 base / head 方向容易弄反：base 是主仓库 `main`，head 是我 Fork 的 `main`。

体会：Fork + PR 就像"在自己的草稿本上改好，再交上去给老师批阅"——既不会动到主仓库，又能留下完整的提交记录。

---

> 截图防抄袭说明：每张关键截图均在同一终端先执行
> `echo "姓名: youliangsiyuan"; date "+%Y-%m-%d %H:%M:%S"`
> 再执行对应命令后截图，图中同时包含姓名拼音与实时时间戳。
