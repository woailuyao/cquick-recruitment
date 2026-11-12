# 当前进度（开发记录，不是考核完成证明）

## 已实现

- HSV 分割、面积筛选、最大候选、轮廓矩质心、未检测到的零值约定。
- 可选开/闭运算、红色色相跨界、配置校验。
- 批量检测 CLI、逐图终端输出、轮廓/中心/坐标标注、PNG/mask/JSON 保存。
- ROS2 接口包、发布节点、订阅节点；发布数据调用实际检测函数；运行时检测参数支持。
- 已确认考核署名 liyutong（与 Linux 登录名 liyut 不同），已填写 README 并整理 SCREENSHOTS.md；使用用户要求的 echo 姓名与 date 时间戳格式。
- 参数实验工具、安装和构建脚本、运行说明、理解与现场问答资料。
- tools/compare_parameters.py 自动执行 baseline/饱和度/面积三个配置，保存两项单变量实验及 CSV；已用两张合成样例验证六行记录，现象和解释保留为空。

## 已实际验证

### Windows 开发自检

独立 Python 3.12 环境，OpenCV 5.0.0（分发包 opencv-python-headless 5.0.0.93）、NumPy 2.5.4。
运行命令：`.venv/Scripts/python.exe tools/self_check.py`。

通过：最大轮廓选择和已知中心 (200,120)、无目标零值、面积过滤、零矩轮廓跳过、红色 H 跨界、非法参数拒绝、四种形态学模式、中文路径、保存 PNG 可重新解码、图片不存在返回失败、Python 语法、XML 清单解析、消息字段文本。记录在 dev_checks/synthetic_only。

### WSL2 / Ubuntu / ROS2

- Windows 重启已由用户确认。发行版 Ubuntu-22.04 通过官方根文件系统导入成功，实测 Ubuntu 22.04/jammy，WSL2 内核 6.18.54.1，Linux 文件系统可访问。
- 已创建 liyut（uid=1000，sudo 组）并设为默认用户；用户在本机完成密码设置，报告 passwd: password updated successfully。未索取或记录密码。
- tools/install_ros2_humble.sh 实际执行成功；Ubuntu 与 ROS 官方源可用，ros2-apt-source 下载 SHA256 校验通过。
- ROS2 Humble、Python 3.10.12、OpenCV 4.5.4、NumPy 1.21.5、rclpy、colcon 已验证可用。
- 以 liyut 普通用户执行 tools/build_ros2.sh，两个包构建成功。工作空间 /home/liyut/ros2_ws，包源链接到 Windows 项目 src。构建日志 dev_checks/ubuntu_build.log。
- Ubuntu 下重复算法自检通过；记录 dev_checks/ubuntu_synthetic_only 和 dev_checks/ubuntu_self_check.log。
- tools/ros2_smoke_check.py 实际运行 PASS：生成的自定义消息四字段一致；启动真实 ros2 run 发布和订阅进程；父验证节点收到 12 条基线消息，有目标中心 (200,120)、面积 2736，未检出状态为全零。
- 实际订阅进程日志记录 48 条消息，invalid_count=0；真实运行 min_area=10000.0 更新成功，随后连续 12 条为未检出；非法 -1.0 更新被拒绝且保留旧配置。
- ros2 interface show、ros2 topic info --verbose 返回成功。测试话题为隔离命名空间下的 /synthetic_smoke_38c8d16036c6/target_position，由默认话题重映射而来；默认 /target_position 的真实照片演示仍待进行。
- 另执行 --default-topic 模式，未添加任何话题 remap，直接在 /target_position 上验证成功；实际订阅节点同样收到 48 条、invalid_count=0，动态参数检查通过，两个子进程均退出 0 且清理无错误。记录 dev_checks/ros2_default_topic_synthetic_001/summary.json；姓名 liyutong 与真实 date 输出保存于 dev_checks/liyutong_default_topic_check.log。此轮仍使用合成开发图，不是最终真实照片证据。
- ROS2 结果、时间、版本和进程清理结果见 dev_checks/ros2_synthetic_001/summary.json、subscriber.log、publisher.log、interface_show.log、topic_info.log。测试结束已清理测试进程。

所有上述自检图片均为程序生成的开发样例，不能代替真实场景考核照片和截图。PNG 重新解码通过；当前模型不支持图像输入，未作目视图像评审。

## 真实遇到的安装问题与处理

1. Windows 默认 Python 3.14 没有 cv2。项目建立独立 Python 3.12 环境做 Windows 自检；ROS2 使用 Ubuntu 系统 Python 和 apt 依赖。
2. 普通 wsl --install 访问 GitHub 发行版列表报 WININET_E_CONNECTION_RESET。
3. 改用微软官方直链 Ubuntu2204-221101.AppxBundle，下载长度 1116834851，MD5 与 HTTP Content-MD5 一致（2D4F98AE9E7F1921722DA620EF72B643）。应用包安装成功，但启动注册报 0x80071772，Windows 解释为加密文件无法解密；未证明具体加密原因。
4. 从同一官方包提取 x64 install.tar.gz，使用 wsl --import Ubuntu-22.04 ... --version 2 成功注册，随后可实际启动。
5. 工具沙箱内 WSL 返回 E_ACCESSDENIED；按权限机制对同一条命令扩大权限重试。该限制与 Linux 用户密码无关。

Linux 虚拟磁盘位于工作区 wsl-installations/Ubuntu-22.04，该目录是系统数据，需保留。项目 downloads 目录中的安装包已排除 Git 提交。

## 真实照片阶段已完成

- 用户上传五张图片，按 clear/small/background/lighting/absent 保存附件副本，未修改原附件。
- 默认基线与 S 下界100→200、min_area100→200000 两项单变量实验均运行；同色书本混入形成误检，提高面积导致小目标漏检。
- 选定 [5,200,80]～[25,255,255]、min_area100、none，四张有目标图检出，一张无目标正确返回False。阴影混入和暗光轮廓缺失仍存在，不宣称精确分割。
- 在 Ubuntu 上用同一批真实图片、默认 /target_position 测试PASS，订阅端30条、invalid_count0；独立观察20条逐条与 detect 输出比对一致；interface/topic info/echo成功，所有子进程退出0。证据 outputs/real_ros_check_001。
- 已生成 REAL_PHOTO_REPORT.md、outputs/real_summary/five_photo_results.png 和 parameter_comparisons.png。图像汇总与同色背景/暗光标注图已目视检查，更新先前模型无法看图的限制。汇总图不是终端截图。

## 运行截图（已完成）

2026-11-01 通过真实终端窗口捕获并保存于 outputs/screenshots：01_detection、03_interface_show、03_pub_sub、03_topic_info、04_parameter_change 含姓名 liyutong 与实际 date 时间戳及对应运行结果。另有三张用真实输出文件排版生成的合成图（01_five_photo_summary、02_result_comparison、02_parameter_comparison），图中标注姓名与生成时间，但不是终端截图。

## 提交材料整理

- README 已按考核模板改成固定小节：任务 1 / 任务 2 / 任务 3 / 加分项 / 问题与解决过程 / 仍未解决的问题 / 复现方式，内容全部来自实测数据。
- 新增第一人称 学习记录.md，记录环境报错、编码乱码、误检漏检等实际过程。
- 提交目录已移除 LEARNING_GUIDE.md（该文件是问答资料，不作为个人学习材料提交；项目根目录仍保留供复习）。
- 提交目录共 100 个文件约 87.7 MB，主要是全尺寸标注 PNG，可按需精简后再推送。

## 尚未完成

- 用户审阅后处理 GitHub 提交/PR（用户尚未 Fork 仓库）。

环境、代码、真实照片实验与通信验证已完成，整个考核尚未完成。
