# bag 数据说明

本目录是任务 2「ros2 bag 回放复现」的配套录制数据，可直接用于：

```bash
ros2 bag info bag/run1
ros2 bag play bag/run1
```

## 内容

| 文件 | 说明 |
|---|---|
| `run1_0.db3` | rosbag2 的 SQLite 数据文件，含 10 条真实消息 |
| `metadata.yaml` | bag 的说明书（话题、消息数、时间戳、序列化格式） |

- 消息总数：**10**
- 时长：**22min 51s 829ms**（2026-11-22 15:38:18 ~ 16:01:10，北京时间）
- 话题：
  - `/servo/command`（`cquick_servo_interfaces/msg/ServoCommand`）× 6
  - `/servo/state`（`std_msgs/msg/String`）× 4
- 总大小约 **25KB**，远小于考核要求的 10MB 上限

## 关于 metadata.yaml 的来源（如实说明）

录制当时只生成了裸数据文件 `run1_0.db3` 和 `bag_test_0.db3`，**未生成 `metadata.yaml`**，因此直接执行 `ros2 bag info` 会报 `Could not find metadata`。

本目录的 `metadata.yaml` 是据 db3 内真实信息反推重建的：

- 话题名、消息类型、序列化格式、QoS profile：从 db3 的 `topics` 表原样读出
- 消息数（4 / 6）、时间戳（起始 1795333098531943796 ns、跨度 1371.83s）、duration：从 `messages` 表统计得出
- 消息定义（schema）：与仓库 `cquick_servo_interfaces/msg/ServoCommand.msg` 一致
- ROS distro `humble`、schema version 3：从 db3 的 `schema` 表读出

重建后已校验：metadata 中的消息数与话题计数与 db3 实际数据**完全一致**。`run1_0.db3` 的有效内容来自当时的 `bag_test_0.db3`（`run1_0.db3` 原始数据为空、0 条消息，故未采用）。

> 格式说明：`metadata.yaml` 严格按 ROS2 Humble 的 `rosbag2_storage` 要求组织——顶层为 `rosbag2_bagfile_information`，`duration` / `starting_time` 为 `nanoseconds` / `nanoseconds_since_epoch` 嵌套结构，话题列表键为 `topics_with_message_count`（每项含 `topic_metadata` 与 `message_count`）。早期扁平格式会被解析为 `invalid node; first invalid key: version`，已修正。

配套 B 站录屏（bag play 回放复现）：<https://www.bilibili.com/video/BV1exy5zoEGD/>
