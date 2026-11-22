# 第 3–4 周 · 整机拼装与舵机控制

> **两周合并考核，总分 100 分 + 加分 10 分。** 四关顺序不能换：点亮一个 → 让 ROS2 接管 → 复制到整机 → 安全下电。
> 完整任务要求、评分标准、FAQ、防抄袭机制 → 见 [`../考核细则_第3-4周.md`](../考核细则_第3-4周.md)

## 这一版比前两周硬性多了什么

| # | 要求 | 为什么加 |
|---|---|---|
| 1 | 控制接口用 **Action**（goal / feedback / result / cancel 四件套） | 真实战队代码就是这么写的，没有选择题 |
| 2 | **一条 `ros2 launch` 全起来** | 不许再「开两个终端各跑一个」 |
| 3 | 设备名、波特率、限位**全部参数化**，值放 yaml | 代码里不许有硬编码 |
| 4 | **运行时 `ros2 param set` 当帧生效** | 改限位不用改代码、重编译、重启 |
| 5 | **自定义 msg + action**，不用 `std_msgs` 糊 | 一条消息描述一件事 |
| 6 | **`ros2 bag` 当交付物**，必须能 `bag play` 复现动作 | 录屏只能看，bag 能重放 |
| 7 | 动作组**由 yaml 定义**，换动作不改 python | 动作是参数，不是逻辑 |

## 分值

| 任务 | 内容 | 分值 |
|---|---|---:|
| 任务 1 | 点亮一个舵机（桌面 · 1 个舵机 · 不装结构） | 15 |
| 任务 2 | 让 ROS2 接管（Action + 参数 + launch + bag） | **50** |
| 任务 3 | 装配 + 多舵机动作组（yaml 驱动） | 25 |
| 任务 4 | 安全下电（现场必考） | 10 |
| 加分 | udev / 闭环 / 看门狗 / 抢占 / 重连 / 单测，上限 +10 | +10 |

## 提交目录

在本目录下新建你的个人文件夹（**姓名拼音，全小写，不要空格**）：

```
电控组/Week3-4/
└── <你的姓名拼音>/
    ├── 01_串口设备.png
    ├── 01_PID回显.png
    ├── 01_中立位标定.png
    ├── 01_电池电压.png
    ├── 02_接口编译.png
    ├── 02_launch一键启动.png
    ├── 02_action_反馈.png
    ├── 02_取消生效.png
    ├── 02_限位保护.png
    ├── 02_param_set现场.png
    ├── 02_bag_info.png
    ├── 02_参数自检.png
    ├── 03_整机照片.png
    ├── 03_接线特写.png
    ├── 03_ID扫描.png
    ├── 03_多舵机动作_关键帧.png
    ├── 调试记录.md
    ├── README.md                 ← 里贴 B 站录屏链接
    ├── bag/
    │   └── run1/                 ← metadata.yaml + .db3，≤10 MB
    └── src/
        ├── cquick_servo_interfaces/
        │   ├── msg/ServoCommand.msg
        │   ├── action/MoveServo.action
        │   ├── CMakeLists.txt
        │   └── package.xml
        └── cquick_servo/
            ├── cquick_servo/          ← servo_driver.py / move_client.py / sequence_player.py
            ├── launch/servo.launch.py
            ├── config/
            │   ├── servo_params.yaml
            │   └── actions/          ← stand_up.yaml / wave.yaml（至少两套）
            ├── resource/cquick_servo
            ├── package.xml
            └── setup.py
```

## 截图清单

| 文件名 | 内容 | 对应任务 |
|---|---|---|
| `01_串口设备.png` | `ls -l /dev/ttyUSB*`（属主组是 dialout） | 任务 1 |
| `01_PID回显.png` | `#001PID!` 有回显 | 任务 1 |
| `01_中立位标定.png` | 每个舵机发到 1500 后的状态 | 任务 1 |
| `01_电池电压.png` | 万用表读数（5–8.4V） | 任务 1 |
| `02_接口编译.png` | `ros2 interface show` 输出（msg + action） | 任务 2 |
| `02_launch一键启动.png` | 一条 `ros2 launch` + 起来后的节点列表（同窗口） | 任务 2 |
| `02_action_反馈.png` | `send_goal --feedback` 的 feedback 滚动 + result | 任务 2 |
| `02_取消生效.png` | 取消后 result 里的 `canceled` | 任务 2 |
| `02_限位保护.png` | 越界 goal 被 rejected | 任务 2 |
| `02_param_set现场.png` | 改参数**不重启**就生效 / 被拒（2.6 六步） | 任务 2 |
| `02_bag_info.png` | `ros2 bag info` 输出 | 任务 2 |
| `02_参数自检.png` | `grep` 白名单自检输出 | 任务 2 |
| `03_整机照片.png` | 正面 + 侧面 | 任务 3 |
| `03_接线特写.png` | 走线、扎带、接口 | 任务 3 |
| `03_ID扫描.png` | 全部舵机在线 | 任务 3 |
| `03_多舵机动作_关键帧.png` | 动作组关键帧 | 任务 3 |

> 每张图都要带「姓名拼音 + 时间戳」。截图前先执行：
> ```bash
> echo "姓名: <你的姓名拼音>"; date "+%Y-%m-%d %H:%M:%S"
> ```

## 录屏与 bag

- **所有录屏发 B 站，README 里贴链接**，仓库里不放视频：任务 2 的正反转 / 取消打断 / 改参数生效 / bag 回放，任务 3 的整机与动作组，任务 4 的下电流程；
- **bag 必须进仓库**（`bag/run1/`，≤10 MB，≤30 秒，只录 `/servo/command` 和 `/servo/state`，**禁止 `-a`**）。

## 提交流程

1. Fork 本仓库 → 2. Clone 你的 Fork → 3. `cd 电控组/Week3-4/` 建 `<姓名拼音>/` → 4. 按上面结构放文件 → 5. `git add` / `commit` / `push` → 6. 发起 Pull Request（模板会自动带出自查清单）。

> 提交前自查：两个包的源码是否完整（`package.xml`、`CMakeLists.txt`、`setup.py` 最容易漏）；`launch/` 和 `config/` 有没有在 `setup.py` 的 `data_files` 里装进 share；有没有把 `build/` `install/` `log/` 一起传上来（**不要传**，已在 `.gitignore`）。

---

**CQuick 战队 · 电控组**
