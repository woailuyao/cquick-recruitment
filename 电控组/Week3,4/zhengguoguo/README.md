cquick-recruitment/
└── 电控组/
    └── Week3,4/
        └── zhengguoguo/ 
            ├── 01_串口设备.png
            ├── 01.png
            ├── 01_电池电压.png  
            ├── 02_接口编译.png  
            ├── 02_接口编译2.png
            ├── 02_launch一键启动.png
            ├── 02_action_反馈.png
            ├── 02_取消生效.png
            ├── 02_取消生效2.png
            ├── 02_限位保护.png
            ├── 02_param_set现场.png
            ├── 02_bag_info.png
            ├── 02_参数自检.png
            ├── 02_取消生效.png
            ├── 02_取消生效2.png
            ├── 03_电池电压.png
            ├── 03_整机照片(正面）.png
            ├── 03_整机照片(侧面）.png
            ├── 03_接线特写.png
            ├── 03_ID扫描.png
            ├── 调试记录.md
            ├── README.md 
            ├── bag/
            │   └── run1/ MB
            └── src/
                ├── cquick_servo_interfaces/
                │   ├── msg/ServoCommand.msg
                │   ├── action/MoveServo.action
                │   ├── CMakeLists.txt
                │   └── package.xml
                └── cquick_servo/
                    ├── cquick_servo/
                    │   ├── servo_driver.py
                    │   ├── move_client.py
                    ├── launch/servo.launch.py
                    ├── config/servo_params.yaml
                    ├── resource/cquick_servo
                    ├── package.xml
                    └── setup.py
                     
                     
# CQuick 战队电控组 · 第 3、4 周考核作业

## 一、 架构图

```mermaid
graph TD
    A[输入] -->|发送 Goal| B(move_client 客户端)
    B -->|Action 请求| C{servo_driver 服务端}
    
    C -->|1. 校验参数| D[判断 pwm_min / pwm_max]
    D -->|超限| E[返回 Result: REJECTED]
    D -->|正常| F[拼接 ASCII 指令]
    
    F -->|串口 115200 8-N-1| G[USB 转 TTL 模块]
    G -->|半双工总线| H[24路控制板]
    H -->|PWM 信号| I((总线舵机))
    
    C -->|5Hz 持续发送| J[Feedback: 进度条]
    C -->|同步发送| K[Result: 成功/失败/canceled]
    
    C -.->|发布调试数据| L[话题 /servo/command]
    C -.->|订阅调试数据| L

## 二、 参数表
| 参数名                 | 默认值          | 为什么是这个默认值                       |
| `port`                 | `/dev/ttyUSB0` | 这是默认串口名，可通过 launch 参数动态覆盖 |
| `baudrate`             | `115200`       | 这是舵机标准通信波特率                    |
| `pwm_min`              | `900`          | 实测结构安全下限，防止舵机卡死            |
| `pwm_max`              | `2100`         | 实测结构安全上限，防止堵转发热烧毁舵机     |
| `default_duration_ms`  | `800`          | 默认转动平滑时间，防止瞬间跳变            |

## 三、 接口契约
- **Goal**：`id`(舵机名称), `pwm`(目标位置), `duration_ms`(用时)。
- **Result**：`success`(成功), `final_pwm`(最终位置), `message`(原因)。
- **Feedback**：`current_pwm`(当前值), `progress`(0.0~1.0 进度)。在 execute 循环中至少以 5Hz 频率发送。


## 四、 一条启动命令及输出
执行 `ros2 launch cquick_servo servo.launch.py`。
输出结果： `Opened serial port /dev/ttyUSB0 at 115200` 及 `Servo Driver initialized.`。

## 五、 安全区间是多少、怎么试出来的？
- **区间**：900 ~ 2100
- **来历**：通过手动发送指令，逐步进行测试，发现在低于 900 或高于 2100 时，舵机转动受阻并伴有“嗡嗡”表示堵转，因此将900 ~ 2100设为安全区间以防舵机烧毁。

## 六、 运行时改参数的演示记录
执行 `ros2 param set /servo_driver pwm_max 1600` 成功。随后发送目标为 `1700` 的 Goal，终端打印 `Goal was rejected`，说明修改参数后无需重启，当即生效。

## 七、 Bag 里录了什么、怎么回放、回放结果
- **内容**：录制了 `/servo/command` 话题，`Message count` 为 1（通过手动 Pub 补录成功）。
- **回放命令**：`ros2 bag play bag/run1`
- **结果**：动作成功复现。

## 八、 接线说明
- **通信线**：红线接控制板 RX，黄线接 TX，黑线接 GND。
- **电池实测电压**：7.8V（安全范围内）。
- **舵机接线**：插在控制板通道 2，白线对准 S（信号）针。

## 九、 动作 yaml 长什么样
本周未能完成，下周一定补上。

## 十、 踩坑记录
1.在建包时误将servo写成serve，虽然及时将文件名更改过来，但是后续进行编译时，依旧反复报错，最终在隐藏的setup文件中终于找到残留的导致报错的旧名字servo，更改过来后得以顺利进行编译。
2.在bag录制时，反复出现录制到的message为0的事情，尽管action的goal返回是success、舵机读回值也没有出现问题。我层层排查，排除了节点未跑起来、USB转TTL模块没有接入虚拟机等问题，最终发现原因在于话题上只有订阅者，没有发布者，于是通过修改有关发布者的底层代码而解决。
3.在进行任务二时反复遇到串口无法链接虚拟机的问题，起初以为是串口接触不良，用过反复拔插USB、强行抢夺串口权限等办法，最后通过寻找串口权限归属，才得以解决问题。

## 十一、 下电三步及原因
本周未能完成，下周一定补上。我根据理论知识得出以下结论
1. **先卸力**：发送 `#000PULK!`。原因：防止舵机断电前死顶结构发热烧毁。
2. **断动力电**：拔掉电池。原因：避免控制板在异常电压下发送乱码指令，导致舵机烧坏。
3. **关板卡**：执行 `sudo shutdown now`命令。原因：防止直接拔电损坏文件系统。

## 十二、 录屏链接（B站）
【任务二-哔哩哔哩】 https://b23.tv/2QL6YQ2
本周仅完成了任务二的视频录制，任务三和任务四的视频下周一定补上。
