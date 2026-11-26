# udev 固定设备名（加分项）

## 解决什么问题

CH340 插上后默认叫 `/dev/ttyUSB0`，这个编号随插拔、换口、重启而漂移（有时变 ttyUSB1）。`servo_params.yaml` 里写死一个编号，换口就得改参数重开 launch。udev 规则按芯片的 USB VID:PID 绑定一个固定名字，插哪个口都叫 `/dev/cquick_servo`，参数不用动。

## 规则文件

`99-cquick-ch340.rules`（同目录），内容一行：

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="cquick_servo", MODE="0666"
```

- `1a86:7523` = CH340 的 VID:PID（`lsusb` 可查）
- `SYMLINK+="cquick_servo"`：生成固定软链
- `MODE="0666"`：普通用户可读写，不用 sudo 跑 launch

## 安装（不用重启）

```bash
sudo cp 99-cquick-ch340.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

拔插一次 CH340，`ls -l /dev/cquick_servo` 应显示指向 `ttyUSBx` 的软链。

## 演示判据（30 秒）

```bash
lsusb | grep 1a86           # 认芯片：1a86:7523
ls -l /dev/cquick_servo     # 固定名在，-> ttyUSBx
ls /dev/ttyUSB*             # 对照：默认名带编号
```

换一个 USB 口再跑 `ls -l /dev/cquick_servo`：**固定名不变、ttyUSB 编号变了**，这就是加分点。

## 口播

"ttyUSB 编号随插拔漂移，launch 的 port 参数就得跟着改；udev 按 1a86:7523 绑固定名，插哪个口都是 /dev/cquick_servo，参数不用动。"

## 兜底

VMware USB 透传环境下软链偶尔不生成。此时不影响功能分：命令继续用 `port:=/dev/ttyUSB0` 覆盖，讲清原理并展示规则文件即可。

## 证据

- `udev设备名.png`：`cat` 规则文件 + `ls -l /dev/cquick_servo -> ttyUSB1` 同屏
