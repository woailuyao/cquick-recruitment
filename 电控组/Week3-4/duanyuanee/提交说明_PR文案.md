# 提交文案（PR / Commit / 网页步骤）—— 照抄即可

> 本文件是给你提交用的文案，GitHub 提交完成后可以删掉，也可以保留。
> 考核仓库：`https://github.com/woailuyao/cquick-recruitment`

---

## 一、Commit message（粘到 commit 输入框）

```
提交电控组第3、4周任务 - duanyuanee
```

---

## 二、PR 标题

```
电控组第3-4周 duanyuanee：整机拼装与舵机控制
```

---

## 三、PR 描述（整段复制到 PR 正文框）

```markdown
## 描述

电控组第 3、4 周考核 · 段媛（duanyuanee）。按四关顺序完成：任务 1 点亮单个舵机 → 任务 2 让 ROS2 接管（Action）→ 任务 3 复制到整机 + 数据驱动动作组 → 任务 4 安全下电。所有关键截图带「姓名拼音 + 实时时间戳」，代码、bag、README、调试记录均已提交。

## 变更内容

新增 `电控组/Week3,4/duanyuanee/`，包含：

**代码（任务 2、3）**
- `src/cquick_servo_interfaces/`：自定义 `msg/ServoCommand.msg`（id/pwm/duration_ms）与 `action/MoveServo.action`（Goal/Result/Feedback 三段），ament_cmake 包，含 rosidl 依赖与 `member_of_group` 声明
- `src/cquick_servo/`：
  - `servo_driver.py`：Action Server，四个回调齐全（goal/cancel/execute + 参数校验回调）；`MultiThreadedExecutor` + `ReentrantCallbackGroup` 保证取消即时生效；动作完成读 PRAD 做 ±30 到位校验；取消时 PULR + 回中位；订阅 `/servo/command` 供 bag 回放，与 Action 共用 `send_pwm`
  - `move_client.py`：Action Client，打印 feedback/result
  - `sequence_player.py`：动作组播放器，读 yaml → 展开 goal → 用 result 当同步信号，日志带 result 时间戳
  - `launch/servo.launch.py`：一条 launch 全启动，参数从包内 yaml 加载，`port` 可覆盖，`demo:=true` 条件启动客户端
  - `config/servo_params.yaml`：七个考核参数 + safe_pwm 全部外置
  - `config/actions/stand_up.yaml`、`wave.yaml`：两套数据驱动动作

**数据 / 文档**
- `bag/run1/`：`ros2 bag info` 可解析的完整 bag（10 条消息，`/servo/command` ×6 + `/servo/state` ×4，约 25KB），`ros2 bag play` 可复现动作
- `screenshots/`：规范命名截图，含 `02_bag_info.png`（姓名 + 时间戳 + bag info 输出同框）、`03_整机照片.png`、`01_电池电压.png`、`03_接线特写.png`、`03_多舵机动作_关键帧.png`
- `README.md`：考核要求的 12 项全部覆盖（架构图、参数表、接口契约、启动命令、安全区间来历、改参数演示、bag 说明、接线、动作 yaml、踩坑、下电、B 站链接）
- `调试记录.md`：真实踩坑记录（现象 → 怀疑 → 验证 → 结论）
- `下电流程.md`：三步下电及原因

**B 站录屏（3 段已投稿，链接在 README 第 12 节）**
- 任务 2 正反转回中位：https://www.bilibili.com/video/BV19xy5zoEoJ/
- 任务 2 bag 回放复现：https://www.bilibili.com/video/BV1exy5zoEGD/
- 任务 3 整机动作演示：https://www.bilibili.com/video/BV1exy5zoEuE/

## 自查清单

- [x] 个人文件夹命名全小写拼音：`duanyuanee`
- [x] 截图按「序号_内容」规范命名，关键图带姓名拼音 + 实时时间戳
- [x] 提交完整多包工程：接口包（ament_cmake）+ 节点包（ament_python）
- [x] 未提交 `build/`、`install/`、`log/`（`.gitignore` 已排除）
- [x] 视频未进仓库，只放截图和 bag；B 站链接已贴 README
- [x] 七个参数全部外置，驱动代码无配置硬编码（协议常量除外）
- [x] Action goal/feedback/result/cancel 全可用，含 PRAD ±30 到位校验
- [x] 动作组由 yaml 定义，两套动作，播放器用 result 同步
- [x] bag ≤10MB，仅录两个话题，`ros2 bag info/play` 已验证

## 如实说明（硬件已归还，无法补录/补拍）

- 任务 2 的「取消打断」「改参数生效」两段 B 站录屏、任务 3「动作组连跑三遍 + 换 yaml」录屏、任务 4「下电流程」录屏：考核结束后实验硬件已归还，无法再补录实机视频；对应命令、参数、动作 yaml 与代码均已写入仓库
- `03_ID扫描.png`（需逐舵机扫 ID）、`02_取消生效.png` 的成功取消截图：同样因硬件归还无法补拍；读回校验与取消回中位的实现已在 `servo_driver.py` 中
- 物料登记表：以 `补充证据/补充_物料清单照片.png`（实物平铺照）代替表格形式
```

---

## 四、提交步骤（你已 Fork → 在你自己的仓库里提交 → 发 PR）

> 你已经从原仓库 Fork 过了，**不用再 Fork**。按仓库 README 和考核第九节的提示，最终是把代码 commit + push 到**你自己 Fork 的仓库**，再从那里发 PR 回原仓库。下面两条路二选一，结果完全一样。

### 方式 A · 网页端上传（不用配 git 凭据，最省事，推荐）

1. 打开 GitHub 并登录 → 点右上角**你的头像** → **Your repositories** → 点进 `cquick-recruitment`（这就是你 Fork 的那个，不是 `woailuyao` 原仓库）。
2. 依次进入 `电控组/` → `Week3,4/` 目录。
3. 点 `Add file` → `Upload files`，把本地整个 `duanyuanee` 文件夹拖进去（GitHub 网页支持拖文件夹，会保留子目录结构）。
   - 确认上传后的路径是 `电控组/Week3,4/duanyuanee/...`，**不要**多套一层。
4. 页面底部 Commit 框：
   - Commit title 填上面「一、Commit message」那行；
   - 点 `Commit changes`（网页端 commit 等价于 README 里的 commit + push）。
5. 回到你 Fork 仓库主页，点 **`Pull requests` → `New pull request`**：
   - **base**：`woailuyao/cquick-recruitment` 的 `main`（原仓库）
   - **compare**：下拉框选**你 Fork 的那个** `cquick-recruitment` 的 `main`（不用手打地址，下拉框里直接点）
   - 标题、正文分别粘贴上面「二」「三」的内容，点 `Create pull request`。
6. 提交完成，把 PR 链接发给评审。

### 方式 B · 命令行（完全照 README 的 Fork → clone → commit → push → PR）

```bash
# 1. 拿你 Fork 仓库的地址：在你 Fork 的仓库网页点绿色 «Code» 按钮 → HTTPS 那行右边有复制图标，点一下就复制好了

# 2. 在你电脑的终端里，先 cd 到你想放代码的地方，然后粘贴刚才复制的地址（直接 Ctrl+V，不用手打）：
git clone 这里粘贴你复制的地址

# 3. 把已整理好的 duanyuanee 文件夹整个拷进 clone 出来的目录里
#    目标路径：cquick-recruitment/电控组/Week3,4/duanyuanee/（Windows 用资源管理器拖进去即可）

# 4. 提交并推送
cd cquick-recruitment
git add "电控组/Week3,4/duanyuanee"
git commit -m "提交电控组第3、4周任务 - duanyuanee"
git push origin main

# 5. 打开你 Fork 的仓库网页 → Pull requests → New pull request
#    base = woailuyao/cquick-recruitment 的 main，compare = 你 Fork 的 main（都是下拉框选，不用手打）
#    粘贴上面「二」「三」的标题和正文 → Create pull request
```

> 注意：
> - **全程不用手打用户名或地址**：网页端进自己的仓库靠头像菜单；命令行靠 «Code» 按钮复制；PR 的 base/compare 靠下拉框选。
> - 不要把 `build/`、`install/`、`log/` 或 `.mp4` 视频传上去（本目录已清理，`screenshots/补充证据/` 里都是图片）。
> - 若上传时发现多了 `__pycache__` 或 `.pyc`，删掉再传（本目录已清理为 0）。

---

## 五、提交前最后一次本地核对清单

- [ ] `duanyuanee/` 目录下没有 `build/`、`install/`、`log/`、`*.mp4`、`__pycache__/`、`*.pyc`
- [ ] `screenshots/` 下有 14 张规范名截图，`02_bag_info.png` 是干净的实测图
- [ ] `bag/run1/` 有 `metadata.yaml` + `run1_0.db3`
- [ ] README 第 12 节 3 条 B 站链接可点开
- [ ] PR 描述里缺失项已如实说明（不要删那段，诚实说明不扣分，藏着才危险）
