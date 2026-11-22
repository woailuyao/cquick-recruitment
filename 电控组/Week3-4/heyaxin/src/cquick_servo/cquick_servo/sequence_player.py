"""
sequence_player.py — 动作组播放器（任务 3）

一套动作 = 一个 yaml，本节点只负责：读 yaml → 展开成 goal 序列 → 按顺序打出去。

三条铁律（对应考核 3.4）：
1. 同一个 step 里的关节**同时下发**（并行）；
2. step 与 step 之间**串行**：上一步所有 goal 的 result 都回来了才发下一步；
3. 要"某个关节先动"，就给它单独开一个 step —— **绝不靠调 duration_ms 硬凑先后**。

同步只用 result，不用 sleep：
    舵机"按时间平滑转到位"的实际耗时 ≠ duration_ms，用 sleep 硬等会累积误差，
    跑三遍必然对不上。所以这里等的是 result，不是时间。

用法：
    ros2 run cquick_servo sequence_player --list                 # 列出可用动作
    ros2 run cquick_servo sequence_player --action stand_up      # 播 config/actions/stand_up.yaml
    ros2 run cquick_servo sequence_player --action wave --repeat 3
    ros2 run cquick_servo sequence_player --file ~/my_action.yaml --dry-run

    --dry-run   只展开、不下发，用来核对 yaml 写对没写对
    --repeat N  连跑 N 遍，跑完自动比对每遍的 final_pwm 是否一致
    --verbose   打印每个关节的 feedback（默认只在最后一条打）
"""

import os
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand

CONSISTENCY_TOLERANCE_PWM = 15   # PRAD 有 ±几个单位抖动，判“一致”给容差

# goal 里 duration / pwm 的合法性由服务端最终判定，这里先做一次同样的检查，
# 好处是 yaml 写错时在"下发之前"就报错，而不是播到一半被拒。
PWM_HARD_MIN = 500
PWM_HARD_MAX = 2500
DURATION_LIMIT_MS = 60000

# 每一步的兜底超时 = 该步最长 duration + 这个余量。
# 它**不是**用来同步的（同步靠 result），只是防止某个 result 永远不回来把程序挂死。
STEP_TIMEOUT_MARGIN_S = 5.0
# 单条 feedback 打印间隔（verbose 模式下按关节限流，避免刷屏）
FEEDBACK_PRINT_INTERVAL_S = 0.5


class YamlError(Exception):
    """yaml 结构/取值不合法，属于"数据"错误，直接退出，不要带病下发。"""


def _candidate_dirs():
    """动作 yaml 可能待的地方，按优先级排。

    为什么要找这么多个：`--symlink-install` 下 yaml 的链接链是
    install/share → build/config/actions → src/config/actions，
    而 glob 在编译时就固定了，**现场新建的 yaml 不会自动多出链接**。
    所以这里把 install、build（经 realpath 拿到的）、src 都列上，
    换动作就不用重新 colcon build —— 这条在"现场给一套没见过的动作"时很关键。
    """
    dirs = []
    try:
        share = get_package_share_directory('cquick_servo')
        dirs.append(os.path.join(share, 'config', 'actions'))
    except Exception:
        pass

    base = os.path.dirname(os.path.realpath(__file__))
    for _ in range(3):
        dirs.append(os.path.normpath(os.path.join(base, '..', 'config', 'actions')))
        parent = os.path.dirname(base)
        if parent == base:
            break
        base = parent

    # build/ 下的同名目录，映射回 src/ 再试一次
    out, seen = [], set()
    for d in dirs:
        for cand in (d, d.replace(os.sep + 'build' + os.sep,
                                  os.sep + 'src' + os.sep, 1)):
            if cand not in seen:
                seen.add(cand)
                out.append(cand)
    return out


def _joints_dir():
    """列目录用：第一个真实存在的目录。"""
    for d in _candidate_dirs():
        if os.path.isdir(d):
            return d
    return _candidate_dirs()[0]


def _all_dirs():
    return [d for d in _candidate_dirs() if os.path.isdir(d)]


def resolve_action_file(action_name):
    """先找 install，再找 build/src；都没有就把 install 那个路径返回去报错。"""
    for d in _candidate_dirs():
        p = os.path.join(d, action_name + '.yaml')
        if os.path.isfile(p):
            return p
    return os.path.join(_joints_dir(), action_name + '.yaml')


def load_sequence(path):
    """读 yaml 并做结构校验，返回 (name, steps)。

    steps = [{'name': str, 'joints': [{'id','pwm','duration_ms'}]}]
    """
    if not os.path.isfile(path):
        raise YamlError(f'动作文件不存在: {path}')
    with open(path, 'r', encoding='utf-8') as f:
        try:
            doc = yaml.safe_load(f)
        except yaml.YAMLError as exc:
            raise YamlError(f'yaml 语法错误: {exc}')

    if not isinstance(doc, dict):
        raise YamlError('yaml 顶层必须是键值对（name / defaults / steps）')

    name = doc.get('name') or os.path.splitext(os.path.basename(path))[0]
    defaults = doc.get('defaults') or {}
    default_duration = defaults.get('duration_ms', 1500)

    raw_steps = doc.get('steps')
    if not isinstance(raw_steps, list) or not raw_steps:
        raise YamlError('steps 必须是非空列表')

    steps = []
    for i, raw in enumerate(raw_steps, start=1):
        if not isinstance(raw, dict) or 'joints' not in raw:
            raise YamlError(f'第 {i} 个 step 缺 joints 字段')
        raw_joints = raw['joints']
        if not isinstance(raw_joints, list) or not raw_joints:
            raise YamlError(f'第 {i} 个 step 的 joints 是空的')
        step_name = raw.get('name') or f'step{i}'

        joints = []
        seen_ids = set()
        for j, item in enumerate(raw_joints, start=1):
            if not isinstance(item, dict):
                raise YamlError(f'第 {i} 步第 {j} 个关节必须是键值对')
            unknown = set(item) - {'id', 'pwm', 'duration_ms'}
            if unknown:
                raise YamlError(
                    f'第 {i} 步第 {j} 个关节有未知字段 {sorted(unknown)}'
                    f'（只认 id / pwm / duration_ms）')
            servo_id = item.get('id')
            pwm = item.get('pwm')
            duration = item.get('duration_ms', default_duration)

            if not isinstance(servo_id, int):
                raise YamlError(f'第 {i} 步第 {j} 个关节的 id 必须是整数，收到 {servo_id!r}')
            if servo_id in seen_ids:
                # 同一步里同一个舵机两条指令，服务端只会收到两条互相打断的 goal
                raise YamlError(
                    f'第 {i} 步里舵机 {servo_id} 出现了两次；'
                    f'要让它先后动两次，请拆成两个 step')
            seen_ids.add(servo_id)

            if not isinstance(pwm, int) or not (PWM_HARD_MIN <= pwm <= PWM_HARD_MAX):
                raise YamlError(
                    f'第 {i} 步舵机 {servo_id} 的 pwm={pwm!r} 超出 {PWM_HARD_MIN}~{PWM_HARD_MAX}')
            if not isinstance(duration, int) or not (0 < duration <= DURATION_LIMIT_MS):
                raise YamlError(
                    f'第 {i} 步舵机 {servo_id} 的 duration_ms={duration!r} 不合法'
                    f'（1~{DURATION_LIMIT_MS}）')

            joints.append({'id': servo_id, 'pwm': pwm, 'duration_ms': duration})
        steps.append({'name': step_name, 'joints': joints})

    return name, steps


class SequencePlayer(Node):
    def __init__(self):
        super().__init__('sequence_player')
        self._client = ActionClient(self, MoveServo, 'move_servo')
        self._verbose = False
        self._active_handles = []
        self._last_feedback_print = {}

    # ---------- 基础 ----------

    def wait_for_server(self, timeout=10.0):
        return self._client.wait_for_server(timeout_sec=timeout)

    def _spin_until(self, predicate, timeout):
        """空转，直到 predicate() 为真或超时。返回是否满足条件。"""
        deadline = time.monotonic() + timeout
        while rclpy.ok():
            if predicate():
                return True
            if time.monotonic() > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.02)
        return predicate()

    def _stamp(self):
        return time.strftime('%H:%M:%S', time.localtime()) + \
            f'.{int(time.time() * 1000) % 1000:03d}'

    def _on_feedback(self, feedback_msg):
        if not self._verbose:
            # 非 verbose 只在 progress 到 1.0 时打一条，证明"过程有反馈"
            if feedback_msg.feedback.progress < 0.999:
                return
        gid = bytes(feedback_msg.goal_id.uuid).hex()[:8]
        now = time.monotonic()
        if now - self._last_feedback_print.get(gid, 0.0) < FEEDBACK_PRINT_INTERVAL_S:
            return
        self._last_feedback_print[gid] = now
        self.get_logger().info(
            f'  feedback {gid} pwm={feedback_msg.feedback.current_pwm} '
            f'progress={feedback_msg.feedback.progress:.2f}')

    # ---------- 单步执行 ----------

    def run_step(self, index, total, step):
        """下发一个 step 的所有 goal，等到全部 result 回来。返回 [(id, ok, final, msg)]。"""
        joints = step['joints']
        plan = ' '.join(f'id={j["id"]}→{j["pwm"]}({j["duration_ms"]}ms)' for j in joints)
        self.get_logger().info(
            f'[step {index}/{total}] {step["name"]}：同时下发 {len(joints)} 个 goal  {plan}')

        t0 = time.monotonic()
        send_futures = []
        stagger = getattr(self, '_stagger_ms', 0) / 1000.0
        for idx_j, j in enumerate(joints):
            goal = MoveServo.Goal()
            goal.command = ServoCommand(id=j['id'], pwm=j['pwm'],
                                        duration_ms=j['duration_ms'])
            send_futures.append(self._client.send_goal_async(
                goal, feedback_callback=self._on_feedback))
            # 错峰下发：避免多台舵机在同一毫秒同时启动造成电流尖峰
            if idx_j < len(joints) - 1 and stagger > 0:
                time.sleep(stagger)

        # ① 等所有 goal 被服务端受理/拒绝
        for f in send_futures:
            self._spin_until(f.done, timeout=10.0)

        outcomes = []
        result_futures = []
        for j, f in zip(joints, send_futures):
            if not f.done() or f.result() is None:
                outcomes.append((j['id'], False, None, '发送超时（服务端无响应）'))
                continue
            handle = f.result()
            if not handle.accepted:
                outcomes.append((j['id'], False, None,
                                 'goal 被拒（pwm 超限位 / id 不在 servo_ids / duration 非法）'))
                continue
            self._active_handles.append(handle)
            result_futures.append((j['id'], handle.get_result_async()))

        # ② 等所有 result —— 这里就是"用 result 当同步信号"
        step_timeout = max(j['duration_ms'] for j in joints) / 1000.0 + STEP_TIMEOUT_MARGIN_S
        for servo_id, rf in result_futures:
            self._spin_until(rf.done, timeout=step_timeout)
            elapsed = time.monotonic() - t0
            if not rf.done():
                outcomes.append((servo_id, False, None,
                                 f'等 result 超时（>{step_timeout:.1f}s，动作未结束）'))
                self.get_logger().warn(
                    f'  ✗ id={servo_id} result 超时，{step_timeout:.1f}s 内没回来')
                continue
            wrapped = rf.result()
            result = wrapped.result
            status = wrapped.status
            ok = bool(result.success) and status == 4  # 4 = STATUS_SUCCEEDED
            outcomes.append((servo_id, ok, result.final_pwm, result.message))
            self.get_logger().info(
                f'  ↳ result id={servo_id} success={result.success} '
                f'final_pwm={result.final_pwm} 用时={elapsed:.2f}s '
                f'[{self._stamp()}] — {result.message}')

        # 已结束的 handle 不再需要取消：留着的只应是在飞（ACCEPTED=1 / EXECUTING=2）的
        self._active_handles = [
            h for h in self._active_handles
            if getattr(h, 'status', 4) in (1, 2)]
        self.get_logger().info(
            f'[step {index}/{total}] 全部 result 已回（{time.monotonic() - t0:.2f}s），'
            f'允许进入下一步')
        return outcomes

    # ---------- 整套 ----------

    def play(self, name, steps, repeat=1, keep_going=False):
        self._keep_going = keep_going
        """跑完整套。返回 (每遍的记录, 是否全部成功)。"""
        runs = []
        all_ok = True
        for r in range(1, repeat + 1):
            if repeat > 1:
                self.get_logger().info(f'========== 第 {r}/{repeat} 遍 ==========')
            records = []
            ok_run = True
            for i, step in enumerate(steps, start=1):
                outcomes = self.run_step(i, len(steps), step)
                records.append((step['name'], outcomes))
                failed = [o for o in outcomes if not o[1]]
                if failed:
                    ok_run = False
                    all_ok = False
                    for sid, _, _, msg in failed:
                        self.get_logger().error(
                            f'  第 {i} 步 舵机 {sid} 失败：{msg}')
                    # 默认一步没走干净就停下：继续发后面的 goal 等于让结构
                    # 在错位下被推着走。做逐关节自检时用 --keep-going，
                    # 一次看完所有关节再统一处理，不用来回跑 30 遍。
                    if self._keep_going:
                        self.get_logger().warn(
                            f'  第 {i} 步有失败，已开 --keep-going，继续下一步')
                    else:
                        self.get_logger().error(
                            '动作组中止（该步有 goal 失败/超时）。排查后重跑，'
                            '或用 --action 换一套动作；先用 --dry-run 核对 yaml。')
                        return runs + [records], all_ok
            runs.append(records)
            self.get_logger().info(f'第 {r} 遍跑完：{len(steps)} 步全部有 result')
        return runs, all_ok

    def cancel_all(self):
        for handle in self._active_handles:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass
        self._active_handles.clear()


def summarize_repeats(logger, runs):
    """比对多遍的 final_pwm 是否一致 —— 这是'可重复'的硬证据。"""
    if len(runs) < 2:
        return True
    def flat(records):
        return [(step_name, sid, final)
                for step_name, outs in records for sid, _, final, _ in outs]
    base = flat(runs[0])
    same = True
    max_gap = 0
    tol = CONSISTENCY_TOLERANCE_PWM
    for idx, records in enumerate(runs[1:], start=2):
        cur = flat(records)
        diffs = []
        for b, c in zip(base, cur):
            if b[0] != c[0] or b[1] != c[1]:
                diffs.append((b, c))
                continue
            bf, cf = b[2], c[2]
            if bf is None or cf is None:
                if bf != cf:
                    diffs.append((b, c))
                continue
            gap = abs(int(bf) - int(cf))
            max_gap = max(max_gap, gap)
            if gap > tol:
                diffs.append((b, c, f'差 {gap}'))
        if not diffs:
            logger.info(f'一致性检查：第 {idx} 遍与第 1 遍一致'
                        f'（{len(base)} 条，偏差均 ≤ {tol} 个 PWM 单位）')
        else:
            same = False
            logger.error(f'一致性检查：第 {idx} 遍与第 1 遍有 {len(diffs)} 处超差'
                         f'（阈值 ±{tol}）→ {diffs[:5]}')
    if same:
        logger.info(f'一致性检查通过：三遍最大偏差 {max_gap} 个 PWM 单位（阈值 ±{tol}）')
    else:
        logger.error(f'一致性检查未通过：三遍最大偏差 {max_gap} 个 PWM 单位（阈值 ±{tol}）')
    return same


def print_dry_run(logger, name, steps):
    total = sum(len(s['joints']) for s in steps)
    logger.info(f'动作名：{name}　共 {len(steps)} 步 / {total} 个 goal（--dry-run，未下发）')
    for i, step in enumerate(steps, start=1):
        plan = '　'.join(f'舵机{j["id"]}→{j["pwm"]}（{j["duration_ms"]}ms）'
                        for j in step['joints'])
        logger.info(f'  第{i}步 {step["name"]}｜{len(step["joints"])} 个关节同时动：{plan}')


def main(args=None):
    rclpy.init(args=args)
    node = SequencePlayer()

    argv = remove_ros_args(args=sys.argv)[1:]
    action_name = None
    yaml_path = None
    repeat = 1
    dry_run = False
    do_list = False
    keep_going = False
    stagger_ms = 0
    i = 0
    while i < len(argv):
        a = argv[i]
        if a in ('--action', '-a') and i + 1 < len(argv):
            action_name = argv[i + 1]; i += 2
        elif a == '--file' and i + 1 < len(argv):
            yaml_path = argv[i + 1]; i += 2
        elif a == '--repeat' and i + 1 < len(argv):
            repeat = int(argv[i + 1]); i += 2
        elif a == '--stagger-ms' and i + 1 < len(argv):
            stagger_ms = int(argv[i + 1]); i += 2
        elif a == '--dry-run':
            dry_run = True; i += 1
        elif a == '--keep-going':
            keep_going = True; i += 1
        elif a == '--verbose':
            node._verbose = True; i += 1
        elif a == '--list':
            do_list = True; i += 1
        else:
            node.get_logger().error(f'看不懂的参数：{a}')
            node.get_logger().info(
                '用法：--action <名字> | --file <路径> [--repeat N] [--dry-run] [--verbose] [--list]')
            node.destroy_node(); rclpy.shutdown(); sys.exit(2)

    node._stagger_ms = stagger_ms

    if do_list:
        # install 和源码树都看一遍：现场新建的 yaml 还没同步进 install 也能列出来
        files = {}
        for d in _all_dirs():
            if not os.path.isdir(d):
                continue
            node.get_logger().info(f'动作文件目录：{d}')
            for f in sorted(os.listdir(d)):
                if f.endswith('.yaml'):
                    files.setdefault(f, os.path.join(d, f))
        if not files:
            node.get_logger().warn('没有 yaml；检查 config/actions/ 是否存在')
        for f, path in sorted(files.items()):
            try:
                nm, steps = load_sequence(path)
                total = sum(len(s['joints']) for s in steps)
                node.get_logger().info(f'  {f:<20} {nm:<14} {len(steps)} 步 / {total} goal')
            except YamlError as exc:
                node.get_logger().error(f'  {f:<20} 读取失败：{exc}')
        node.destroy_node(); rclpy.shutdown(); return

    if action_name and not yaml_path:
        yaml_path = resolve_action_file(action_name)
    if not yaml_path:
        node.get_logger().error('必须给 --action <名字> 或 --file <路径>（--list 可看有哪些）')
        node.destroy_node(); rclpy.shutdown(); sys.exit(2)

    try:
        name, steps = load_sequence(yaml_path)
    except YamlError as exc:
        node.get_logger().error(f'yaml 有问题：{exc}')
        node.get_logger().error(f'文件：{yaml_path}')
        node.destroy_node(); rclpy.shutdown(); sys.exit(2)

    node.get_logger().info(f'动作：{name}　{len(steps)} 步　文件：{yaml_path}')

    if dry_run:
        print_dry_run(node.get_logger(), name, steps)
        node.destroy_node(); rclpy.shutdown(); return

    if not node.wait_for_server(timeout=10.0):
        node.get_logger().error('等不到 /move_servo 服务端 —— 先起 servo_driver')
        node.destroy_node(); rclpy.shutdown(); sys.exit(1)

    exit_code = 0
    try:
        runs, ok = node.play(name, steps, repeat=repeat, keep_going=keep_going)
        if not ok:
            exit_code = 1
        if repeat > 1 and runs:
            summarize_repeats(node.get_logger(), runs)
    except KeyboardInterrupt:
        # Ctrl-C：把在飞的 goal 全部取消，服务端会回中位再返回 canceled
        node.get_logger().warn('收到 Ctrl-C，取消在飞的 goal（服务端会把舵机回中位）')
        node.cancel_all()
        exit_code = 130
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
