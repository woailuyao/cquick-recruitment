"""动作组播放器 —— 读 yaml -> 展开成 goal 序列 -> 用 result 同步.

它能干嘛：
  读一个 yaml 动作文件 → 一条条发给驱动节点 → 每条都等 result 回来才发下一条
  → 支持整套重复 N 遍、支持"保持姿势 N 毫秒"。

整个文件分三块，从上往下看：
  【区域 1】导入 + 工具函数 —— 不用动
  【区域 2】★ 配置区 —— 想改默认动作、目录、名字，改这里
  【区域 3】核心逻辑 + 主程序 —— 不用动

★ 注意：真正"调动作"是在 yaml 里改，不是改这个 py。
   这个 py 只负责"读 yaml 并一条条执行"，写一次就不用动了。
"""

# =====================================================================
# 【区域 1】导入 + 工具函数 —— 不用动
# =====================================================================
import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo


# ####################################################################
# #                                                                    #
# #   ★★★【区域 2】配置区 —— 这几个名字对不上就改这里 ★★★              #
# #                                                                    #
# ####################################################################
# 一般这些都不用动。什么时候要动？
#   - 换了包名 / 换了 yaml 文件夹名 / 换了 Action 名字 → 改对应行
#   - 想让"不写 -p 参数时"默认播别的动作 → 改 DEFAULT_MOTION
# 改等号右边的值就行，别动左边的名字。
# ---------------------------------------------------------------------

PACKAGE_NAME = "cquick_servo"       # ★1 你的包名
ACTIONS_DIR = "actions"             # ★2 yaml 放在 config/ 下的哪个子文件夹
ACTION_NAME = "move_servo"          # ★3 Action 名字，要和驱动节点里的一致
DEFAULT_MOTION = "walk.yaml"    # ★4 默认播哪个动作（命令行 -p 能覆盖它）

# ####################################################################
# #                        配置区到此为止                              #
# ####################################################################


# =====================================================================
# 【区域 3】核心逻辑 + 主程序 —— 不用动（看懂就行）
# ---------------------------------------------------------------------
# 三层结构：play 管"重复几遍"，run_step 管"一步怎么发"，load 管"读 yaml"。
# =====================================================================
class SequencePlayer(Node):
    def __init__(self):
        super().__init__('sequence_player')
        # 从命令行读 motion_file；没给就用配置区的 DEFAULT_MOTION
        self.declare_parameter('motion_file', DEFAULT_MOTION)
        self._client = ActionClient(self, MoveServo, ACTION_NAME)

    # ---- 读 yaml：就是这个函数把 yaml 引进来的 ----
    def load(self, filename):
        share = get_package_share_directory(PACKAGE_NAME)          # 找到包的安装目录
        path = os.path.join(share, 'config', ACTIONS_DIR, filename)  # 拼出 yaml 完整路径
        with open(path, 'r', encoding='utf-8') as f:               # 打开这个文件
            data = yaml.safe_load(f)                                # ★ 这一行真正把 yaml 读成数据
        self.get_logger().info(
            f"加载动作组 [{data['motion_name']}]：{len(data['steps'])} 步，文件 {filename}")
        return data

    # ---- 执行一步：有 hold_ms 就只等待；否则发 moves 并逐个等 result ----
    def run_step(self, index, step):
        name = step['name']

        # 保持步：不发指令，只停住 hold_ms 毫秒（对应"等待1秒"）
        if 'hold_ms' in step:
            hold = int(step['hold_ms'])
            self.get_logger().info(f"step{index} [{name}] 保持姿势 {hold}ms")
            time.sleep(hold / 1000.0)
            return True

        # 普通步：一步里的 moves 全部【并发下发】—— 循环里连续 send，中间不等待、不 sleep，
        # 让所有舵机在同一瞬间开始动（这是"协同动作"的关键，不能一个动完再发下一个）。
        # 每个 goal 都记下它的舵机 ID，日志里显示"几号舵机"，而不是 goal0/goal1。
        ids = [int(m['id']) for m in step['moves']]
        self.get_logger().info(
            f"step{index} [{name}] ★ 同时下发 {len(ids)} 个舵机：{ids}")

        pending = []
        for m in step['moves']:
            servo_id = int(m['id'])
            goal = MoveServo.Goal()
            goal.command.id = servo_id
            goal.command.pwm = int(m['pwm'])
            goal.command.duration_ms = int(step['duration_ms'])
            # 连发不等待：所有 goal 一起飞向驱动节点
            pending.append((servo_id, self._client.send_goal_async(goal)))

        # 关键：每一个 future 都要 spin 到完成，不能只 spin 第一个
        # （只 spin 第一个，其余没完成 → result() 是 None → 下一行就崩）
        for _, f in pending:
            rclpy.spin_until_future_complete(self, f)
        # 把 (舵机ID, goal句柄) 配对保留，丢掉没发出去的 None
        goals = [(sid, f.result()) for sid, f in pending if f.result() is not None]
        if not goals or not all(g.accepted for _, g in goals):
            self.get_logger().error(f"step{index} [{name}] 有 goal 被拒，停止")
            return False

        # 逐个拿 result，打印【真实舵机ID】+ 时间戳 —— 验收要求日志里能看到 result 时序
        for servo_id, g in goals:
            rf = g.get_result_async()
            rclpy.spin_until_future_complete(self, rf)
            result = rf.result().result
            stamp = self.get_clock().now().to_msg().sec
            if result.success:
                self.get_logger().info(
                    f"step{index} [{name}] 舵机{servo_id} result OK @ {stamp}s")
            else:
                self.get_logger().warn(
                    f"step{index} [{name}] 舵机{servo_id} 失败: {result.reason}")
        return True

    # ---- 播放：把整套步骤重复 repeat 遍 ----
    def play(self, filename):
        self._client.wait_for_server()
        data = self.load(filename)              # ← 调用上面的 load，把 yaml 读进来
        steps = data['steps']
        repeat = int(data.get('repeat', 1))     # 没写 repeat 就跑 1 遍
        self.get_logger().info(f"整套动作重复 {repeat} 遍")
        for r in range(repeat):
            self.get_logger().info(f"===== 第 {r + 1}/{repeat} 遍 =====")
            for i, step in enumerate(steps):
                if not self.run_step(i, step):
                    return
        self.get_logger().info(f"动作组 [{filename}] 播放结束")


def main(args=None):
    rclpy.init(args=args)
    node = SequencePlayer()
    name = node.get_parameter('motion_file').value   # ← 从命令行拿到 yaml 文件名
    try:
        node.play(name)
    finally:
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()