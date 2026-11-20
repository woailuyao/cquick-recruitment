#!/usr/bin/env python3
"""任务3装配态行程测试：逐关节、小步逼近、越界即退、自动记录。

默认每颗执行：校准位 → 正向每次+50 → 回不跟随即回中
                    → 负向每次-50 → 回不跟随即回中。
所有目标都是逻辑 PWM，驱动负责叠加 offset 和实际值限位。

用法：
    ros2 run cquick_servo joint_range_check
    ros2 run cquick_servo joint_range_check --ros-args -p step:=50 -p duration_ms:=1200

听到声音变闷、持续嗡嗡或看到顶结构，立即 Ctrl+C；脚本会请求整机卸力。
"""
import json
import time

import rclpy
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

from cquick_servo_interfaces.action import MoveServo


class JointRangeCheck(Node):
    def __init__(self):
        super().__init__('joint_range_check')
        self.declare_parameter('step', 50)
        self.declare_parameter('ids', [0])       # [0]=全部；也可 [2,7] 只测指定关节
        self.declare_parameter('margin', 50)
        self.declare_parameter('duration_ms', 1200)
        self.declare_parameter('pause_s', 0.3)
        self.declare_parameter('output_file', 'task3_joint_ranges.json')
        self._action = ActionClient(self, MoveServo, '/move_servo')
        self._get_params = self.create_client(
            GetParameters, '/servo_driver/get_parameters')
        self._set_params = self.create_client(
            SetParameters, '/servo_driver/set_parameters')
        self._active_handle = None
        self._emergency_sent = False

    def _wait(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError('等待 ROS2 响应超时')
            rclpy.spin_once(self, timeout_sec=0.02)
        return future.result()

    def _driver_config(self):
        if not self._get_params.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('驱动参数服务不可用')
        request = GetParameters.Request()
        request.names = [
            'servo_ids', 'joint_offsets', 'home_pwm', 'pwm_min', 'pwm_max']
        response = self._wait(self._get_params.call_async(request), 5.0)
        values = response.values
        return {
            'ids': [int(v) for v in values[0].integer_array_value],
            'offsets': [int(v) for v in values[1].integer_array_value],
            'home': int(values[2].integer_value),
            'pwm_min': int(values[3].integer_value),
            'pwm_max': int(values[4].integer_value),
        }

    def emergency_stop(self):
        if self._active_handle is not None:
            self._active_handle.cancel_goal_async()
        if not self._emergency_sent:
            self._emergency_sent = True
            if self._set_params.service_is_ready():
                request = SetParameters.Request()
                request.parameters = [
                    Parameter('torque_enable', value=False).to_parameter_msg()]
                self._set_params.call_async(request)
        self.get_logger().error('行程测试异常停止：已请求整机卸力')

    def goal(self, servo_id, logical, duration):
        handle = self._wait(self._action.send_goal_async(
            self._make_goal(servo_id, logical, duration)), 5.0)
        if handle is None or not handle.accepted:
            raise RuntimeError(f'id={servo_id} goal 被意外拒绝')
        self._active_handle = handle
        response = self._wait(
            handle.get_result_async(), duration / 1000.0 + 5.0)
        self._active_handle = None
        result = response.result
        self.get_logger().info(
            f'id={servo_id} logical={logical} success={result.success} '
            f'message={result.message} final={result.final_pwm}')
        return result

    @staticmethod
    def _make_goal(servo_id, logical, duration):
        goal = MoveServo.Goal()
        goal.id = int(servo_id)
        goal.pwm = int(logical)
        goal.duration_ms = int(duration)
        return goal

    def require_home(self, servo_id, home, duration):
        result = self.goal(servo_id, home, duration)
        if not result.success:
            raise RuntimeError(f'id={servo_id} 无法回校准位: {result.message}')

    def run(self):
        if not self._action.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('move_servo 不可用')
        config = self._driver_config()
        requested_ids = [int(v) for v in self.get_parameter('ids').value]
        ids = config['ids'] if requested_ids == [0] else requested_ids
        unknown = [servo_id for servo_id in ids if servo_id not in config['ids']]
        if unknown:
            raise ValueError(f'指定 ID 不在驱动 servo_ids 中: {unknown}')
        offsets = config['offsets']
        home = config['home']
        step = int(self.get_parameter('step').value)
        margin = int(self.get_parameter('margin').value)
        duration = int(self.get_parameter('duration_ms').value)
        pause = float(self.get_parameter('pause_s').value)
        output = str(self.get_parameter('output_file').value)
        if step not in (20, 50) or margin < 50 or margin > 100:
            raise ValueError('step 只允许 20/50，margin 必须为 50~100')
        if not 1000 <= duration <= 1500:
            raise ValueError('行程测试 duration_ms 必须在 1000~1500')

        records = {}
        for servo_id in ids:
            offset = offsets[servo_id - 1]
            home_actual = home + offset
            record = {
                'offset': offset,
                'home_logical': home,
                'home_actual': home_actual,
                'step': step,
                'margin': margin,
            }
            self.get_logger().info(f'========== 行程测试 ID {servo_id} ==========')

            for direction, label in ((1, 'positive'), (-1, 'negative')):
                self.require_home(servo_id, home, duration)
                logical = home
                last_ok_actual = home_actual
                stop_reason = 'global_limit'

                while True:
                    candidate = logical + direction * step
                    candidate_actual = candidate + offset
                    if not config['pwm_min'] <= candidate_actual <= config['pwm_max']:
                        break

                    result = self.goal(servo_id, candidate, duration)
                    if result.success:
                        logical = candidate
                        last_ok_actual = candidate_actual
                        time.sleep(pause)  # 观察/散热；到位由 result 判定
                        continue

                    if str(result.message).startswith('not_reached'):
                        stop_reason = (
                            f'not_reached at actual={candidate_actual}, '
                            f'final={result.final_pwm}')
                        self.get_logger().warning(
                            f'id={servo_id} {label} 边界：{stop_reason}，立即回校准位')
                        break
                    raise RuntimeError(
                        f'id={servo_id} 非预期失败: {result.message}')

                record[label + '_last_ok_actual'] = last_ok_actual
                record[label + '_stop_reason'] = stop_reason
                self.require_home(servo_id, home, duration)

            lower = record['negative_last_ok_actual']
            upper = record['positive_last_ok_actual']
            record['suggested_actual_min'] = lower + margin
            record['suggested_actual_max'] = upper - margin
            record['suggested_logical_min'] = lower + margin - offset
            record['suggested_logical_max'] = upper - margin - offset
            if record['suggested_actual_min'] > record['suggested_actual_max']:
                raise RuntimeError(
                    f'id={servo_id} 可用行程不足两侧余量，禁止据此设计动作')
            records[str(servo_id)] = record
            # 每完成一颗就落盘，后续关节异常也不会丢失已有记录。
            with open(output, 'w', encoding='utf-8') as handle:
                json.dump(records, handle, ensure_ascii=False, indent=2)
            self.get_logger().info(
                f'ID {servo_id} 实际可用[{lower},{upper}]；'
                f'动作建议逻辑[{record["suggested_logical_min"]},'
                f'{record["suggested_logical_max"]}]')
            time.sleep(1.0)

        with open(output, 'w', encoding='utf-8') as handle:
            json.dump(records, handle, ensure_ascii=False, indent=2)
        self.get_logger().info(f'全部行程测试完成，记录已保存：{output}')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = JointRangeCheck()
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        node.get_logger().warning('Ctrl+C，停止行程测试')
    except Exception as exc:
        node.get_logger().error(str(exc))
    finally:
        if not ok:
            node.emergency_stop()
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
