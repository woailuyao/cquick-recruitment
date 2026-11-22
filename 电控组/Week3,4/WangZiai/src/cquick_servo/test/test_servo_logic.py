"""纯逻辑单元测试：不连硬件，`pytest` 直接跑。

覆盖：协议层收发、PWM 夹紧、goal 预校验、三套动作 yaml、mock 偏差模拟。
"""
import glob
import os

import time

import pytest
import rclpy
import yaml
from typing import cast
from rclpy.action import GoalResponse

from ament_index_python.packages import get_package_share_directory

from cquick_servo.servo_driver import (
    BusServo,
    MockTransport,
    PWM_ABS_MAX,
    PWM_ABS_MIN,
    ServoDriverNode,
)


class FakeGoal:
    def __init__(self, sid=1, pwm=1500, dur=500):
        self.id = sid
        self.pwm = pwm
        self.target_pwm = pwm  # 兼容旧字段名
        self.duration_ms = dur


@pytest.fixture(scope="module")
def node():
    rclpy.init(args=["--ros-args", "-p", "port:=mock", "-p", "watchdog_timeout:=0.0"])
    n = ServoDriverNode()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_mock_protocol():
    bus = BusServo(MockTransport())
    assert bus.ping(1) == "#001PID!"
    bus.move(1, 1600, 500)
    assert bus.read_pos(1) == 1600


def test_pwm_clamped(node):
    hi = node._p_int(node.P_PWM_MAX)   # 参数上限（默认 2100）
    lo = node._p_int(node.P_PWM_MIN)
    node._send(1, 3000, 100)           # 超参数上限 → 夹到 pwm_max
    assert node._servo.read_pos(1) == hi
    node._send(1, 100, 100)            # 超参数下限 → 夹到 pwm_min
    assert node._servo.read_pos(1) == lo


def test_goal_reject(node):
    assert node._on_goal(FakeGoal(sid=99)) == GoalResponse.REJECT       # id 不在列表
    assert node._on_goal(FakeGoal(pwm=3000)) == GoalResponse.REJECT      # 超限
    assert node._on_goal(FakeGoal(dur=0)) == GoalResponse.REJECT         # 时长非法
    assert node._on_goal(FakeGoal()) == GoalResponse.ACCEPT


def test_action_yaml_well_formed():
    """share/config/actions 下的每个 yaml：有 steps、每步有舵机、PWM 与时长合法。"""
    share = get_package_share_directory("cquick_servo")
    files = sorted(glob.glob(os.path.join(share, "config", "actions", "*.yaml")))
    assert files, "config/actions 下没有 yaml"
    for path in files:
        name = os.path.basename(path)
        doc = cast(dict, yaml.safe_load(open(path, encoding="utf-8")) or {})
        steps = doc.get("steps") or []
        assert steps, f"{name} 没有 steps"
        for step in steps:
            servos = step.get("servos") or []
            assert servos, f"{name} 有空 step"
            for s in servos:
                assert PWM_ABS_MIN <= s["pwm"] <= PWM_ABS_MAX
                assert s["duration_ms"] > 0


def test_mock_deviation_is_one_shot(monkeypatch):
    monkeypatch.setenv("CQ_MOCK_DEV", "60")
    bus = BusServo(MockTransport())
    bus.move(1, 1500, 100)
    assert bus.read_pos(1) == 1560     # 第一次差 60
    bus.move(1, 1500, 100)
    assert bus.read_pos(1) == 1500     # 重发后到位


def test_watchdog_and_pulr(node):
    from rclpy.parameter import Parameter

    node.set_parameters([Parameter("watchdog_timeout", value=0.2)])
    node._last_cmd_time -= 1.0         # 假装闲置 1 秒
    node._watchdog_tick()
    assert 1 in node._torqued_off_ids
    node._send(1, 1500, 100)           # 再动时应先 PULR
    assert 1 not in node._torqued_off_ids


def test_stale_reservation_released(node):
    """预占 30 秒还没执行 → 看门狗应自动释放；刚预占的不动。"""
    from rclpy.parameter import Parameter

    node.set_parameters([Parameter("watchdog_timeout", value=1.0)])
    node._busy_ids[1] = time.monotonic() - 60.0     # 假装 60 秒前预占
    node._busy_ids[2] = time.monotonic()            # 刚预占
    node._watchdog_tick()
    assert 1 not in node._busy_ids
    assert 2 in node._busy_ids
    node._busy_ids.pop(2, None)


def test_shutdown_service(node):
    """下电服务：全部卸力 + 关串口，之后拒绝新目标。"""
    from std_srvs.srv import Trigger

    assert node._shutdown_requested is False
    out = node._on_shutdown(Trigger.Request(), Trigger.Response())
    assert out.success is True
    assert "卸力" in out.message
    assert node._shutdown_requested is True
    assert node._on_goal(FakeGoal()) == GoalResponse.REJECT
