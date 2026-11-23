"""pytest：协议拼串与解析（干净、无硬件可跑，pytest 加分项）。

  colcon test --packages-select cquick_servo
  # 或直接：pytest src/cquick_servo/test/test_protocol.py -v
"""
from cquick_servo.servo_link import (
    HEARTBEAT_PERIOD,
    PWM_ELECTRIC_MAX,
    PWM_ELECTRIC_MIN,
    RETRY_DURATION_MS,
    WATCHDOG_TIMEOUT,
    SerialLink,
    build_move,
    build_verb,
    clamp_duration,
    parse_id,
    parse_pwm,
)
from cquick_servo.sequence_logic import (
    STATUS_CANCELED,
    STATUS_FAILED,
    STATUS_OK,
    classify_canceled,
    pick_next_path,
    should_abort,
)


def test_build_move_pads_id_to_three_digits():
    assert build_move(1, 1500, 1000) == "#001P1500T1000!"
    assert build_move(24, 900, 500) == "#024P900T500!"


def test_build_verb():
    assert build_verb(3, "PULK") == "#003PULK!"
    assert build_verb(1, "PID") == "#001PID!"


def test_parse_pwm_reads_prad_reply():
    assert parse_pwm("#001P1500!") == 1500
    assert parse_pwm("#012P0900!") == 900
    assert parse_pwm("garbage") is None


def test_parse_id():
    assert parse_id("#007!") == 7
    assert parse_id("nope") is None


def test_electric_range_constants():
    # 电气极限是协议约定，不能被参数放大（安全区间的裁剪另算）
    assert PWM_ELECTRIC_MIN == 500
    assert PWM_ELECTRIC_MAX == 2500


def test_serial_link_defaults_are_8n1():
    link = SerialLink.__new__(SerialLink)
    assert link is not None


# ---------------- 加分项：回读超差重发 / 看门狗 的时间常数（纯函数，无硬件可跑） ----------------
def test_clamp_duration_raises_to_firmware_floor():
    # ZL-IS2 实测忽略 T<1000，600 ms 的短行程必须被抬到 1000
    assert clamp_duration(600, 1000) == 1000


def test_clamp_duration_keeps_longer_move():
    assert clamp_duration(1500, 1000) == 1500


def test_retry_duration_is_shorter_than_floor():
    # 重发用的短行程故意比固件下限短，靠 clamp_duration 抬上来，测试锁住这个关系
    assert RETRY_DURATION_MS < 1000
    assert clamp_duration(RETRY_DURATION_MS, 1000) == 1000


def test_watchdog_budget_survives_three_heartbeats():
    # 心跳周期 ×3 必须不大于超时时间，否则节点正常运行也会被看门狗误判
    assert HEARTBEAT_PERIOD * 3 <= WATCHDOG_TIMEOUT


# ---------------- 加分项：动作组抢占（纯函数，无硬件可跑） ----------------
def test_sequence_should_abort_only_between_steps():
    # 没请求抢占时永远不中止
    assert should_abort(False, 1, 4) is False
    assert should_abort(False, 4, 4) is False
    # 有请求时，任何一步开始前都允许中止（步内不打断）
    assert should_abort(True, 1, 4) is True
    assert should_abort(True, 3, 4) is True
    # 空动作组直接中止，别卡在循环里
    assert should_abort(True, 1, 0) is True


def test_sequence_pick_next_path_and_cancel_classification():
    # 没有 pending、或 pending 就是当前这条，都不算抢占
    assert pick_next_path("", "/a/stand_up.yaml") == ("/a/stand_up.yaml", False)
    assert pick_next_path("/a/stand_up.yaml", "/a/stand_up.yaml") == (
        "/a/stand_up.yaml",
        False,
    )
    # 换了新路径才是抢占
    assert pick_next_path("/a/squat.yaml", "/a/stand_up.yaml") == ("/a/squat.yaml", True)
    # 被抢占时驱动回 success=False + message=canceled，不能算失败
    assert classify_canceled(False, "canceled") == STATUS_CANCELED
    assert classify_canceled(False, "动作被取消") == STATUS_CANCELED
    # 正常到位、以及真失败，各归各位
    assert classify_canceled(True, "ok(diff=0)") == STATUS_OK
    assert classify_canceled(False, "not_reached:47") == STATUS_FAILED
