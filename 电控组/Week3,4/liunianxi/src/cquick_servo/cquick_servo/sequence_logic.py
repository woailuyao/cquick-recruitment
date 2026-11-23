"""动作组抢占的纯逻辑（不依赖 rclpy，方便 pytest 覆盖）。

为什么单独拆一个模块：抢占的判定规则（何时中止、用哪条新路径、被取消算失败还是跳过）
是纯函数级的规则，和 ROS 通信没关系。放在这里可以直接单测，不用起节点、不用有硬件。
"""
from __future__ import annotations

from typing import Optional, Tuple

STATUS_OK = "ok"
STATUS_CANCELED = "canceled"
STATUS_FAILED = "failed"


def should_abort(preempt_requested: bool, step_index: int, total_steps: int) -> bool:
    """要不要在「进入下一步之前」中止当前动作组。

    抢占只在步与步之间生效：正在走的这一步不打断（舵机不会停在半路），
    最多多等一步的时间（当前动作组单步 1 s 上下）。
    """
    if not preempt_requested:
        return False
    if total_steps <= 0:
        return True
    return 1 <= step_index <= total_steps


def pick_next_path(pending: Optional[str], current: str) -> Tuple[str, bool]:
    """返回 (接下来要跑的路径, 是否发生抢占)。

    pending 为空、或和当前这条一样，就当作没抢占（继续跑 current）。
    """
    if pending and pending != current:
        return pending, True
    return current, False


def classify_canceled(result_ok: bool, message: str) -> str:
    """把一次 result 分成 ok / canceled / failed。

    被抢占时驱动会把在跑的 goal 取消，result 是 success=False + message=canceled，
    这种情况不算失败（否则日志里会凭空多出「失败 N 个 joint」）。
    """
    text = (message or "").lower()
    if result_ok:
        return STATUS_OK
    if "cancel" in text or "取消" in text:
        return STATUS_CANCELED
    return STATUS_FAILED
