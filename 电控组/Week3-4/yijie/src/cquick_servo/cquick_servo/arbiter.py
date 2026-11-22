# -*- coding: utf-8 -*-
"""arbiter.py —— 按舵机粒度做 goal 抢占（纯逻辑，不依赖 ROS）。

规则只有一条：**同一个舵机同时只允许最新的那个 goal 继续跑**。

* 新 goal 进来 -> begin(id) 拿到更大的 token；
* 旧 goal 在循环里 is_current(id, token) 变 False -> 自己 abort，不发任何指令；
* 不同 id 之间互不影响（任务 3 一个 step 里几个关节并行就靠这个）。

为什么不用「全局最新」：一个 step 里 6 个关节同时下发，如果全局只留最新的，
先发的 5 个会被自己人误杀，动作组直接瘫痪。
"""

import threading


class GoalArbiter:
    """维护每个舵机最新 goal 的序号。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._counter = 0
        self._newest = {}          # servo_id -> token

    def begin(self, servo_id: int) -> int:
        """登记一个新 goal，返回它的 token（单调递增，全局唯一）。"""
        with self._lock:
            self._counter += 1
            token = self._counter
            self._newest[int(servo_id)] = token
            return token

    def is_current(self, servo_id: int, token: int) -> bool:
        """这个 token 是否仍是该舵机最新的 goal。"""
        with self._lock:
            return self._newest.get(int(servo_id)) == token

    def newest(self, servo_id: int) -> int:
        with self._lock:
            return self._newest.get(int(servo_id), 0)

    def forget(self, servo_id: int, token: int) -> None:
        """goal 结束后清理，避免字典无限增长（可选）。"""
        with self._lock:
            if self._newest.get(int(servo_id)) == token:
                self._newest.pop(int(servo_id), None)
