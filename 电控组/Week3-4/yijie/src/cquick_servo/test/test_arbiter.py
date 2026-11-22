# -*- coding: utf-8 -*-
"""arbiter.py 的单测：抢占规则（按舵机粒度）。"""

from cquick_servo.arbiter import GoalArbiter


def test_same_servo_latest_wins():
    arbiter = GoalArbiter()
    first = arbiter.begin(1)
    assert arbiter.is_current(1, first)
    second = arbiter.begin(1)
    assert not arbiter.is_current(1, first)
    assert arbiter.is_current(1, second)


def test_different_servos_are_independent():
    arbiter = GoalArbiter()
    goal_1 = arbiter.begin(1)
    goal_2 = arbiter.begin(2)
    goal_3 = arbiter.begin(3)
    # 2 号、3 号后来的 goal 不影响 1 号
    assert arbiter.is_current(1, goal_1)
    assert arbiter.is_current(2, goal_2)
    assert arbiter.is_current(3, goal_3)


def test_tokens_are_unique_and_increasing():
    arbiter = GoalArbiter()
    tokens = [arbiter.begin(1), arbiter.begin(2), arbiter.begin(1)]
    assert len(set(tokens)) == 3
    assert tokens == sorted(tokens)


def test_forget_does_not_remove_newer_goal():
    arbiter = GoalArbiter()
    old = arbiter.begin(1)
    new = arbiter.begin(1)
    arbiter.forget(1, old)              # 旧 goal 退出时清理
    assert arbiter.is_current(1, new)   # 新 goal 不能被误删
    arbiter.forget(1, new)
    assert not arbiter.is_current(1, new)
