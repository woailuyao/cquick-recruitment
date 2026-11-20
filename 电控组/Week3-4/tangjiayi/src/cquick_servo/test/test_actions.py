#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""动作 yaml 的静态校验（不用接硬件，不用起节点）。

为什么要有这个测试：
    动作是数据，数据也会错。pwm 写成 2200（超出 pwm_max 2100）时，
    现场表现是「跑动作组跑一半被 server 拒收、整套动作停住」，
    而这种错完全可以在提交前用 pytest 拦掉。

规则：
    - 每个 joint 必须有 id / pwm；duration_ms 可省（= 用 default_duration_ms）
    - id 必须在 servo_params.yaml 的 servo_ids 里
    - pwm 必须落在 pwm_min ~ pwm_max 之间
    - 动作文件至少 2 个 step，且至少有一个 step 是「多关节同时动」
      （任务 3 要求的多舵机协同，交一个单关节动作组体现不出来）
"""

import glob
import os

import pytest
import yaml

CONFIG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
PARAMS_FILE = os.path.join(CONFIG_DIR, 'servo_params.yaml')
ACTION_FILES = sorted(glob.glob(os.path.join(CONFIG_DIR, 'actions', '*.yaml')))


@pytest.fixture(scope='module')
def driver_params():
    with open(PARAMS_FILE, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    return data['servo_driver']['ros__parameters']


def test_params_yaml_top_key_is_node_name(driver_params):
    # yaml 最外层 key 必须等于节点名，写错了参数静默不加载，最难查
    with open(PARAMS_FILE, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    assert 'servo_driver' in data
    assert 'ros__parameters' in data['servo_driver']


def test_action_files_exist():
    assert len(ACTION_FILES) >= 2, '任务 3 至少交两套动作'


@pytest.mark.parametrize('path', ACTION_FILES, ids=[os.path.basename(p) for p in ACTION_FILES])
def test_action_file_is_playable(path, driver_params):
    with open(path, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle)

    steps = data['steps']
    assert len(steps) >= 2, f'{path} 至少要 2 个 step'

    servo_ids = driver_params['servo_ids']
    pwm_min, pwm_max = driver_params['pwm_min'], driver_params['pwm_max']
    multi_joint_steps = 0

    for step in steps:
        joints = step['joints']
        assert joints, f'{path} 里有个 step 没有 joints'
        if len(joints) > 1:
            multi_joint_steps += 1
        for joint in joints:
            assert joint['id'] in servo_ids, \
                f'{path}: id {joint["id"]} 不在 servo_ids {servo_ids} 里，会被 server 拒收'
            assert pwm_min <= joint['pwm'] <= pwm_max, \
                f'{path}: id {joint["id"]} 的 pwm {joint["pwm"]} 超出 {pwm_min}~{pwm_max}'
            duration = joint.get('duration_ms')
            if duration is not None:
                # 给太小会猛跳撞结构，这是电控的通用下限
                assert duration >= 100, \
                    f'{path}: id {joint["id"]} 的 duration {duration}ms 太小，会猛跳'

    assert multi_joint_steps >= 1, f'{path} 里没有任何一步是「多关节同时动」'
