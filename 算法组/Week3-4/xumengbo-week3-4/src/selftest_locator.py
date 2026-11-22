"""定位链路骨架离线自检（不依赖相机/模型/标注）。

验证：地图加载、投影接口、匹配与状态机可跑通。
运行：python selftest_locator.py
"""
import os
import loc_common
import map_matcher
import ground_projection


def main():
    # 1. 加载联调地图
    field_map = loc_common.FieldMap.from_yaml(
        os.path.join(loc_common.CONFIG_DIR, "field_theory.yaml"))
    total = sum(len(v) for v in field_map.markers.values())
    print(f"地图加载：{field_map.map_version}，标志点 {total} 个")
    for t, pts in field_map.markers.items():
        print(f"  {t}: {len(pts)} 个")

    # 2. 构造模拟观测：直接取地图上 3 个点当作载体系地面点（等价载体在原点、yaw=0）
    sample_types = ["LCross", "TCross", "XCross"]
    obs = []
    for t in sample_types:
        name, x, y = field_map.markers[t][0]
        obs.append({"type": t, "x_b": x, "y_b": y, "conf": 0.9, "u": 640, "v": 360})

    # 3. 匹配（外参未实测不影响匹配模块本身）
    res = map_matcher.match(obs, field_map)
    print("\n匹配结果：")
    print(f"  valid={res['valid']}  status={res['status']}  "
          f"markers={res['marker_count']}  residual={res['residual']}")
    print(f"  carrier_pose={res['carrier_pose']}")

    # 4. 不足 3 点应判 insufficient_markers
    res2 = map_matcher.match(obs[:2], field_map)
    print(f"\n两点场景：status={res2['status']}（应为 insufficient_markers）")

    print("\n自检完成：链路模块可导入、可调用，无异常。")


if __name__ == "__main__":
    main()
