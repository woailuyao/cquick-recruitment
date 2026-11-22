"""给现场测试选点：算场地上哪些站位能看到最多的"地图内"标志。

可见判据（用你标定出的真实参数）：
  地面距离 1.0~3.6 m（外参实测可视范围）
  水平方向在光轴 ±21° 内（fx=1662.76, W=1280 -> atan(640/1662.76)）
"""
import math
import sys

sys.path.insert(0, '.')
from cvlib.fieldmap import load_field

HALF_FOV = math.degrees(math.atan(640 / 1662.76))
RMIN, RMAX = 0.8, 4.5


def visible(x, y, th_deg, points):
    th = math.radians(th_deg)
    c, s = math.cos(-th), math.sin(-th)
    out = []
    for p in points:
        dx, dy = p.x - x, p.y - y
        bx, by = c * dx - s * dy, s * dx + c * dy
        r = math.hypot(bx, by)
        if r < RMIN or r > RMAX:
            continue
        if abs(math.degrees(math.atan2(by, bx))) > HALF_FOV:
            continue
        out.append((p.mid, p.type, round(r, 2)))
    return out


def main():
    fm = load_field('config/field.yaml')
    print(f'场地外框 x[{fm.bounds[0]:.2f},{fm.bounds[1]:.2f}] '
          f'y[{fm.bounds[2]:.2f},{fm.bounds[3]:.2f}]')
    print(f'可视判据：距离 {RMIN}~{RMAX} m，水平 ±{HALF_FOV:.1f}°\n')

    best = []
    x = fm.bounds[0] + 0.3
    while x <= fm.bounds[1] - 0.3:
        y = fm.bounds[2] + 0.3
        while y <= fm.bounds[3] - 0.3:
            for th in range(-180, 180, 10):
                v = visible(x, y, th, fm.points)
                if len(v) >= 2:
                    best.append((len(v), round(x, 2), round(y, 2), th, v))
            y += 0.2
        x += 0.2

    best.sort(key=lambda t: -t[0])
    if best:
        print(f'最多能同时看到 {best[0][0]} 个地图标志\n')
    else:
        print('任何站位都看不到 2 个以上，检查参数')
        return
    shown = set()
    cnt = 0
    for n, x, y, th, v in best:
        key = (round(x), round(y), th // 30)
        if key in shown:
            continue
        shown.add(key)
        cnt += 1
        ids = ', '.join(f'{m}({t[0]},{r}m)' for m, t, r in v)
        print(f'  站位 ({x:5.2f}, {y:5.2f})  朝向 yaw={th:+4d}°  -> 可见 {n} 个: {ids}')
        if cnt >= 12:
            break


if __name__ == '__main__':
    main()
