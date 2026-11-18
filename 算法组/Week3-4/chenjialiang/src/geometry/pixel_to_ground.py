
import cv2
import numpy as np

K = np.array([
    [700.0, 0.0, 480.0],
    [0.0, 700.0, 270.0],
    [0.0, 0.0, 1.0],
], dtype=np.float64)

def pixel_to_ray(u, v, K, R, C):
    raise NotImplementedError

def intersect_ground(O, d, min_dz=0.05):
    raise NotImplementedError

def main():
    h = 1.2
    C = np.array([0.0, 0.0, h])
    R = np.eye(3)

    fy = K[1, 1]
    cy = K[1, 2]

    print("v(px)   程序算的距离   手算 h*fy/(v-cy)   相对误差")
    for v in [300, 360, 450, 540]:
        ray = pixel_to_ray(480.0, float(v), K, R, C)
        if ray is None:
            continue
        pt = intersect_ground(*ray)
        if pt is None:
            print(f"{v:>5}   射线太平，丢弃")
            continue
        dist = float(np.hypot(pt[0] - C[0], pt[1] - C[1]))
        manual = h * fy / (v - cy) if v != cy else float("inf")
        err = abs(dist - manual) / manual if manual else float("inf")
        print(f"{v:>5}   {dist:>12.3f}   {manual:>16.3f}   {err:>8.1%}")

if __name__ == "__main__":
    main()
