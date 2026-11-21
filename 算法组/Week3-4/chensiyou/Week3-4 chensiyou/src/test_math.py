import numpy as np
import math
import yaml


FX = 1679.29
FY = 665.24
CX = 1661.56
CY = 300.86


CAMERA_HEIGHT = 0.5
CAMERA_PITCH_DEG = 15.0


def pixel_to_ground(u, v):

    x_c = (u - CX) / FX
    y_c = (v - CY) / FY
    ray_camera = np.array([x_c, y_c, 1.0])


    pitch_rad = math.radians(CAMERA_PITCH_DEG)
    cos_p, sin_p = math.cos(pitch_rad), math.sin(pitch_rad)

    R_x = np.array([
        [1, 0, 0],
        [0, cos_p, sin_p],
        [0, -sin_p, cos_p]
    ])

    ray_ground = np.dot(R_x, ray_camera)


    dy = ray_ground[1]


    if dy <= 0.001:
        return None, None


    lam = CAMERA_HEIGHT / dy


    X_forward = lam * ray_ground[2]
    Y_right = lam * ray_ground[0]

    return X_forward, Y_right



if __name__ == '__main__':
    print(f"当前参数: 高度 {CAMERA_HEIGHT}m, 俯角 {CAMERA_PITCH_DEG}度\n")

    u1, v1 = 640, 720
    X1, Y1 = pixel_to_ground(u1, v1)
    if X1 is not None:
        print(f"画面最底部 {u1, v1} -> 前方: {X1:.2f} 米")
    else:
        print(f"画面最底部 {u1, v1} -> 指向天空，无法测距")