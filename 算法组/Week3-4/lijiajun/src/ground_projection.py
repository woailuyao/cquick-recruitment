import math

import cv2
import numpy as np


def camera_rotation(pitch_deg, roll_deg):
    ph, ps = math.radians(pitch_deg), math.radians(roll_deg)
    c, s = math.cos(ph), math.sin(ph)
    z = np.array([c, 0.0, -s])
    x = np.array([0.0, -1.0, 0.0])
    y = np.cross(z, x)
    xr = x * math.cos(ps) + y * math.sin(ps)
    yr = -x * math.sin(ps) + y * math.cos(ps)
    return np.column_stack([xr, yr, z])


def undistort_pixels(pixels, K, dist):
    pts = np.asarray(pixels, dtype=float).reshape(-1, 1, 2)
    return cv2.undistortPoints(pts, K, dist, P=K).reshape(-1, 2)


def distort_pixels(pixels_und, K, dist):
    p = np.asarray(pixels_und, dtype=float).reshape(-1, 2)
    norm = (p - np.array([K[0, 2], K[1, 2]])) / np.array([K[0, 0], K[1, 1]])
    obj = np.column_stack([norm, np.ones(len(norm))])
    out, _ = cv2.projectPoints(obj, np.zeros(3), np.zeros(3), K,
                               np.asarray(dist, dtype=float).ravel())
    return out.reshape(-1, 2)


def pixel_to_ground(pixels, K, dist, height_m, pitch_deg, roll_deg, strict=True):
    p = undistort_pixels(pixels, K, dist)
    R_cw = camera_rotation(pitch_deg, roll_deg)
    ones = np.ones((len(p), 1))
    d_cam = np.hstack([p, ones]) @ np.linalg.inv(K).T
    d_loc = d_cam @ R_cw.T
    dz = d_loc[:, 2]
    below = dz < -1e-9
    if strict and not np.all(below):
        raise ValueError("存在不指向地面的像素（在地平线之上），不能求交")
    lam = np.where(below, -height_m / np.where(below, dz, -1.0), np.nan)
    ground = np.column_stack([d_loc[:, 0] * lam, d_loc[:, 1] * lam])
    return ground, lam, d_loc


def ground_to_pixel(ground_xy, K, dist, height_m, pitch_deg, roll_deg):
    g = np.asarray(ground_xy, dtype=float).reshape(-1, 2)
    R_cw = camera_rotation(pitch_deg, roll_deg)
    P_loc = np.column_stack([g[:, 0], g[:, 1], np.zeros(len(g)) - height_m])
    d_cam = P_loc @ R_cw
    uv = d_cam @ K.T
    uv = uv[:, :2] / uv[:, 2:3]
    return distort_pixels(uv, K, dist)


def horizon_row(K, pitch_deg):
    return K[1, 2] - K[1, 1] * math.tan(math.radians(pitch_deg))


def selftest(K, dist, height_m, pitch_deg, roll_deg, size=(1280, 720)):
    rows = []
    for u in (60.0, 340.0, 640.0, 940.0, 1220.0):
        for v in (200.0, 300.0, 420.0, 560.0, 700.0):
            if v >= size[1]:
                continue
            g, lam, _ = pixel_to_ground([[u, v]], K, dist, height_m, pitch_deg, roll_deg,
                                        strict=False)
            if not np.isfinite(g[0][0]):
                continue
            back = ground_to_pixel(g, K, dist, height_m, pitch_deg, roll_deg)
            err = float(np.hypot(*(back[0] - np.array([u, v]))))
            rows.append((u, v, g[0][0], g[0][1], err))
    worst = max(r[4] for r in rows)
    return rows, worst
