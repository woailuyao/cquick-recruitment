import math
import numpy as np

from robust_fit_irls import (
    weighted_fit_2d,
    huber_irls_fit
)


def pose_error(R_est, t_est, R_true, t_true):

    D = R_est @ R_true.T

    yaw = abs(math.atan2(D[1, 0], D[0, 0]))

    position = np.linalg.norm(t_est - t_true)

    return math.degrees(yaw), position


def main():

    a = np.array([
        [0, 0],
        [1, 0],
        [0, 1],
        [1, 1],
        [2, 0.5]
    ], dtype=float)

    theta = 0.35

    R_true = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])

    t_true = np.array([1.2, -0.4])

    noise = np.array([
        [0.004, -0.003],
        [-0.002, 0.004],
        [0.003, 0.001],
        [-0.004, -0.002],
        [0.002, 0.003]
    ])

    b_clean = a @ R_true.T + t_true + noise

    print("========== 实验1：正常噪声 ==========")

    result_clean = huber_irls_fit(
        a, b_clean, delta=0.02
    )

    print("RMS:", result_clean["rms"])
    print("Weights:", result_clean["weights"])
    print("Converged:", result_clean["converged"])

    assert result_clean["converged"]

    print("\n========== 实验2：加入异常匹配点 ==========")

    b_outlier = b_clean.copy()

    b_outlier[4] += np.array([0.45, -0.35])

    R_base, t_base = weighted_fit_2d(
        a, b_outlier, np.ones(len(a))
    )

    result = huber_irls_fit(
        a, b_outlier, delta=0.02
    )

    base_yaw, base_position = pose_error(
        R_base, t_base, R_true, t_true
    )

    robust_yaw, robust_position = pose_error(
        result["R"], result["t"], R_true, t_true
    )

    print("Baseline:")
    print("Yaw error:", base_yaw, "deg")
    print("Position error:", base_position, "m")

    print("\nHuber IRLS:")
    print("Yaw error:", robust_yaw, "deg")
    print("Position error:", robust_position, "m")
    print("Weights:", result["weights"])
    print("RMS:", result["rms"])
    print("Iterations:", result["iterations"])

    assert robust_yaw < base_yaw
    assert robust_position < base_position
    assert result["weights"][-1] < 0.2
    assert result["converged"]

    print("\nPASS: 合成异常点实验通过")


if __name__ == "__main__":
    main()
