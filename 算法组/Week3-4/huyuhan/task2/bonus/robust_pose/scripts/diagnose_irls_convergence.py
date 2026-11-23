import math
import numpy as np

from robust_fit_irls import huber_irls_fit

THETA = 0.35
R_TRUE = np.array([
    [np.cos(THETA), -np.sin(THETA)],
    [np.sin(THETA), np.cos(THETA)]
])
T_TRUE = np.array([1.2, -0.4])


def pose_error(R, t):
    D = R @ R_TRUE.T
    yaw = abs(math.degrees(
        math.atan2(D[1, 0], D[0, 0])
    ))
    pos = np.linalg.norm(t - T_TRUE)
    return yaw, pos


def rotation_difference(R1, R2):
    D = R1 @ R2.T
    return abs(math.degrees(
        math.atan2(D[1, 0], D[0, 0])
    ))


def main():
    print("========== IRLS收敛性专项诊断 ==========")

    for magnitude in [0.10, 0.18]:
        n, k = 5, 2

        point_rng = np.random.default_rng(20261122 + n)
        a = point_rng.uniform(-1.5, 1.5, (n, 2))

        for seed in range(30):
            rng = np.random.default_rng(
                100000*n + 1000*k
                + int(magnitude*100) + seed
            )

            noise = rng.normal(0, 0.004, (n, 2))
            b = a @ R_TRUE.T + T_TRUE + noise

            indices = rng.choice(n, size=k, replace=False)

            directions = rng.normal(size=(k, 2))
            directions /= np.linalg.norm(
                directions, axis=1, keepdims=True
            )

            b[indices] += magnitude * directions

            r50 = huber_irls_fit(
                a, b, delta=0.02, max_iter=50
            )

            if r50["converged"]:
                continue

            print(
                f"\n偏移={magnitude:.2f}m "
                f"seed={seed} "
                f"原始迭代={r50['iterations']}"
            )

            for limit in [200, 500]:
                r = huber_irls_fit(
                    a, b,
                    delta=0.02,
                    max_iter=limit
                )

                yaw, pos = pose_error(r["R"], r["t"])

                print(
                    f"max_iter={limit} "
                    f"converged={r['converged']} "
                    f"iterations={r['iterations']} "
                    f"yaw={yaw:.5f}deg "
                    f"pos={pos*1000:.4f}mm "
                    f"delta_yaw={rotation_difference(r['R'], r50['R']):.6f}deg "
                    f"delta_t={np.linalg.norm(r['t']-r50['t'])*1000:.5f}mm"
                )

    print("\nCONVERGENCE DIAGNOSTIC COMPLETE")


if __name__ == "__main__":
    main()
