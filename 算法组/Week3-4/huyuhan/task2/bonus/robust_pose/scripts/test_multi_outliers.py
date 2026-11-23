import math
import numpy as np

from robust_fit_irls import weighted_fit_2d, huber_irls_fit

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

def main():
    print("========== 双异常点鲁棒性实验 ==========")
    print("每组30个随机种子")
    print("Huber delta = 0.02 m")
    print()

    for n in [5, 8, 10]:
        point_rng = np.random.default_rng(20261122 + n)
        a = point_rng.uniform(-1.5, 1.5, (n, 2))

        for k in [1, 2]:
            for magnitude in [0.10, 0.18]:

                base_yaw, base_pos = [], []
                irls_yaw, irls_pos = [], []

                base_pass = 0
                irls_pass = 0
                outlier_weights = []
                raw_rms_values = []
                weighted_rms_values = []
                converged_flags = []
                iteration_counts = []

                for seed in range(30):
                    rng = np.random.default_rng(
                        100000*n + 1000*k
                        + int(magnitude*100) + seed
                    )

                    noise = rng.normal(
                        0, 0.004, (n, 2)
                    )

                    b = a @ R_TRUE.T + T_TRUE + noise

                    indices = rng.choice(
                        n, size=k, replace=False
                    )

                    directions = rng.normal(size=(k, 2))
                    directions /= np.linalg.norm(
                        directions, axis=1, keepdims=True
                    )

                    b[indices] += magnitude * directions

                    R0, t0 = weighted_fit_2d(
                        a, b, np.ones(n)
                    )

                    result = huber_irls_fit(
                        a, b, delta=0.02
                    )

                    converged_flags.append(result["converged"])
                    iteration_counts.append(result["iterations"])

                    for R, t, yaw_list, pos_list, method in [
                        (R0, t0, base_yaw, base_pos, "svd"),
                        (result["R"], result["t"],
                         irls_yaw, irls_pos, "irls")
                    ]:
                        residuals = np.linalg.norm(
                            a @ R.T + t - b, axis=1
                        )

                        rms = np.sqrt(np.mean(residuals**2))
                        yaw, pos = pose_error(R, t)

                        yaw_list.append(yaw)
                        pos_list.append(pos)

                        if np.max(residuals) <= 0.22 and rms <= 0.15:
                            if method == "svd":
                                base_pass += 1
                            else:
                                irls_pass += 1

                        if method == "irls":
                            raw_rms_values.append(rms)
                            weights = result["weights"]
                            wrms = np.sqrt(
                                np.sum(weights * residuals**2)
                                / np.sum(weights)
                            )
                            weighted_rms_values.append(wrms)
                            outlier_weights.extend(
                                weights[indices].tolist()
                            )

                print(
                    f"N={n}, 异常点={k}, "
                    f"偏移={magnitude:.2f}m"
                )

                print(
                    f"SVD: yaw={np.median(base_yaw):.4f}° "
                    f"pos={np.median(base_pos)*1000:.3f}mm "
                    f"pass={base_pass}/30"
                )

                print(
                    f"IRLS: yaw={np.median(irls_yaw):.4f}° "
                    f"pos={np.median(irls_pos)*1000:.3f}mm "
                    f"pass={irls_pass}/30"
                )

                print(
                    f"异常点权重={np.median(outlier_weights):.4f} "
                    f"raw_RMS={np.median(raw_rms_values)*1000:.3f}mm "
                    f"weighted_RMS={np.median(weighted_rms_values)*1000:.3f}mm"
                )

                print(
                    f"IRLS收敛={sum(converged_flags)}/30 "
                    f"median_iter={np.median(iteration_counts):.1f} "
                    f"max_iter={np.max(iteration_counts)}"
                )
                print()

    print("MULTI-OUTLIER EXPERIMENT COMPLETE")

if __name__ == "__main__":
    main()
