import math
import numpy as np

from robust_fit_irls import weighted_fit_2d, huber_irls_fit

a = np.array([
    [0, 0], [1, 0], [0, 1],
    [1, 1], [2, 0.5]
], dtype=float)

theta = 0.35
R_true = np.array([
    [np.cos(theta), -np.sin(theta)],
    [np.sin(theta),  np.cos(theta)]
])
t_true = np.array([1.2, -0.4])

gate = 0.22
max_rms = 0.15
magnitudes = [0.05, 0.10, 0.15, 0.18, 0.21]

def errors(R, t):
    D = R @ R_true.T
    yaw = abs(math.degrees(
        math.atan2(D[1, 0], D[0, 0])
    ))
    pos = np.linalg.norm(t - t_true)
    return yaw, pos

print("异常点门限内鲁棒性实验")
print("每种情况使用30个随机噪声种子")
print("单位：角度deg，距离mm")
print()

for magnitude in magnitudes:

    base_errors = []
    robust_errors = []
    base_valid = 0
    robust_valid = 0
    robust_weights = []
    robust_weighted_rms = []

    for seed in range(30):

        rng = np.random.default_rng(seed)

        noise = rng.normal(
            0, 0.004, size=(len(a), 2)
        )

        b = a @ R_true.T + t_true + noise

        # 加入一个方向固定、大小可控的异常偏移
        b[4] += magnitude * np.array([0.8, -0.6])

        R0, t0 = weighted_fit_2d(
            a, b, np.ones(len(a))
        )

        result = huber_irls_fit(
            a, b, delta=0.02
        )

        for R, t, target, valid_counter in [
            (R0, t0, base_errors, "base"),
            (result["R"], result["t"], robust_errors, "robust")
        ]:

            residuals = np.linalg.norm(
                a @ R.T + t - b, axis=1
            )

            rms = np.sqrt(np.mean(residuals ** 2))

            target.append(errors(R, t))

            if np.max(residuals) <= gate and rms <= max_rms:
                if valid_counter == "base":
                    base_valid += 1
                else:
                    robust_valid += 1

        w = result["weights"]
        r = result["residuals"]

        wrms = np.sqrt(
            np.sum(w * r**2) / np.sum(w)
        )

        robust_weights.append(w[-1])
        robust_weighted_rms.append(wrms)

    def median(data, index):
        return np.median([x[index] for x in data])

    print(f"异常偏移: {magnitude:.2f} m")

    print(
        "SVD:",
        f"yaw={median(base_errors, 0):.4f}",
        f"pos={median(base_errors, 1)*1000:.3f} mm",
        f"通过={base_valid}/30"
    )

    print(
        "IRLS:",
        f"yaw={median(robust_errors, 0):.4f}",
        f"pos={median(robust_errors, 1)*1000:.3f} mm",
        f"通过={robust_valid}/30",
        f"异常点权重={np.median(robust_weights):.4f}",
        f"加权RMS={np.median(robust_weighted_rms)*1000:.3f} mm"
    )

    print()

print("SWEEP COMPLETE")
