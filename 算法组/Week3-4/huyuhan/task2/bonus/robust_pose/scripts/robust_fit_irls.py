import numpy as np


def weighted_fit_2d(a, b, weights):
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    w = np.asarray(weights, dtype=float)

    if a.shape != b.shape or a.ndim != 2 or a.shape[1] != 2:
        raise ValueError("a and b must have shape (N, 2)")

    if w.shape != (len(a),) or np.any(w < 0):
        raise ValueError("Invalid weights")

    if not np.isfinite(a).all() or not np.isfinite(b).all():
        raise ValueError("Non-finite coordinates")

    total = w.sum()
    if total <= 0:
        raise ValueError("All weights are zero")

    w = w / total

    ac = np.sum(a * w[:, None], axis=0)
    bc = np.sum(b * w[:, None], axis=0)

    A = a - ac
    B = b - bc

    weighted_A = A * np.sqrt(w[:, None])
    weighted_B = B * np.sqrt(w[:, None])

    if (np.linalg.matrix_rank(weighted_A) < 2 or
            np.linalg.matrix_rank(weighted_B) < 2):
        raise ValueError("Degenerate point configuration")

    H = A.T @ (B * w[:, None])

    u, _, vt = np.linalg.svd(H)

    R = vt.T @ u.T

    if np.linalg.det(R) < 0:
        vt[-1, :] *= -1
        R = vt.T @ u.T

    t = bc - R @ ac

    return R, t


def huber_irls_fit(a, b, delta=0.02,
                   max_iter=500, tol=1e-9):

    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)

    if len(a) < 3:
        raise ValueError("At least three correspondences required")

    if delta <= 0:
        raise ValueError("delta must be positive")

    R, t = weighted_fit_2d(
        a, b, np.ones(len(a))
    )

    converged = False

    for iteration in range(max_iter):

        residuals = np.linalg.norm(
            a @ R.T + t - b, axis=1
        )

        weights = np.ones(len(a))

        mask = residuals > delta
        weights[mask] = (
            delta / residuals[mask]
        )

        R_new, t_new = weighted_fit_2d(
            a, b, weights
        )

        change = (
            np.linalg.norm(R_new - R)
            + np.linalg.norm(t_new - t)
        )

        R, t = R_new, t_new

        if change < tol:
            converged = True
            break

    residuals = np.linalg.norm(
        a @ R.T + t - b, axis=1
    )

    weights = np.minimum(
        1.0,
        delta / np.maximum(residuals, 1e-12)
    )

    return {
        "R": R,
        "t": t,
        "residuals": residuals,
        "weights": weights,
        "iterations": iteration + 1,
        "converged": converged,
        "rms": float(np.sqrt(np.mean(residuals ** 2))),
        "max_residual": float(np.max(residuals))
    }
