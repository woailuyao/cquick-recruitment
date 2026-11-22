from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parents[1] / "figures"
OUT.mkdir(parents=True, exist_ok=True)

# 单异常点扫描
offset = np.array([0.05, 0.10, 0.15, 0.18, 0.21])

svd_yaw = [0.7720, 1.5119, 2.2382, 2.6674, 3.0919]
irls_yaw = [0.6140, 0.6949, 0.7183, 0.7222, 0.7250]

svd_pos = [3.007, 4.727, 6.679, 7.809, 8.880]
irls_pos = [2.932, 3.726, 3.824, 3.839, 3.925]

weights = [0.6093, 0.2509, 0.1550, 0.1259, 0.1061]

def line_plot(x, y1, y2, ylabel, title, filename):
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(x, y1, marker="o", label="SVD")
    ax.plot(x, y2, marker="s", label="Huber IRLS")
    ax.set_xlabel("Outlier displacement (m)")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(OUT / filename, dpi=200)
    plt.close(fig)

line_plot(
    offset, svd_yaw, irls_yaw,
    "Median yaw error (deg)",
    "Single-outlier yaw error comparison",
    "01_single_yaw.png"
)

line_plot(
    offset, svd_pos, irls_pos,
    "Median position error (mm)",
    "Single-outlier position error comparison",
    "02_single_position.png"
)

fig, ax = plt.subplots(figsize=(8, 5))
ax.plot(offset, weights, marker="o")
ax.set_xlabel("Outlier displacement (m)")
ax.set_ylabel("Median Huber weight")
ax.set_title("Huber weight versus outlier displacement")
ax.grid(True, alpha=0.3)
fig.tight_layout()
fig.savefig(OUT / "03_outlier_weights.png", dpi=200)
plt.close(fig)

# 多异常点实验
N = [5,5,5,5,8,8,8,8,10,10,10,10]
K = [1,1,2,2] * 3
D = [0.10,0.18,0.10,0.18] * 3

labels = [
    f"N={n}\nk={k}\n{d:.2f}m"
    for n,k,d in zip(N,K,D)
]

multi_svd_yaw = [
    .7142,1.0227,.9745,1.8017,
    .3680,.6784,.3874,.7380,
    .2146,.4108,.4063,.7409
]

multi_irls_yaw = [
    .2278,.2226,.3954,.4271,
    .1331,.1165,.1498,.1283,
    .0708,.0635,.1214,.1480
]

multi_svd_pos = [
    20.226,37.122,32.512,59.127,
    12.989,24.039,17.599,30.327,
    9.863,17.616,15.375,28.285
]

multi_irls_pos = [
    6.447,5.876,11.141,12.460,
    3.884,4.013,5.680,5.744,
    2.762,2.761,4.152,3.703
]

def bar_plot(a, b, ylabel, title, filename):
    x = np.arange(len(labels))
    width = 0.38

    fig, ax = plt.subplots(figsize=(13, 6))
    ax.bar(x-width/2, a, width, label="SVD")
    ax.bar(x+width/2, b, width, label="Huber IRLS")

    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=40, ha="right")
    ax.set_ylabel(ylabel)
    ax.set_title(title + " (30 random seeds per condition)")
    ax.legend()
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(OUT / filename, dpi=200)
    plt.close(fig)

bar_plot(
    multi_svd_yaw, multi_irls_yaw,
    "Median yaw error (deg)",
    "Multi-outlier yaw error comparison",
    "04_multi_yaw.png"
)

bar_plot(
    multi_svd_pos, multi_irls_pos,
    "Median position error (mm)",
    "Multi-outlier position error comparison",
    "05_multi_position.png"
)

print("ALL FIVE FIGURES GENERATED")
for p in sorted(OUT.glob("*.png")):
    print(p.name, p.stat().st_size, "bytes")
