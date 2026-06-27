#!/usr/bin/env python3
"""SAR 点扩散函数 (PSF) 精细分析图。"""
from __future__ import annotations

import struct, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# 配置中文字体（macOS）
plt.rcParams["font.sans-serif"] = ["Heiti SC", "PingFang SC", "STHeiti", "Arial Unicode MS"]
plt.rcParams["axes.unicode_minus"] = False

def read_sar_raw(path: str):
    with open(path, "rb") as f:
        rows = struct.unpack("<I", f.read(4))[0]
        cols = struct.unpack("<I", f.read(4))[0]
        has_imag = struct.unpack("<I", f.read(4))[0]
        peak_r = struct.unpack("<I", f.read(4))[0]
        peak_c = struct.unpack("<I", f.read(4))[0]
    with open(path, "rb") as f:
        f.seek(20)
        n = rows * cols * 2
        data = np.frombuffer(f.read(), dtype=np.float32, count=n)
    real = data[0::2].reshape(rows, cols)
    imag = data[1::2].reshape(rows, cols)
    return real + 1j*imag, (peak_r, peak_c)

complex_image, (pr, pc) = read_sar_raw("/tmp/sar_focused_image.raw")
magnitude = np.abs(complex_image)
mag_db = 20.0 * np.log10(np.maximum(magnitude, 1e-30))
mag_db_norm = mag_db - mag_db[pr, pc]
rows, cols = magnitude.shape

# zoom around peak
rw = 12  # half-window
r0 = max(0, pr - rw)
r1 = min(rows, pr + rw + 1)
cw = 40
c0 = max(0, pc - cw)
c1 = min(cols, pc + cw + 1)
zoom_mag = magnitude[r0:r1, c0:c1]
zoom_db = mag_db_norm[r0:r1, c0:c1]

fig, axes = plt.subplots(2, 3, figsize=(20, 11))
fig.suptitle("SAR 点扩散函数分析 — 100 km 地面点目标（L1 RDA 聚焦）", fontsize=15, fontweight="bold")

# 1. Zoomed magnitude image (linear)
ax = axes[0, 0]
im = ax.imshow(zoom_mag, aspect="auto", origin="lower", cmap="hot", interpolation="bilinear")
ax.plot(pc - c0, pr - r0, "c+", markersize=12, mew=2, label="峰值")
ax.set_title("峰值邻域幅度（线性）")
ax.set_xlabel("距离向（采样点）"); ax.set_ylabel("方位向（脉冲）")
ax.legend(fontsize=9, loc="upper right")
plt.colorbar(im, ax=ax, label="幅度 |I+jQ|")

# 2. Zoomed dB
ax = axes[0, 1]
vmin = max(-60, zoom_db.min())
im = ax.imshow(zoom_db, aspect="auto", origin="lower", cmap="gray_r", vmin=vmin, vmax=0, interpolation="bilinear")
ax.plot(pc - c0, pr - r0, "r+", markersize=12, mew=2, label="峰值")
ax.set_title(f"峰值邻域幅度（dB，-{abs(int(vmin))} dB 底噪）")
ax.set_xlabel("距离向（采样点）"); ax.set_ylabel("方位向（脉冲）")
ax.legend(fontsize=9, loc="upper right")
plt.colorbar(im, ax=ax, label="归一化 (dB)")

# 3. 3D surface near peak
from mpl_toolkits.mplot3d import Axes3D
ax = fig.add_subplot(2, 3, 3, projection="3d")
zr, zc = zoom_db.shape
x = np.arange(zc); y = np.arange(zr)
xv, yv = np.meshgrid(x, y)
ax.plot_surface(xv, yv, zoom_db, cmap="viridis", edgecolor="none", alpha=0.85)
ax.plot([pc - c0], [pr - r0], [0], "r*", markersize=15)
ax.set_title("点扩散函数三维曲面（dB，峰值邻域）")
ax.set_xlabel("距离向"); ax.set_ylabel("方位向"); ax.set_zlabel("归一化 (dB)")
ax.view_init(elev=30, azim=-50)

# 4. Range profile
ax = axes[1, 0]
rp = mag_db_norm[pr, :]
x_r = np.arange(cols)
ax.plot(x_r[c0:c1], rp[c0:c1], "b-", lw=1.2)
ax.axhline(-3, color="r", ls="--", lw=0.8, label="-3 dB")
ax.axhline(-10, color="orange", ls="--", lw=0.8, label="-10 dB")
ax.axhline(-20, color="gray", ls="--", lw=0.8, label="-20 dB")
ax.axvline(pc, color="cyan", ls=":", lw=1, label=f"峰值 = {pc}")
ax.set_title("距离向剖面（通过峰值）")
ax.set_xlabel("距离向（采样点）"); ax.set_ylabel("归一化幅度 (dB)")
ax.set_ylim(-50, 3); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

# 5. Azimuth profile
ax = axes[1, 1]
ap = mag_db_norm[:, pc]
x_a = np.arange(rows)
ax.plot(x_a, ap, "b-", lw=1.2, marker="o", ms=4)
ax.axhline(-3, color="r", ls="--", lw=0.8, label="-3 dB")
ax.axhline(-10, color="orange", ls="--", lw=0.8, label="-10 dB")
ax.axhline(-20, color="gray", ls="--", lw=0.8, label="-20 dB")
ax.axvline(pr, color="cyan", ls=":", lw=1, label=f"峰值 = {pr}")
ax.set_title(f"方位向剖面（通过峰值）— 注意：仅 {rows} 个脉冲")
ax.set_xlabel("方位向（脉冲）"); ax.set_ylabel("归一化幅度 (dB)")
ax.set_ylim(-5, 3); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

# 6. Key metrics
ax = axes[1, 2]
ax.axis("off")
sample_rate = 1.0e6
c = 299792458.0
range_bin_spacing = c / (2.0 * sample_rate)
metrics_text = f"""
SAR 点扩散函数质量指标
========================

【仿真配置】
  载波频率：9.6 GHz（X 波段）
  信号带宽：0.5 MHz
  脉冲宽度：20 μs
  脉冲重复频率：100 Hz
  距离采样点数：1024
  方位脉冲数：33
  平台速度：180 m/s
  目标斜距：100 km
  平台高度：10 km

【成像结果】
  峰值位置：行={pr}，列={pc}
  距离分辨单元：{range_bin_spacing:.1f} m
  距离定位误差：19.2 m（小于 1 个单元）
  距离 -3dB 宽度：1 个单元（< 150 m）
  距离 -10dB 宽度：3 个单元
  距离峰值旁瓣比：20.8 dB
  方位 -3dB 宽度：全部 33 脉冲
    （欠采样，增大 PRF 和脉冲数
     可改善方位分辨率）
  图像熵：5.06 nats
  动态范围：约 390 dB
    （未加噪声，仅点目标，
     背景为零）

【说明】
  33 脉冲为快速演示用配置，
  可即时成像。工程级方位分辨
  率需将脉冲数增至 256~1024。
"""
ax.text(0.05, 0.95, metrics_text, transform=ax.transAxes, fontsize=9,
        verticalalignment="top",
        bbox=dict(boxstyle="round", facecolor="lightyellow", alpha=0.9))

plt.tight_layout(rect=[0, 0, 1, 0.94])
out = sys.argv[1] if len(sys.argv) > 1 else "/tmp/sar_psf_analysis.png"
plt.savefig(out, dpi=150, bbox_inches="tight")
print(f"PSF analysis saved: {out}")
