#!/usr/bin/env python3
"""
SAR 聚焦图像可视化工具。

从 sar_session_usage 示例导出的 raw binary 文件中读取复数聚焦图像，
生成用于分析的出版物级图像显示。

用法:
  python3 tools/visualize_sar_image.py /tmp/sar_focused_image.raw
"""

from __future__ import annotations

import struct
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")  # 无头模式直接保存文件
import matplotlib.pyplot as plt
from matplotlib import colormaps

# 配置中文字体（macOS）
plt.rcParams["font.sans-serif"] = ["Heiti SC", "PingFang SC", "STHeiti", "Arial Unicode MS"]
plt.rcParams["axes.unicode_minus"] = False  # 负号正常显示


def read_sar_raw(path: str) -> tuple[np.ndarray, tuple[int, int]]:
    """读取 SAR 聚焦图像 raw binary 文件。

    文件格式:
      uint32: rows
      uint32: cols
      uint32: has_imag (1=复数, 0=纯实数)
      uint32: peak_row (由 C++ 示例写入的峰值像素坐标)
      uint32: peak_col
      之后是 rows*cols 个 float32 复数对 (real, imag)，行主序。
    """
    with open(path, "rb") as f:
        rows = struct.unpack("<I", f.read(4))[0]
        cols = struct.unpack("<I", f.read(4))[0]
        has_imag = struct.unpack("<I", f.read(4))[0]
        peak_row = struct.unpack("<I", f.read(4))[0]
        peak_col = struct.unpack("<I", f.read(4))[0]

    total_pixels = rows * cols
    dtype = [("real", np.float32), ("imag", np.float32)] if has_imag else np.float32
    # Read binary blob after header
    with open(path, "rb") as f:
        f.seek(5 * 4)  # skip 5 uint32 header
        raw = f.read()
    if has_imag:
        n_floats = total_pixels * 2
        data = np.frombuffer(raw, dtype=np.float32, count=n_floats)
        real = data[0::2].reshape(rows, cols).astype(np.float64)
        imag = data[1::2].reshape(rows, cols).astype(np.float64)
        complex_image = real + 1j * imag
    else:
        complex_image = np.frombuffer(raw, dtype=np.float32, count=total_pixels).reshape(rows, cols).astype(np.float64)
    return complex_image, (peak_row, peak_col)


def plot_sar_image(complex_image: np.ndarray, peak: tuple[int, int],
                   title: str = "SAR 聚焦图像 — 100 km 地面点目标",
                   output_png: str | None = None) -> None:
    """绘制包含幅度图、dB 图、距离/方位切面的四象限 SAR 图像。"""
    rows, cols = complex_image.shape
    magnitude = np.abs(complex_image)
    magnitude_db = 20.0 * np.log10(np.maximum(magnitude, 1e-30))

    # 动态范围裁剪：以峰值为 0dB，向下 50dB
    vmax_db = np.max(magnitude_db)
    vmin_db = vmax_db - 50.0

    pr, pc = peak

    fig = plt.figure(figsize=(18, 12))
    fig.suptitle(title, fontsize=16, fontweight="bold", y=0.98)

    # ---------- 左上：线性幅度 ----------
    ax1 = fig.add_subplot(2, 3, 1)
    im1 = ax1.imshow(magnitude, aspect="auto", origin="lower",
                     cmap="inferno", interpolation="nearest")
    ax1.plot(pc, pr, "c+", markersize=14, markeredgewidth=2, label=f"峰值 ({pr},{pc})")
    ax1.set_title("幅度（线性）", fontsize=13)
    ax1.set_xlabel("距离向（采样点数）")
    ax1.set_ylabel("方位向（脉冲数）")
    ax1.legend(fontsize=9, loc="upper right")
    plt.colorbar(im1, ax=ax1, label="|I+jQ| 幅度")

    # ---------- 中上：对数幅度 (dB) ----------
    ax2 = fig.add_subplot(2, 3, 2)
    im2 = ax2.imshow(magnitude_db, aspect="auto", origin="lower",
                     cmap="gray", vmin=vmin_db, vmax=vmax_db, interpolation="nearest")
    ax2.plot(pc, pr, "r+", markersize=14, markeredgewidth=2, label=f"峰值 ({pr},{pc})")
    ax2.set_title("幅度（dB）— 动态范围 50 dB", fontsize=13)
    ax2.set_xlabel("距离向（采样点数）")
    ax2.set_ylabel("方位向（脉冲数）")
    ax2.legend(fontsize=9, loc="upper right")
    plt.colorbar(im2, ax=ax2, label="分贝 (dB)")

    # ---------- 右上：伪彩色增强 ----------
    ax3 = fig.add_subplot(2, 3, 3)
    im3 = ax3.imshow(magnitude_db, aspect="auto", origin="lower",
                     cmap="hot", vmin=vmin_db, vmax=vmax_db, interpolation="bilinear")
    ax3.plot(pc, pr, "c+", markersize=14, markeredgewidth=2, label=f"峰值 ({pr},{pc})")
    ax3.set_title("幅度（dB）— 伪彩色 + 双线性插值", fontsize=13)
    ax3.set_xlabel("距离向（采样点数）")
    ax3.set_ylabel("方位向（脉冲数）")
    ax3.legend(fontsize=9, loc="upper right")
    plt.colorbar(im3, ax=ax3, label="分贝 (dB)")

    # ---------- 左下：距离向切面 (通过峰值) ----------
    ax4 = fig.add_subplot(2, 3, 4)
    range_profile_db = 20.0 * np.log10(np.maximum(magnitude[pr, :], 1e-30))
    range_profile_db -= np.max(range_profile_db)
    x_axis_pixels = np.arange(cols, dtype=np.float64)
    ax4.plot(x_axis_pixels, range_profile_db, "b-", linewidth=0.8)
    ax4.axvline(pc, color="r", linestyle="--", linewidth=1, label=f"峰值列 = {pc}")
    ax4.set_title("距离向切面（方位向通过峰值）", fontsize=13)
    ax4.set_xlabel("距离向（采样点数）")
    ax4.set_ylabel("归一化幅度 (dB)")
    ax4.set_ylim(-50, 3)
    ax4.grid(True, alpha=0.3)
    ax4.legend(fontsize=9)

    # 标注 -3dB 宽度
    above_3db = range_profile_db > -3.0
    if np.any(above_3db):
        indices = np.where(above_3db)[0]
        width_3db = indices[-1] - indices[0] + 1
        ax4.annotate(f"-3dB 宽度 ≈ {width_3db} 个采样点",
                     xy=(pc, -3.0), xytext=(pc + 50, -6),
                     arrowprops=dict(arrowstyle="->", color="gray"),
                     fontsize=10, color="darkred")

    # ---------- 中下：方位向切面 (通过峰值) ----------
    ax5 = fig.add_subplot(2, 3, 5)
    az_profile_db = 20.0 * np.log10(np.maximum(magnitude[:, pc], 1e-30))
    az_profile_db -= np.max(az_profile_db)
    y_axis_pixels = np.arange(rows, dtype=np.float64)
    ax5.plot(y_axis_pixels, az_profile_db, "b-", linewidth=0.8)
    ax5.axvline(pr, color="r", linestyle="--", linewidth=1, label=f"峰值行 = {pr}")
    ax5.set_title("方位向切面（距离向通过峰值）", fontsize=13)
    ax5.set_xlabel("方位向（脉冲数）")
    ax5.set_ylabel("归一化幅度 (dB)")
    ax5.set_ylim(-50, 3)
    ax5.grid(True, alpha=0.3)
    ax5.legend(fontsize=9)

    # 标注 -3dB 宽度
    above_3db_az = az_profile_db > -3.0
    if np.any(above_3db_az):
        indices = np.where(above_3db_az)[0]
        width_3db_az = indices[-1] - indices[0] + 1
        ax5.annotate(f"-3dB 宽度 ≈ {width_3db_az} 个脉冲",
                     xy=(pr, -3.0), xytext=(pr + 5, -8),
                     arrowprops=dict(arrowstyle="->", color="gray"),
                     fontsize=10, color="darkred")

    # ---------- 右下：3D 曲面 ----------
    ax6 = fig.add_subplot(2, 3, 6, projection="3d")
    # 下采样以控制渲染性能（保留峰值周围的完整区域）
    ds_factor = max(1, min(rows, cols) // 80)
    mag_ds = magnitude[::ds_factor, ::ds_factor]
    dr, dc = mag_ds.shape
    x = np.arange(dc)
    y = np.arange(dr)
    xv, yv = np.meshgrid(x, y)
    ax6.plot_surface(xv, yv, mag_ds, cmap="viridis", edgecolor="none", alpha=0.85)
    ax6.set_title("三维幅度曲面（下采样）", fontsize=13)
    ax6.set_xlabel("距离向")
    ax6.set_ylabel("方位向")
    ax6.set_zlabel("|I+jQ| 幅度")
    ax6.view_init(elev=38, azim=-55)

    plt.tight_layout(rect=[0, 0, 1, 0.95])

    if output_png:
        plt.savefig(output_png, dpi=150, bbox_inches="tight")
        print(f"图像已保存: {output_png}")

    plt.show()


def print_statistics(complex_image: np.ndarray, peak: tuple[int, int]) -> None:
    """输出聚焦图像质量统计。"""
    magnitude = np.abs(complex_image)
    magnitude_db = 20.0 * np.log10(np.maximum(magnitude, 1e-30))
    pr, pc = peak

    peak_val = magnitude[pr, pc]
    peak_db = magnitude_db[pr, pc]

    # PSLR: 峰值旁瓣比
    range_profile = magnitude[pr, :]
    az_profile = magnitude[:, pc]

    mainlobe_half = 5  # 主瓣半宽（排除峰值周围 ±5 bins）
    r_left = max(0, pc - mainlobe_half)
    r_right = min(magnitude.shape[1], pc + mainlobe_half + 1)
    range_sidelobes = np.concatenate([range_profile[:r_left], range_profile[r_right:]])
    psr_range = 20.0 * np.log10(peak_val / np.max(range_sidelobes)) if len(range_sidelobes) > 0 else np.inf

    a_top = max(0, pr - mainlobe_half)
    a_bottom = min(magnitude.shape[0], pr + mainlobe_half + 1)
    az_sidelobes = np.concatenate([az_profile[:a_top], az_profile[a_bottom:]])
    psr_az = 20.0 * np.log10(peak_val / np.max(az_sidelobes)) if len(az_sidelobes) > 0 else np.inf

    print(f"""
=== SAR 聚焦图像统计 ===
图像尺寸:        {magnitude.shape[0]} 行 × {magnitude.shape[1]} 列
峰值位置:        ({pr}, {pc})
峰值幅度:        {peak_val:.6e} ({peak_db:.2f} dB)
动态范围:        {np.max(magnitude_db) - np.min(magnitude_db):.1f} dB
距离向 PSLR:     {psr_range:.2f} dB
方位向 PSLR:     {psr_az:.2f} dB
图像熵:          {np.sum(magnitude * np.log(np.maximum(magnitude, 1e-30))):.4f} nats
""")


def main() -> None:
    if len(sys.argv) < 2:
        path = "/tmp/sar_focused_image.raw"
        print(f"未指定文件路径，使用默认: {path}")
    else:
        path = sys.argv[1]

    complex_image, peak = read_sar_raw(path)
    print_statistics(complex_image, peak)

    output_png = sys.argv[2] if len(sys.argv) > 2 else None
    plot_sar_image(complex_image, peak, output_png=output_png)


if __name__ == "__main__":
    main()
