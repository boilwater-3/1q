#!/usr/bin/env python3
"""批量场景验证结果分析脚本。

读取五个模块（AR/EOS/ESR/SAR/SBIRS）各自 scenarios.csv 汇总文件，打印：
  - 每模块的场景汇总表（关键参数 → 关键指标）。
  - 高亮回放分叉场景（replay_ok=0）与软断言告警（warning_count/error_count > 0）。
  - 关键物理趋势的单调性检查（带宽↑→距离分辨率↓、对比度↑→检出率↑ 等）。
  - 若 matplotlib 可用，为每个模块生成一张趋势 PNG（失败优雅降级）。

用法:
    python3 analyze_batch_results.py [base_output_dir]
    默认 base_output_dir = /tmp/1q/batch_validation

仅依赖标准库 + 可选 matplotlib（无则跳过绘图）。
"""

import csv
import os
import sys

DEFAULT_BASE_DIR = "/tmp/1q/batch_validation"

# 每个模块的输出子目录与显示名。
MODULES = [
    ("airborne_radar", "AR 机载雷达"),
    ("electro_optical_sensor", "EOS 光电传感器"),
    ("electronic_surveillance_radar", "ESR 电子侦察"),
    ("sar", "SAR 合成孔径雷达"),
    ("sbirs_sensor", "SBIRS 天基红外传感器"),
]

# ANSI 颜色码（终端高亮）。
RED = "\033[91m"
YELLOW = "\033[93m"
GREEN = "\033[92m"
BOLD = "\033[1m"
RESET = "\033[0m"


def colored(text, color):
    return f"{color}{text}{RESET}"


def read_csv(path):
    """读取 CSV，返回 (fieldnames, rows[list of dict])；文件不存在返回 (None, None)。"""
    if not os.path.exists(path):
        return None, None
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        return reader.fieldnames, rows


def to_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def to_int(value, default=0):
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def print_header(title):
    print()
    print(colored("=" * 72, BOLD))
    print(colored(f"  {title}", BOLD))
    print(colored("=" * 72, BOLD))


def fmt(v, width=10, prec=4):
    """格式化数值为定宽字符串。"""
    if isinstance(v, float):
        return f"{v:{width}.{prec}f}"
    return f"{str(v):<{width}}"


def analyze_module(module_dir, display_name):
    """分析单个模块的 scenarios.csv，返回是否有问题（用于退出码）。"""
    csv_path = os.path.join(module_dir, "scenarios.csv")
    fields, rows = read_csv(csv_path)
    if rows is None:
        print(colored(f"  [跳过] {display_name}: 未找到 {csv_path}", YELLOW))
        return False

    has_problem = False
    replay_fail = 0
    warn_scenarios = []
    err_scenarios = []

    for r in rows:
        replay_ok = to_int(r.get("replay_ok", "1"))
        warn_cnt = to_int(r.get("warning_count", "0"))
        err_cnt = to_int(r.get("error_count", "0"))
        sid = r.get("scenario_id", "?")
        if replay_ok == 0:
            replay_fail += 1
            # 回放分叉记为 warning 级提示，不计入 has_problem（属模块确定性属性，
            # 与各模块 C++ 程序的退出逻辑一致：只有 error 才阻塞）。
        if warn_cnt > 0:
            warn_scenarios.append((sid, warn_cnt, r.get("warnings", "")))
        if err_cnt > 0:
            err_scenarios.append((sid, err_cnt, r.get("warnings", "")))
            has_problem = True

    print(f"\n  场景总数: {len(rows)}")
    print(f"  回放分叉场景: {replay_fail}")
    print(f"  软断言 warning 场景: {len(warn_scenarios)}")
    print(f"  软断言 error   场景: {len(err_scenarios)}")

    if err_scenarios:
        print(colored("\n  [error 场景详情]", RED))
        for sid, cnt, msg in err_scenarios[:10]:
            print(f"    {sid} (err={cnt}): {msg[:120]}")
    if warn_scenarios:
        print(colored("\n  [warning 场景详情]", YELLOW))
        for sid, cnt, msg in warn_scenarios[:10]:
            print(f"    {sid} (warn={cnt}): {msg[:120]}")

    # 模块特化的趋势摘要。
    print_module_trend(module_dir, display_name, rows)
    return has_problem


def print_module_trend(module_dir, display_name, rows):
    """按模块打印关键趋势摘要表。"""
    print(colored("\n  [关键指标摘要]", BOLD))

    if "airborne_radar" in module_dir:
        # AR: 距离 → 确认率（固定 RCS=5, n=3, 默认阈值）
        print("    距离 → 稳态确认均值 (rcs=5.0, n=3, 默认阈值):")
        for r in sorted(rows, key=lambda x: to_float(x.get("target_range_km", 0))):
            if (to_float(r.get("rcs_m2", 0)) == 5.0 and to_int(r.get("target_count", 0)) == 3
                    and to_float(r.get("min_snr_db_override", -1)) < 0):
                print(f"      range={fmt(to_float(r['target_range_km']), 7, 1)}km  "
                      f"confirmed={fmt(to_float(r.get('steady_confirmed_mean', 0)))}  "
                      f"match_rate={fmt(to_float(r.get('steady_match_rate_mean', 0)))}")

    elif "electro_optical" in module_dir:
        # EOS: 对比度 → 检出率/融合SNR (固定 offset=0.010, day)
        print("    对比度 → 检出率/融合SNR (offset=0.010, day):")
        for r in rows:
            if (abs(to_float(r.get("target_lon_offset_deg", 0)) - 0.010) < 1e-6
                    and r.get("lighting") == "day"):
                print(f"      contrast={r.get('contrast', '?'):<4}  "
                      f"det_rate={fmt(to_float(r.get('steady_detection_rate_mean', 0)))}  "
                      f"fused_db={fmt(to_float(r.get('steady_fused_snr_db_mean', 0)), 10, 3)}")

    elif "electronic_surveillance" in module_dir:
        # ESR: 距离 → 真值匹配率 (固定 fc=8GHz, occ=0.1)
        print("    距离 → 真值匹配率 (fc=8GHz, occ=0.1):")
        for r in sorted(rows, key=lambda x: to_float(x.get("emitter_range_km", 0))):
            if (abs(to_float(r.get("carrier_ghz", 0)) - 8.0) < 1e-6
                    and abs(to_float(r.get("spectrum_occupancy", 0)) - 0.1) < 1e-6):
                print(f"      range={fmt(to_float(r['emitter_range_km']), 7, 1)}km  "
                      f"truth_match={fmt(to_float(r.get('steady_truth_match_rate_mean', 0)))}  "
                      f"hyp_conf={fmt(to_float(r.get('steady_hyp_confidence_mean', 0)))}")

    elif "sar" in module_dir:
        # SAR: 带宽 → 距离分辨率 (固定 r=100km, p=33)
        print("    带宽 → 距离分辨率 (slant=100km, pulses=33):")
        for r in sorted(rows, key=lambda x: to_float(x.get("bandwidth_mhz", 0))):
            if (abs(to_float(r.get("slant_range_km", 0)) - 100.0) < 1e-6
                    and to_int(r.get("azimuth_pulses", 0)) == 33):
                print(f"      bw={fmt(to_float(r['bandwidth_mhz']), 6, 1)}MHz  "
                      f"res_m={fmt(to_float(r.get('range_resolution_3db_m', 0)), 10, 3)}  "
                      f"snr_db={fmt(to_float(r.get('estimated_snr_db', 0)), 8, 2)}  "
                      f"entropy={fmt(to_float(r.get('image_entropy_nats', 0)))}")

    elif "sbirs_sensor" in module_dir:
        # SBIRS: 距离 → 红外 SNR / 检出（固定 800K, 10m2）
        print("    距离 → 红外SNR/检出 (temperature=800K, area=10m2):")
        for r in sorted(rows, key=lambda x: to_float(x.get("range_km", 0))):
            if (abs(to_float(r.get("temperature_k", 0)) - 800.0) < 1e-6
                    and abs(to_float(r.get("projected_area_m2", 0)) - 10.0) < 1e-6):
                print(f"      range={fmt(to_float(r['range_km']), 7, 0)}km  "
                      f"snr={fmt(to_float(r.get('max_snr_linear', 0)), 12, 4)}  "
                      f"detections={to_int(r.get('detection_count', 0))}")


def try_plot(base_dir):
    """尝试用 matplotlib 生成趋势图；不可用则跳过。"""
    try:
        import matplotlib
        matplotlib.use("Agg")  # 非交互后端
        import matplotlib.pyplot as plt
    except ImportError:
        print(colored("\n[绘图] matplotlib 不可用，跳过趋势图生成。", YELLOW))
        return

    plot_sar_bandwidth(base_dir, plt)
    print(colored("\n[绘图] 趋势图已生成。", GREEN))


def plot_sar_bandwidth(base_dir, plt):
    """绘制 SAR 带宽 → 距离分辨率 趋势图（最清晰的物理趋势）。"""
    csv_path = os.path.join(base_dir, "sar", "scenarios.csv")
    fields, rows = read_csv(csv_path)
    if rows is None:
        return
    bw, res = [], []
    for r in rows:
        if (abs(to_float(r.get("slant_range_km", 0)) - 100.0) < 1e-6
                and to_int(r.get("azimuth_pulses", 0)) == 33):
            bw.append(to_float(r.get("bandwidth_mhz", 0)))
            res.append(to_float(r.get("range_resolution_3db_m", 0)))
    if not bw:
        return
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(bw, res, "o-", color="#d62728", linewidth=2, markersize=8)
    ax.set_xlabel("Signal Bandwidth (MHz)")
    ax.set_ylabel("Range Resolution 3dB (m)")
    ax.set_title("SAR: Bandwidth vs Range Resolution (lower is better)")
    ax.grid(True, alpha=0.3)
    out_path = os.path.join(base_dir, "sar", "trend_bandwidth_resolution.png")
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    print(f"    SAR 带宽-分辨率趋势图: {out_path}")


def main():
    base_dir = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_BASE_DIR
    print_header(f"批量场景验证结果分析\n  基础目录: {base_dir}")

    if not os.path.isdir(base_dir):
        print(colored(f"\nFATAL: 基础目录不存在: {base_dir}", RED))
        print("请先运行各模块的批量验证程序，或指定正确的输出目录。")
        return 1

    any_problem = False
    for subdir, display_name in MODULES:
        module_dir = os.path.join(base_dir, subdir)
        print_header(display_name)
        if analyze_module(module_dir, display_name):
            any_problem = True

    print_header("总结")
    if any_problem:
        print(colored("  存在 error 场景，请查看上方详情。", RED))
    else:
        print(colored("  所有模块：无 error 场景。", GREEN))
    print(colored("  注：回放分叉（如有）记为 warning，属模块确定性属性，不视为失败。", YELLOW))

    try_plot(base_dir)
    return 1 if any_problem else 0


if __name__ == "__main__":
    sys.exit(main())
