#!/usr/bin/env python3
"""fd_trace_to_viz.py — FD 轨迹 CSV → 统一可视化契约（build_viewer.py 可直接消费）。

用法：
  python3 fd_trace_to_viz.py <fd_trace.csv> [--out <dir>] [--aircraft-id N] [--sample-sec 1.0]

输入：tests/unit/flight_dynamic/fd_tools 各 *_trace_csv 工具导出的逐步轨迹 CSV（WGS84
弧度制 LLA + 米制高度；0.01 s 步进），支持三种列格式：
  - 13 列（orbit/racetrack_trace_csv）：sim_time_sec,lat_rad,lon_rad,alt_m,...,
    heading_rad,...,vtrue_mps,...
  - 11 列（sturn_trace_csv）：sim_time_sec,lat_rad,lon_rad,alt_m,...,heading_deg,...,
    vtrue_mps（heading 为度）
  - 7 列（gtest AircraftManeuverTest dump）：time_sec,lat_rad,lon_rad,alt_m,
    pitch_rad,roll_rad,heading_rad（无速度列）

输出：<dir>/platform_track.csv（统一契约 v2：度制 LLA、aircraft_id 列、按
--sample-sec 抽行、cycle = 抽样序号）+ header-only 的 target_truth/route_plan
（契约核心文件，--check 表头校验通过；传感器文件缺省跳过）。生成后用
examples/common/viz/build_viewer.py <dir> 即可把 FD 单机轨迹拖进共享查看器。

分工说明：plot_maneuvers.py / orbit_visualize.py（matplotlib 静态图）保留为
机动质量验证用途（误差/高度剖面）；本脚本只做数据契约归一，不替代质量图。
"""

import argparse
import csv
import math
import os
import sys

RAD_TO_DEG = 180.0 / math.pi

PLATFORM_HEADER = ("cycle,t_sec,aircraft_id,lat_deg,lon_deg,alt_m,heading_deg,"
                   "speed_mps,wp_index,wp_count,model")
TRUTH_HEADER = "cycle,t_sec,target_id,entity_type,lat_deg,lon_deg,alt_m,rcs"
ROUTE_HEADER = "aircraft_id,index,lat_deg,lon_deg,alt_m,speed_mps,radius_m"


def f(row, key):
    """取行字段为 float；缺失/空字段返回 None。"""
    v = row.get(key)
    if v is None or v == "":
        return None
    try:
        return float(v)
    except ValueError:
        return None


def main():
    parser = argparse.ArgumentParser(
        description="FD 轨迹 CSV → 统一可视化契约（platform_track.csv 度制）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    parser.add_argument("trace_csv", help="FD 逐步轨迹 CSV（13/11/7 列格式之一）")
    parser.add_argument("--out", default=None,
                        help="输出目录（默认 = 输入文件同名目录下的 <basename>_viz/）")
    parser.add_argument("--aircraft-id", type=int, default=1,
                        help="aircraft_id（默认 1；多机合并时用于区分各机轨迹）")
    parser.add_argument("--sample-sec", type=float, default=1.0,
                        help="抽样间隔（s，默认 1.0；FD 为 0.01 s 步进，按间隔取最近行）")
    args = parser.parse_args()

    if not os.path.exists(args.trace_csv):
        sys.stderr.write("错误：找不到轨迹文件 %s\n" % args.trace_csv)
        return 1
    if args.sample_sec <= 0.0:
        sys.stderr.write("错误：--sample-sec 必须 > 0\n")
        return 1

    with open(args.trace_csv, newline="") as fh:
        reader = csv.DictReader(fh)
        fields = reader.fieldnames or []
        rows = list(reader)
    if not rows:
        sys.stderr.write("错误：%s 无数据行\n" % args.trace_csv)
        return 1

    # 列格式识别（统一到 lat/lon/alt/heading/speed 语义）。
    if "lat_rad" not in fields or "lon_rad" not in fields or "alt_m" not in fields:
        sys.stderr.write("错误：%s 缺少 lat_rad/lon_rad/alt_m 列（FD 轨迹 CSV 格式？）\n"
                         % args.trace_csv)
        return 1
    t_key = "sim_time_sec" if "sim_time_sec" in fields else "time_sec"
    if "heading_rad" in fields:
        hdg_is_deg = False
    elif "heading_deg" in fields:
        hdg_is_deg = True
    else:
        sys.stderr.write("错误：%s 缺少 heading_rad/heading_deg 列\n" % args.trace_csv)
        return 1
    spd_key = "vtrue_mps" if "vtrue_mps" in fields else None

    # 按 --sample-sec 抽样（取 ≥ 下一采样时刻的首行；t 单调递增）。
    sampled = []
    next_t = 0.0
    for r in rows:
        t = f(r, t_key)
        if t is None:
            continue
        if t < next_t:
            continue
        sampled.append((t, r))
        next_t += args.sample_sec

    out_dir = args.out or os.path.join(
        os.path.dirname(os.path.abspath(args.trace_csv)),
        os.path.splitext(os.path.basename(args.trace_csv))[0] + "_viz")
    os.makedirs(out_dir, exist_ok=True)

    plat_path = os.path.join(out_dir, "platform_track.csv")
    with open(plat_path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(PLATFORM_HEADER.split(","))
        for i, (t, r) in enumerate(sampled, start=1):
            lat_deg = f(r, "lat_rad") * RAD_TO_DEG
            lon_deg = f(r, "lon_rad") * RAD_TO_DEG
            alt = f(r, "alt_m")
            hdg = f(r, "heading_rad") if not hdg_is_deg else f(r, "heading_deg")
            if hdg is not None and not hdg_is_deg:
                hdg *= RAD_TO_DEG
            spd = f(r, spd_key) if spd_key else None
            row = [i, "%.3f" % t, args.aircraft_id,
                   "%.7f" % lat_deg, "%.7f" % lon_deg,
                   "%.2f" % alt if alt is not None else "",
                   "%.2f" % hdg if hdg is not None else "",
                   "%.2f" % spd if spd is not None else "",
                   "", "", "jsbsim"]
            writer.writerow(row)

    # 契约核心文件 header-only（--check 表头校验通过；无航路/目标数据）。
    for name, header in (("target_truth.csv", TRUTH_HEADER),
                         ("route_plan.csv", ROUTE_HEADER)):
        with open(os.path.join(out_dir, name), "w", newline="") as fh:
            fh.write(header + "\n")

    print("已转换：%s → %s" % (args.trace_csv, plat_path))
    print("  抽样 %d 行（间隔 %.2f s，aircraft_id=%d）" %
          (len(sampled), args.sample_sec, args.aircraft_id))
    print("  查看器：python3 examples/common/viz/build_viewer.py %s" % out_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
