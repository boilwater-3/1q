#!/usr/bin/env python3
"""
sbirs_orbit_viewer.py — SBIRS 卫星场景三维可视化查看器。

从卫星场景可执行（sbirs_triple_sat_fix_messages）落盘的 CSV 构建单文件
自包含 HTML（vanilla JS + canvas 3D 投影，无 CDN、离线可用），还原：

  地球（线框球，遮挡判定同款半径）→ 三颗 GEO 卫星（星下点凝视视场锥）→
  每星逐目标视线状态（检出画视线；被地球遮挡的目标标出遮挡深度）→
  融合航迹 → 双星交会点（由两星测角视线几何交会复算，与库交会误差对照）。

用法：
  python3 sbirs_orbit_viewer.py <log_dir> [--out viewer.html]
  python3 sbirs_orbit_viewer.py --check <log_dir>   # 几何复算校验（不生成 HTML）

<log_dir> 为场景输出目录（examples/log/sbirs_triple_sat_fix_messages/），
须含场景 exe 落盘的五份 CSV：
  sbirs_sats.csv      —— 卫星静态几何（ECEF + 视场角 + ECI↔ECEF 旋转角 gmst）
  sbirs_truth.csv     —— 目标真值逐周期 ECEF 轨迹
  sbirs_los.csv       —— 每周期每星逐目标视线状态/测角/SNR（库调试视图快照）
  sbirs_fused.csv     —— 融合航迹（LLA 后验已由 exe 转 ECEF）
  sbirs_dual_fix.csv  —— 双星交会误差样本（位置误差/视线残差/斜距误差）

几何复算口径与库一致：测角 az=atan2(y,x)、el=asin(z)（ECI 系）；ECI→ECEF
绕 Z 转 −gmst；地球遮挡球半径 6371 km（视线到地心最近距离 ≤ 半径即遮挡）。
交会点为两条测角视线（ECEF 直线）最近点对的中点，属查看器端复算展示；
库交会误差以 sbirs_dual_fix.csv 为准，两者一致性由 --check 校验封口。

仅依赖 Python 标准库。
"""

import argparse
import csv
import html
import json
import math
import os
import sys

EARTH_RADIUS_M = 6371000.0     # 库 EarthOccultation 遮挡判定同款（kMeanEarthRadiusM）
AZ_EL_TOL_DEG = 0.05           # 测角复算 vs 库测角容差（姿态噪声 1σ≈0.01°，取 5σ）

SAT_PALETTE = ["#1f77b4", "#ff7f0e", "#2ca02c", "#9467bd", "#8c564b",
               "#e377c2", "#7f7f7f", "#bcbd22", "#17becf", "#d62728"]
TARGET_PALETTE = ["#d62728", "#9467bd", "#2ca02c", "#8c564b", "#e377c2"]
COLOR_FUSED = "#e377c2"
COLOR_FIX = "#d62728"
COLOR_EARTH_GRID = "#3d6a8f"
COLOR_EARTH_BACK = "#1c2f42"
COLOR_EARTH_FILL = "#0d1f33"

CSV_HEADERS = {
    "sbirs_sats.csv": ("sat_id,source_id,ecef_x_m,ecef_y_m,ecef_z_m,gmst_rad,"
                       "scan_center_az_deg,scan_center_el_deg,wfov_az_deg,wfov_el_deg,"
                       "nfov_az_deg,nfov_el_deg"),
    "sbirs_truth.csv": "cycle,t_sec,target_id,ecef_x_m,ecef_y_m,ecef_z_m",
    "sbirs_los.csv": ("cycle,t_sec,source_id,target_id,present_in_input,status,"
                      "az_rad,el_rad,snr_linear,estimated_range_m"),
    "sbirs_fused.csv": ("cycle,t_sec,key,lifecycle,confidence,has_position,"
                        "ecef_x_m,ecef_y_m,ecef_z_m,channels"),
    "sbirs_dual_fix.csv": ("cycle,t_sec,key,position_error_m,los_residual_m,"
                           "slant_range_error_m"),
}


# ═══════════════════════════════════════════════════════════════════
# CSV 读取与向量小工具
# ═══════════════════════════════════════════════════════════════════

def load_csv(path):
    """读取 CSV 为 dict 列表；文件缺失返回 None。"""
    try:
        with open(path, newline="") as fh:
            return list(csv.DictReader(fh))
    except OSError:
        return None


def f(row, key):
    """取行字段为 float；缺失/空字段返回 None。"""
    v = row.get(key)
    if v is None or v == "":
        return None
    try:
        return float(v)
    except ValueError:
        return None


def to_int(row, key):
    """取行字段为 int；缺失/空字段返回 None。"""
    v = row.get(key)
    if v is None or v == "":
        return None
    try:
        return int(float(v))
    except ValueError:
        return None


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    n = norm(a)
    if n <= 0.0:
        return (0.0, 0.0, 0.0)
    return (a[0] / n, a[1] / n, a[2] / n)


def add_scaled(p, d, s):
    return (p[0] + d[0] * s, p[1] + d[1] * s, p[2] + d[2] * s)


# ═══════════════════════════════════════════════════════════════════
# 几何复算（口径同库：ECI 极坐标 / GMST 旋转 / 球模型遮挡）
# ═══════════════════════════════════════════════════════════════════

def ecef_dir_to_eci_az_el(direction_ecef, gmst_rad):
    """ECEF 方向 → ECI 极坐标 az/el（rad）。ECI = Rz(+gmst)·ECEF。"""
    c, s = math.cos(gmst_rad), math.sin(gmst_rad)
    x = c * direction_ecef[0] - s * direction_ecef[1]
    y = s * direction_ecef[0] + c * direction_ecef[1]
    z = direction_ecef[2]
    az = math.atan2(y, x)
    el = math.asin(max(-1.0, min(1.0, z / max(norm((x, y, z)), 1e-12))))
    return az, el


def bearing_to_ecef_dir(az_rad, el_rad, gmst_rad):
    """ECI 极坐标测角 → ECEF 单位视线方向。ECI→ECEF = Rz(−gmst)。"""
    dx = math.cos(el_rad) * math.cos(az_rad)
    dy = math.cos(el_rad) * math.sin(az_rad)
    dz = math.sin(el_rad)
    c, s = math.cos(gmst_rad), math.sin(gmst_rad)
    return (c * dx + s * dy, -s * dx + c * dy, dz)


def occult_margin_m(observer_ecef, target_ecef):
    """视线被地球遮挡的余量（m，负值 = 挡入深度）。公式同库 EarthOccultation。"""
    los = sub(target_ecef, observer_ecef)
    rng = norm(los)
    if rng <= 0.0:
        return EARTH_RADIUS_M
    u = unit(los)
    s_closest = -dot(observer_ecef, u)
    if s_closest <= 0.0 or s_closest >= rng:
        return EARTH_RADIUS_M
    closest_sq = dot(observer_ecef, observer_ecef) - s_closest * s_closest
    return math.sqrt(max(closest_sq, 0.0)) - EARTH_RADIUS_M


def closest_point_of_lines(p1, d1, p2, d2):
    """两条直线（单位方向）最近点对；近平行返回 None。"""
    b = dot(d1, d2)
    denom = 1.0 - b * b
    if abs(denom) < 1e-9:
        return None
    w0 = sub(p1, p2)
    d_ = dot(d1, w0)
    e = dot(d2, w0)
    s = (b * e - d_) / denom
    t = (e - b * d_) / denom
    return add_scaled(p1, d1, s), add_scaled(p2, d2, t)


# ═══════════════════════════════════════════════════════════════════
# 数据装配
# ═══════════════════════════════════════════════════════════════════

def build_data(log_dir):
    """读取五份 CSV，装配查看器 JSON（含交会点复算）。缺失文件抛 RuntimeError。"""
    rows_by_name = {}
    for name in CSV_HEADERS:
        rows = load_csv(log_dir.rstrip("/") + "/" + name)
        if rows is None:
            raise RuntimeError("%s 缺失（先跑场景 exe sbirs_triple_sat_fix_messages）" % name)
        rows_by_name[name] = rows

    sats_rows = rows_by_name["sbirs_sats.csv"]
    if not sats_rows:
        raise RuntimeError("sbirs_sats.csv 为空")
    gmst = f(sats_rows[0], "gmst_rad")
    sats = []
    for i, r in enumerate(sats_rows):
        sats.append({
            "id": r["sat_id"], "sid": to_int(r, "source_id"),
            "p": [f(r, "ecef_x_m") / 1000.0, f(r, "ecef_y_m") / 1000.0,
                  f(r, "ecef_z_m") / 1000.0],
            "scanAz": f(r, "scan_center_az_deg"), "scanEl": f(r, "scan_center_el_deg"),
            "wfovAz": f(r, "wfov_az_deg"), "wfovEl": f(r, "wfov_el_deg"),
            "nfovAz": f(r, "nfov_az_deg"), "nfovEl": f(r, "nfov_el_deg"),
            "color": SAT_PALETTE[i % len(SAT_PALETTE)],
        })

    cycles = 0
    truth = {}
    for r in rows_by_name["sbirs_truth.csv"]:
        c = to_int(r, "cycle")
        cycles = max(cycles, c)
        key = str(to_int(r, "target_id"))
        truth.setdefault(key, []).append({
            "c": c, "t": f(r, "t_sec"),
            "p": [f(r, "ecef_x_m") / 1000.0, f(r, "ecef_y_m") / 1000.0,
                  f(r, "ecef_z_m") / 1000.0],
        })
    truth_by_cycle_key = {}
    for key, pts in truth.items():
        for p in pts:
            truth_by_cycle_key[(p["c"], key)] = p["p"]

    los = []
    los_by_cst = {}
    for r in rows_by_name["sbirs_los.csv"]:
        c = to_int(r, "cycle")
        sid = to_int(r, "source_id")
        tid = str(to_int(r, "target_id"))
        entry = {
            "c": c, "sid": sid, "tid": tid,
            "present": to_int(r, "present_in_input") == 1,
            "status": r["status"], "az": f(r, "az_rad"), "el": f(r, "el_rad"),
            "snr": f(r, "snr_linear"), "rng": f(r, "estimated_range_m"),
        }
        los.append(entry)
        los_by_cst[(c, sid, tid)] = entry

    fused = []
    for r in rows_by_name["sbirs_fused.csv"]:
        has_pos = to_int(r, "has_position") == 1
        fused.append({
            "c": to_int(r, "cycle"), "key": to_int(r, "key"),
            "lc": r["lifecycle"], "conf": f(r, "confidence"),
            "p": [f(r, "ecef_x_m") / 1000.0, f(r, "ecef_y_m") / 1000.0,
                  f(r, "ecef_z_m") / 1000.0] if has_pos else None,
            "ch": r["channels"],
        })

    # 交会复算：评估星对 = sats[0]/sats[1]（库 API 只吃前两颗）。对每周期每目标，
    # 取两星 detected 的测角视线（ECEF 直线），最近点对中点即交会位置；
    # 与该周期真值的距离应与库 dual_sat 样本一致（--check 封口）。
    sat_by_sid = {s["sid"]: s for s in sats}
    fix = []
    for r in rows_by_name["sbirs_dual_fix.csv"]:
        c = to_int(r, "cycle")
        key = str(to_int(r, "key"))
        entry = {
            "c": c, "key": key,
            "err": f(r, "position_error_m"), "resid": f(r, "los_residual_m"),
            "srerr": f(r, "slant_range_error_m"),
            "p": None, "recalcErr": None,
        }
        los_a = los_by_cst.get((c, sats[0]["sid"], key))
        los_b = los_by_cst.get((c, sats[1]["sid"], key))
        if los_a and los_b and los_a["az"] is not None and los_b["az"] is not None:
            da = bearing_to_ecef_dir(los_a["az"], los_a["el"], gmst)
            db = bearing_to_ecef_dir(los_b["az"], los_b["el"], gmst)
            pair = closest_point_of_lines(sat_by_sid[sats[0]["sid"]]["p"],
                                          da, sat_by_sid[sats[1]["sid"]]["p"], db)
            if pair:
                mid = ((pair[0][0] + pair[1][0]) / 2.0, (pair[0][1] + pair[1][1]) / 2.0,
                       (pair[0][2] + pair[1][2]) / 2.0)
                entry["p"] = [mid[0], mid[1], mid[2]]
                truth_p = truth_by_cycle_key.get((c, key))
                if truth_p:
                    entry["recalcErr"] = norm(sub(mid, truth_p)) * 1000.0
        fix.append(entry)

    # 每星每目标的遮挡余量（几何复算，供状态面板与被挡标注；仅真值在场目标）。
    # 位置换算回米（数据装配统一 km 展示口径），公式按米计算后除回 km。
    for e in los:
        if not e["present"]:
            e["margin"] = None
            continue
        truth_p = truth_by_cycle_key.get((e["c"], e["tid"]))
        if truth_p is None or e["sid"] not in sat_by_sid:
            e["margin"] = None
            continue
        sat_p_m = [c * 1000.0 for c in sat_by_sid[e["sid"]]["p"]]
        truth_p_m = [c * 1000.0 for c in truth_p]
        e["margin"] = occult_margin_m(sat_p_m, truth_p_m) / 1000.0

    meta = {
        "cycles": cycles,
        "gmst": gmst,
        "earthR": EARTH_RADIUS_M / 1000.0,
        "evalPair": [sats[0]["sid"], sats[1]["sid"]],
    }
    return {"meta": meta, "sats": sats, "truth": truth, "los": los, "fused": fused,
            "fix": fix}


# ═══════════════════════════════════════════════════════════════════
# --check 几何复算校验
# ═══════════════════════════════════════════════════════════════════

def wrap_deg(diff_deg):
    """角度差 wrap 到 (−180, 180]。"""
    return (diff_deg + 180.0) % 360.0 - 180.0


def validate(log_dir):
    """三类回归封口：表头契约 / 测角与遮挡复算一致 / 交会复算与库误差一致。"""
    problems = []
    warnings = []
    for name, expected in CSV_HEADERS.items():
        path = log_dir.rstrip("/") + "/" + name
        if not os.path.exists(path):
            problems.append("%s 缺失" % name)
            continue
        with open(path, newline="") as fh:
            header = fh.readline().strip()
        if header != expected:
            problems.append("%s 表头不匹配：%s" % (name, header))
    if problems:
        return problems, warnings

    data = build_data(log_dir)
    meta = data["meta"]
    if not meta["cycles"]:
        problems.append("sbirs_truth.csv 为空")
        return problems, warnings

    sat_by_sid = {s["sid"]: s for s in data["sats"]}
    truth_by_cycle_key = {}
    for key, pts in data["truth"].items():
        for p in pts:
            truth_by_cycle_key[(p["c"], key)] = p["p"]

    # 1) 测角复算：detected 行的库 az/el vs 卫星→真值几何视线（容差覆盖姿态噪声）。
    n_checked = 0
    max_az_err = max_el_err = 0.0
    for e in data["los"]:
        if e["status"] != "detected" or e["az"] is None:
            continue
        truth_p = truth_by_cycle_key.get((e["c"], e["tid"]))
        sat = sat_by_sid.get(e["sid"])
        if truth_p is None or sat is None:
            continue
        los_dir = unit(sub(truth_p, sat["p"]))
        az_geo, el_geo = ecef_dir_to_eci_az_el(los_dir, meta["gmst"])
        az_err = abs(wrap_deg(math.degrees(az_geo - e["az"])))
        el_err = abs(math.degrees(el_geo - e["el"]))
        n_checked += 1
        max_az_err = max(max_az_err, az_err)
        max_el_err = max(max_el_err, el_err)
        if az_err > AZ_EL_TOL_DEG or el_err > AZ_EL_TOL_DEG:
            problems.append(
                "cycle %d 星 %d 目标 %s 测角复算超差：Δaz=%.4f° Δel=%.4f°（容差 %.2f°）"
                % (e["c"], e["sid"], e["tid"], az_err, el_err, AZ_EL_TOL_DEG))
    if n_checked == 0:
        problems.append("无 detected 视线样本可校验测角")

    # 2) 遮挡复算：几何被挡（margin<0）的目标不得 detected；
    #    detected 且 margin>0 为正常，几何被挡的应 not_in_output。
    n_occult = 0
    for e in data["los"]:
        if e["margin"] is None:
            continue
        if e["margin"] < 0.0:
            n_occult += 1
            if e["status"] == "detected":
                problems.append(
                    "cycle %d 星 %d 目标 %s：几何被地球遮挡（margin=%.1f km）却检出"
                    % (e["c"], e["sid"], e["tid"], e["margin"]))
    if n_occult == 0:
        warnings.append("全程无被遮挡视线样本（三星场景应存在地球遮挡行）")

    # 3) 交会复算：最近点中点 vs 库交会位置误差（容差吸收交会对小测角噪声的放大）。
    n_fix = 0
    fix_diffs = []
    for fx in data["fix"]:
        if fx["p"] is None or fx["recalcErr"] is None:
            continue
        n_fix += 1
        diff = abs(fx["recalcErr"] - fx["err"])
        fix_diffs.append(diff)
        tol = max(0.15 * fx["err"], 500.0)
        if diff > tol:
            problems.append(
                "cycle %d 目标 %s 交会复算 vs 库误差超差：复算 %.1f m / 库 %.1f m（容差 %.1f m）"
                % (fx["c"], fx["key"], fx["recalcErr"], fx["err"], tol))
    if n_fix == 0:
        warnings.append("无可复算的交会样本（需两评估星同周期 detected）")

    if fix_diffs:
        warnings.append("交会复算对照：最大偏差 %.1f m（%d 样本）"
                        % (max(fix_diffs), n_fix))
    if n_checked:
        warnings.append("测角复算对照：max Δaz=%.4f° Δel=%.4f°（%d 样本）"
                        % (max_az_err, max_el_err, n_checked))
    return problems, warnings


# ═══════════════════════════════════════════════════════════════════
# HTML 模板（vanilla JS + canvas 3D 投影，自包含离线）
# ═══════════════════════════════════════════════════════════════════

HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>SBIRS 卫星场景三维可视化 — {title}</title>
<style>
  :root {{
    --bg:#10151c; --panel:#171e27; --ink:#dce4ee; --muted:#8393a8;
    --line:#242e3b;
  }}
  * {{ box-sizing:border-box; }}
  body {{ margin:0; padding:14px 18px; background:var(--bg); color:var(--ink);
         font:14px/1.5 -apple-system,"PingFang SC","Helvetica Neue",Arial,sans-serif; }}
  h1 {{ font-size:17px; margin:0 0 3px; }}
  .sub {{ color:var(--muted); font-size:12px; margin-bottom:10px; }}
  .panel {{ background:var(--panel); border:1px solid var(--line); border-radius:8px;
           padding:10px 12px; margin-bottom:12px; }}
  .panel h2 {{ font-size:13px; margin:0 0 6px; color:#aebdd2; font-weight:600; }}
  #controls {{ display:flex; flex-wrap:wrap; gap:12px; align-items:center; }}
  #controls label {{ font-size:12.5px; color:var(--muted); }}
  .btn {{ padding:3px 12px; border:1px solid #3a4757; border-radius:5px; background:#1d2632;
         color:var(--ink); cursor:pointer; font-size:12.5px; }}
  .btn:hover {{ background:#26313f; }}
  .btn.active {{ background:#2b6cb0; border-color:#2b6cb0; color:#fff; }}
  #cycleSlider {{ width:360px; }}
  #cycleReadout {{ font-variant-numeric:tabular-nums; min-width:110px; font-size:12.5px; }}
  .layers {{ display:flex; flex-wrap:wrap; gap:8px; }}
  .layers label {{ display:flex; align-items:center; gap:3px; cursor:pointer; font-size:12.5px; }}
  .swatch {{ width:10px; height:10px; border-radius:2px; display:inline-block; }}
  #stage {{ position:relative; }}
  canvas {{ display:block; border-radius:6px; background:#0a0f16; cursor:grab; }}
  canvas:active {{ cursor:grabbing; }}
  .tooltip {{ position:fixed; display:none; pointer-events:none; background:rgba(15,20,27,.95);
             color:#e8eef6; font-size:12px; padding:6px 9px; border-radius:5px; z-index:50;
             max-width:340px; white-space:pre-line; border:1px solid #33404f; }}
  #bottom {{ display:flex; gap:12px; align-items:stretch; }}
  #bottom .panel {{ flex:1; margin-bottom:0; }}
  table.los {{ border-collapse:collapse; font-size:12px; width:100%; }}
  table.los th, table.los td {{ text-align:left; padding:3px 8px 3px 0;
                               border-bottom:1px solid var(--line); color:var(--ink); }}
  table.los th {{ color:var(--muted); font-weight:500; }}
  .occult {{ color:#e0705a; }}
  .det {{ color:#63c98a; }}
  svg {{ display:block; width:100%; height:auto; }}
  .hint {{ color:var(--muted); font-size:11.5px; margin-top:4px; }}
</style>
</head>
<body>
<h1>SBIRS 卫星场景三维可视化</h1>
<div class="sub">{sub_line}</div>

<div class="panel">
  <div id="controls">
    <button id="playBtn" class="btn">▶ 播放</button>
    <label>速度
      <select id="speedSel">
        <option value="1">1 周期/s</option>
        <option value="2" selected>2 周期/s</option>
        <option value="5">5 周期/s</option>
        <option value="10">10 周期/s</option>
      </select>
    </label>
    <input id="cycleSlider" type="range" min="1" max="{cycles}" value="1">
    <span id="cycleReadout">cycle 1 / {cycles}</span>
    <span class="layers" id="layerToggles"></span>
  </div>
  <div class="hint">视角：拖拽旋转 · 滚轮缩放 · 预设
    <button class="btn" id="viewGlobal">全景</button>
    <button class="btn" id="viewSatA">A 星后方</button>
    <button class="btn" id="viewT1">目标 1 区域</button>
    （单位 km，ECEF 地固坐标系；地球为 {earth_r} km 线框球 = 库遮挡判定同款半径）
  </div>
</div>

<div class="panel" id="stage">
  <canvas id="cv3d" width="{canvas_w}" height="{canvas_h}"></canvas>
</div>

<div id="bottom">
  <div class="panel">
    <h2>当前周期视线状态（每星 × 每目标）</h2>
    <table class="los" id="losTable"></table>
    <div class="hint">「被挡深度」为几何复算：星→目标视线到地心最近距离 − 地球半径，
      负值即被地球遮挡（库按同款公式拒绝检出）。</div>
  </div>
  <div class="panel">
    <h2>双星交会误差（{eval_pair_label}，库 dual_sat 样本）</h2>
    <div id="fixWrap"></div>
  </div>
</div>

<div class="tooltip" id="tooltip"></div>

<script id="viz-data" type="application/json">{data_json}</script>
<script>
"use strict";
/* ═══════════════ 数据 ═══════════════ */
const DATA = JSON.parse(document.getElementById("viz-data").textContent);
const META = DATA.meta;
const N = META.cycles || 0;
const SATS = DATA.sats || [];
const SAT_BY_SID = {{}};
for (const s of SATS) SAT_BY_SID[s.sid] = s;
const TRUTH_KEYS = Object.keys(DATA.truth || {{}}).sort((a, b) => a - b);
const TRUTH_COLOR = {{}};
TRUTH_KEYS.forEach((k, i) => TRUTH_COLOR[k] = {target_palette}[i % {target_palette_len}]);
const LOS = DATA.los || [];
const LOS_BY_C = {{}};
for (const e of LOS) (LOS_BY_C[e.c] = LOS_BY_C[e.c] || []).push(e);
const FUSED = DATA.fused || [];
const FUSED_BY_C = {{}};
for (const e of FUSED) (FUSED_BY_C[e.c] = FUSED_BY_C[e.c] || []).push(e);
const FIX = DATA.fix || [];
const FIX_BY_CK = {{}};
for (const e of FIX) FIX_BY_CK[e.c + ":" + e.key] = e;
const EARTH_R = META.earthR;
const GEO_R = SATS.length ? Math.hypot(SATS[0].p[0], SATS[0].p[1], SATS[0].p[2]) : 42164.0;
const COLOR_EARTH_GRID = "{COLOR_EARTH_GRID}";
const COLOR_EARTH_BACK = "{COLOR_EARTH_BACK}";
const COLOR_EARTH_FILL = "{COLOR_EARTH_FILL}";
const COLOR_FIX = "{COLOR_FIX}";
const COLOR_FUSED = "{COLOR_FUSED}";

/* ═══════════════ 相机与 3D→2D 投影 ═══════════════
   相机由注视点 target、方位角 yaw、俯仰角 pitch、距离 dist 决定；
   eye = target + dist·(cos p·cos y, cos p·sin y, sin p)。
   投影：p_cam 相对相机正交基 (x=右, y=上, z=前)，透视除以深度。 */
const cam = {{ target:[0,0,0], yaw: -0.44, pitch: 0.28, dist: 98000, fovScale: 1.35 }};
function eyePos() {{
  const cp = Math.cos(cam.pitch), sp = Math.sin(cam.pitch);
  return [cam.target[0] + cam.dist * cp * Math.cos(cam.yaw),
          cam.target[1] + cam.dist * cp * Math.sin(cam.yaw),
          cam.target[2] + cam.dist * sp];
}}
let VIEW = null;
function updateView(cv) {{
  const eye = eyePos();
  const fwd = [cam.target[0]-eye[0], cam.target[1]-eye[1], cam.target[2]-eye[2]];
  const fl = Math.hypot(fwd[0], fwd[1], fwd[2]);
  const f = [fwd[0]/fl, fwd[1]/fl, fwd[2]/fl];
  const upW = [0, 0, 1];
  let rx = [upW[1]*f[2]-upW[2]*f[1], upW[2]*f[0]-upW[0]*f[2], upW[0]*f[1]-upW[1]*f[0]];
  const rl = Math.hypot(rx[0], rx[1], rx[2]) || 1;
  rx = [rx[0]/rl, rx[1]/rl, rx[2]/rl];
  const ry = [f[1]*rx[2]-f[2]*rx[1], f[2]*rx[0]-f[0]*rx[2], f[0]*rx[1]-f[1]*rx[0]];
  const scale = Math.min(cv.width, cv.height) * cam.fovScale;
  VIEW = {{
    eye, f, rx, ry, cx: cv.width/2, cy: cv.height/2, scale,
    project(p) {{
      const dx = p[0]-eye[0], dy = p[1]-eye[1], dz = p[2]-eye[2];
      const depth = dx*f[0] + dy*f[1] + dz*f[2];
      if (depth <= 1.0) return null;
      const px = dx*rx[0] + dy*rx[1] + dz*rx[2];
      const py = dx*ry[0] + dy*ry[1] + dz*ry[2];
      return {{x: this.cx + px/depth*this.scale, y: this.cy - py/depth*this.scale, depth}};
    }},
    front(p) {{  // 球面点可见性：法向与相机方向点积 > 0
      const n = [p[0]/EARTH_R, p[1]/EARTH_R, p[2]/EARTH_R];
      const toEye = [eye[0]-p[0], eye[1]-p[1], eye[2]-p[2]];
      return n[0]*toEye[0] + n[1]*toEye[1] + n[2]*toEye[2] > 0;
    }},
  }};
}}

/* ═══════════════ 状态与图层 ═══════════════ */
let cycle = 1, playing = false, timer = null;
const LAYERS = [
  {{key:"grid",  label:"地球网格", color:COLOR_EARTH_GRID, on:true}},
  {{key:"sats",  label:"卫星",     color:"#1f77b4", on:true}},
  {{key:"fov",   label:"视场锥",   color:"#4a7fa5", on:true}},
  {{key:"los",   label:"视线",     color:"#8fb8d8", on:true}},
  {{key:"truth", label:"目标真值", color:"#d62728", on:true}},
  {{key:"fused", label:"融合航迹", color:COLOR_FUSED, on:true}},
  {{key:"fix",   label:"双星交会", color:COLOR_FIX, on:true}},
  {{key:"labels",label:"标签",     color:"#aab8c8", on:true}},
];
const layerOn = k => LAYERS.find(l => l.key === k).on;
const tooltip = document.getElementById("tooltip");
function showTip(text, ev) {{
  tooltip.textContent = text;
  tooltip.style.display = "block";
  let x = ev.clientX + 14, y = ev.clientY + 14;
  if (x + tooltip.offsetWidth > window.innerWidth - 8) x = ev.clientX - tooltip.offsetWidth - 14;
  if (y + tooltip.offsetHeight > window.innerHeight - 8) y = ev.clientY - tooltip.offsetHeight - 14;
  tooltip.style.left = x + "px"; tooltip.style.top = y + "px";
}}
function hideTip() {{ tooltip.style.display = "none"; }}
function fmt(v, d) {{ return (v === null || v === undefined) ? "-" : v.toFixed(d); }}

/* ═══════════════ 几何小工具 ═══════════════ */
function vsub(a,b) {{ return [a[0]-b[0], a[1]-b[1], a[2]-b[2]]; }}
function vdot(a,b) {{ return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }}
function vlen(a) {{ return Math.hypot(a[0],a[1],a[2]); }}
function vscale(a,s) {{ return [a[0]*s, a[1]*s, a[2]*s]; }}
function vunit(a) {{ const l = vlen(a)||1; return [a[0]/l,a[1]/l,a[2]/l]; }}
function vcross(a,b) {{ return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]; }}
// 射线(o,d) 与球(r) 最近交点；无命中返回 null。
function raySphere(o, d, r) {{
  const b = vdot(o, d), c = vdot(o, o) - r*r;
  const disc = b*b - c;
  if (disc < 0) return null;
  const t = -b - Math.sqrt(disc);
  if (t < 0) return null;
  return [o[0]+d[0]*t, o[1]+d[1]*t, o[2]+d[2]*t];
}}

/* ═══════════════ 地球网格预采样 ═══════════════ */
const GRID = (() => {{
  const rings = [];
  for (let lat = -60; lat <= 60; lat += 30) {{
    const pts = [];
    const rr = EARTH_R * Math.cos(lat*Math.PI/180), z = EARTH_R * Math.sin(lat*Math.PI/180);
    for (let a = 0; a <= 360; a += 5) {{
      const t = a*Math.PI/180;
      pts.push([rr*Math.cos(t), rr*Math.sin(t), z]);
    }}
    rings.push({{pts, lat, lon: null}});
  }}
  for (let lon = 0; lon < 360; lon += 30) {{
    const pts = [];
    const t = lon*Math.PI/180;
    for (let a = -90; a <= 90; a += 5) {{
      const p = a*Math.PI/180;
      pts.push([EARTH_R*Math.cos(p)*Math.cos(t), EARTH_R*Math.cos(p)*Math.sin(t), EARTH_R*Math.sin(p)]);
    }}
    rings.push({{pts, lat: null, lon}});
  }}
  return rings;
}})();

/* ═══════════════ 视场锥足印（每星一次缓存） ═══════════════
   光轴 = 星→地心（星下点凝视，场景 scan_center_el = 0 即指向地心）；
   圆锥半角 = 宽视场 az/el 半张角的大者（方形视场的内接圆近似）。 */
const FOV_CONES = SATS.map(s => {
  const axis = vunit(vscale(s.p, -1));
  const half = Math.max(s.wfovAz, s.wfovEl) / 2 * Math.PI / 180;
  // 与光轴正交的两个基向量。
  let u = Math.abs(axis[2]) < 0.9 ? [0,0,1] : [1,0,0];
  const e1 = vunit(vcross(u, axis));
  const e2 = vunit(vcross(axis, e1));
  const rays = [];
  for (let k = 0; k < 12; k++) {{
    const ph = k / 12 * 2 * Math.PI;
    const dir = vunit([
      axis[0]*Math.cos(half) + (e1[0]*Math.cos(ph) + e2[0]*Math.sin(ph))*Math.sin(half),
      axis[1]*Math.cos(half) + (e1[1]*Math.cos(ph) + e2[1]*Math.sin(ph))*Math.sin(half),
      axis[2]*Math.cos(half) + (e1[2]*Math.cos(ph) + e2[2]*Math.sin(ph))*Math.sin(half),
    ]);
    rays.push({{dir, hit: raySphere(s.p, dir, EARTH_R)}});
  }}
  return {{sat: s, rays}};
});

/* ═══════════════ 悬停命中列表（每帧重建） ═══════════════ */
let hitTargets = [];

/* ═══════════════ 主渲染 ═══════════════ */
const cv = document.getElementById("cv3d");
const ctx = cv.getContext("2d");
function drawSegment(p0, p1) {{
  const a = VIEW.project(p0), b = VIEW.project(p1);
  if (!a || !b) return;
  ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
}}
function drawPolyline3(pts, fromIdx) {{
  ctx.beginPath();
  let started = false;
  for (let i = fromIdx || 0; i < pts.length; i++) {{
    const q = VIEW.project(pts[i]);
    if (!q) {{ started = false; continue; }}
    if (!started) {{ ctx.moveTo(q.x, q.y); started = true; }}
    else ctx.lineTo(q.x, q.y);
  }}
  ctx.stroke();
}}

function render() {{
  updateView(cv);
  ctx.clearRect(0, 0, cv.width, cv.height);
  hitTargets = [];
  const eye = VIEW.eye;

  // —— 地球：轮廓圆填充 + 经纬网格（前面亮、背面暗） ——
  const d0 = vlen(vsub(eye, [0,0,0]));
  if (layerOn("grid")) {{
    const ang = Math.asin(Math.min(1, EARTH_R / d0));
    const rr = Math.tan(ang) * VIEW.scale;
    const g = ctx.createRadialGradient(VIEW.cx - rr*0.3, VIEW.cy - rr*0.3, rr*0.2,
                                       VIEW.cx, VIEW.cy, rr);
    g.addColorStop(0, "#12293f"); g.addColorStop(1, COLOR_EARTH_FILL);
    ctx.beginPath(); ctx.arc(VIEW.cx, VIEW.cy, rr, 0, 2*Math.PI);
    ctx.fillStyle = g; ctx.fill();
    for (const ring of GRID) {{
      for (let i = 0; i + 1 < ring.pts.length; i += 2) {{
        const p = ring.pts[i], q = ring.pts[i+1];
        const front = VIEW.front(p);
        const a = VIEW.project(p), b = VIEW.project(q);
        if (!a || !b) continue;
        ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y);
        const isEq = ring.lat === 0;
        ctx.strokeStyle = front ? (isEq ? "#5b8db8" : COLOR_EARTH_GRID) : COLOR_EARTH_BACK;
        ctx.lineWidth = isEq && front ? 1.3 : 0.8;
        ctx.stroke();
      }}
    }}
  }}

  // —— 视场锥：母线（星→球面足印）+ 足印闭合圈 ——
  if (layerOn("fov")) {{
    for (const cone of FOV_CONES) {{
      ctx.strokeStyle = cone.sat.color; ctx.globalAlpha = 0.32; ctx.lineWidth = 0.9;
      for (const ray of cone.rays) if (ray.hit) drawSegment(cone.sat.p, ray.hit);
      ctx.globalAlpha = 0.55; ctx.setLineDash([4, 4]);
      ctx.beginPath();
      let started = false;
      for (const ray of cone.rays) {{
        if (!ray.hit) {{ started = false; continue; }}
        const q = VIEW.project(ray.hit);
        if (!q) {{ started = false; continue; }}
        if (!started) {{ ctx.moveTo(q.x, q.y); started = true; }} else ctx.lineTo(q.x, q.y);
      }}
      ctx.closePath(); ctx.stroke();
      ctx.setLineDash([]); ctx.globalAlpha = 1;
    }}
  }}

  // —— 星下点连线（卫星→地心方向到球面） ——
  if (layerOn("sats")) {{
    ctx.setLineDash([2, 5]); ctx.lineWidth = 1;
    for (const s of SATS) {{
      ctx.strokeStyle = s.color; ctx.globalAlpha = 0.45;
      const nadir = vscale(vunit(s.p), EARTH_R);
      drawSegment(s.p, nadir);
    }}
    ctx.setLineDash([]); ctx.globalAlpha = 1;
  }}

  // —— 目标真值轨迹（淡全轨迹 + 截至当前周期加粗） ——
  if (layerOn("truth")) {{
    for (const k of TRUTH_KEYS) {{
      const pts = DATA.truth[k].map(q => q.p);
      if (!pts.length) continue;
      ctx.strokeStyle = TRUTH_COLOR[k]; ctx.lineWidth = 1;
      ctx.globalAlpha = 0.35;
      drawPolyline3(pts, 0);
      ctx.globalAlpha = 1; ctx.lineWidth = 2.2;
      const upto = Math.min(cycle, DATA.truth[k].length);
      drawPolyline3(pts.slice(0, upto), 0);
    }}
  }}

  // —— 融合航迹点列（半透明小点 + 当前点） ——
  if (layerOn("fused")) {{
    for (const e of FUSED) {{
      if (!e.p) continue;
      const q = VIEW.project(e.p);
      if (!q) continue;
      const isNow = e.c === cycle;
      ctx.beginPath(); ctx.arc(q.x, q.y, isNow ? 4.5 : 1.6, 0, 2*Math.PI);
      ctx.fillStyle = COLOR_FUSED;
      ctx.globalAlpha = isNow ? 1 : 0.3;
      ctx.fill();
      ctx.globalAlpha = 1;
      if (isNow) hitTargets.push({{x:q.x, y:q.y, r:7, tip:
        `融合 k${{e.key}} @ cycle ${{e.c}}\\n生命周期 ${{e.lc}}，置信 ${{fmt(e.conf,3)}}\\n通道 ${{e.ch}}`}});
    }}
  }}

  // —— 视线：本周期 detected 星 → 目标真值 ——
  if (layerOn("los")) {{
    const truthNow = {{}};
    for (const k of TRUTH_KEYS) {{
      const arr = DATA.truth[k];
      const q = arr[Math.min(cycle, arr.length) - 1];
      if (q) truthNow[k] = q.p;
    }}
    for (const e of (LOS_BY_C[cycle] || [])) {{
      if (e.status !== "detected" && e.status !== "below_threshold") continue;
      const tp = truthNow[e.tid];
      const sat = SAT_BY_SID[e.sid];
      if (!tp || !sat) continue;
      ctx.strokeStyle = sat.color;
      ctx.lineWidth = e.status === "detected" ? 1.1 : 0.8;
      ctx.globalAlpha = e.status === "detected" ? 0.8 : 0.35;
      if (e.status !== "detected") ctx.setLineDash([2, 4]);
      drawSegment(sat.p, tp);
      ctx.setLineDash([]); ctx.globalAlpha = 1;
    }}
  }}

  // —— 双星交会：A/B 测角视线（虚线）→ 交会点十字 → 交会点到真值的误差线 ——
  if (layerOn("fix")) {{
    const truthNow = {{}};
    for (const k of TRUTH_KEYS) {{
      const arr = DATA.truth[k];
      const q = arr[Math.min(cycle, arr.length) - 1];
      if (q) truthNow[k] = q.p;
    }}
    for (const k of TRUTH_KEYS) {{
      const fx = FIX_BY_CK[cycle + ":" + k];
      if (!fx || !fx.p) continue;
      const satsAB = [SATS[0], SATS[1]];
      ctx.setLineDash([3, 4]); ctx.lineWidth = 0.9; ctx.globalAlpha = 0.75;
      for (const s of satsAB) {{
        ctx.strokeStyle = s.color;
        drawSegment(s.p, fx.p);
      }}
      ctx.setLineDash([]); ctx.globalAlpha = 1;
      const q = VIEW.project(fx.p);
      const tp = truthNow[k];
      if (tp) {{
        ctx.strokeStyle = COLOR_FIX; ctx.lineWidth = 1.4;
        drawSegment(fx.p, tp);
      }}
      if (q) {{
        ctx.strokeStyle = COLOR_FIX; ctx.lineWidth = 1.6;
        ctx.beginPath();
        ctx.moveTo(q.x-7, q.y); ctx.lineTo(q.x+7, q.y);
        ctx.moveTo(q.x, q.y-7); ctx.lineTo(q.x, q.y+7);
        ctx.stroke();
        hitTargets.push({{x:q.x, y:q.y, r:8, tip:
          `双星交会 目标 ${{k}} @ cycle ${{fx.c}}\\n位置误差（库） ${{fmt(fx.err,0)}} m` +
          (fx.recalcErr !== null ? `\\n位置误差（查看器复算） ${{fmt(fx.recalcErr,0)}} m` : "") +
          `\\n视线残差 ${{fmt(fx.resid,1)}} m\\n斜距误差 ${{fmt(fx.srerr,0)}} m`}});
      }}
    }}
  }}

  // —— 卫星标记 + 标签 ——
  for (const s of SATS) {{
    const q = VIEW.project(s.p);
    if (!q) continue;
    ctx.fillStyle = s.color;
    ctx.fillRect(q.x - 4, q.y - 4, 8, 8);
    ctx.strokeStyle = "#fff"; ctx.lineWidth = 1;
    ctx.strokeRect(q.x - 4, q.y - 4, 8, 8);
    hitTargets.push({{x:q.x, y:q.y, r:9, tip:
      `卫星 ${{s.id}}（源 ${{s.sid}}）\\nGEO 半径 ${{fmt(GEO_R,0)}} km\\n扫描中心 az ${{fmt(s.scanAz,1)}}° el ${{fmt(s.scanEl,1)}}°\\n宽视场 ${{fmt(s.wfovAz,0)}}°×${{fmt(s.wfovEl,0)}}°（锥为内接圆近似）`}});
    if (layerOn("labels")) {{
      ctx.fillStyle = s.color; ctx.font = "12px sans-serif";
      ctx.fillText(`星 ${{s.id}}`, q.x + 8, q.y - 6);
    }}
  }}

  // —— 目标真值当前位置标记 + 标签 ——
  if (layerOn("truth")) {{
    for (const k of TRUTH_KEYS) {{
      const arr = DATA.truth[k];
      const q = arr[Math.min(cycle, arr.length) - 1];
      if (!q) continue;
      const sp = VIEW.project(q.p);
      if (!sp) continue;
      ctx.beginPath(); ctx.arc(sp.x, sp.y, 4, 0, 2*Math.PI);
      ctx.fillStyle = TRUTH_COLOR[k]; ctx.fill();
      ctx.strokeStyle = "#fff"; ctx.lineWidth = 1; ctx.stroke();
      hitTargets.push({{x:sp.x, y:sp.y, r:7, tip:
        `目标真值 T${{k}} @ cycle ${{q.c}}\\nECEF (${{fmt(q.p[0],0)}}, ${{fmt(q.p[1],0)}}, ${{fmt(q.p[2],0)}}) km\\n地心距 ${{fmt(vlen(q.p),0)}} km`}});
      if (layerOn("labels")) {{
        ctx.fillStyle = TRUTH_COLOR[k]; ctx.font = "12px sans-serif";
        ctx.fillText(`T${{k}}`, sp.x + 7, sp.y + 4);
      }}
    }}
  }}
}}

/* ═══════════════ 状态面板与误差曲线 ═══════════════ */
const STATUS_NAMES = {{
  detected: "检出", below_threshold: "低于门限", coasting: "滑行保持",
  not_in_output: "未检出", not_executed: "本周期未执行", unknown: "未知",
}};
function renderLosTable() {{
  const rows = LOS_BY_C[cycle] || [];
  let htmlStr = "<tr><th>卫星</th><th>目标</th><th>状态</th><th>测角 az/el（°）</th><th>SNR</th><th>被挡深度（km）</th></tr>";
  for (const e of rows) {{
    const sat = SAT_BY_SID[e.sid];
    const stName = STATUS_NAMES[e.status] || e.status;
    const occulted = e.margin !== null && e.margin !== undefined && e.margin < 0;
    const azDeg = e.az === null ? "-" : (e.az * 180 / Math.PI).toFixed(3);
    const elDeg = e.el === null ? "-" : (e.el * 180 / Math.PI).toFixed(3);
    htmlStr += `<tr><td style="color:${{sat ? sat.color : "#888"}}">星 ${{sat ? sat.id : e.sid}}</td>` +
      `<td>T${{e.tid}}</td>` +
      `<td class="${{e.status === "detected" ? "det" : (occulted ? "occult" : "")}}">${{stName}}</td>` +
      `<td>${{e.status === "detected" ? azDeg + " / " + elDeg : "-"}}</td>` +
      `<td>${{e.status === "detected" ? fmt(e.snr, 3) : "-"}}</td>` +
      `<td class="${{occulted ? "occult" : ""}}">${{occulted ? fmt(e.margin, 1) : "可见"}}</td></tr>`;
  }}
  document.getElementById("losTable").innerHTML = htmlStr;
}}
function renderFixChart() {{
  const W = 460, H = 180, ML = 46, MB = 24;
  const keys = [...new Set(FIX.map(e => e.key))];
  const maxErr = Math.max(100, ...FIX.map(e => e.err));
  const ns = "http://www.w3.org/2000/svg";
  const svg = document.createElementNS(ns, "svg");
  svg.setAttribute("viewBox", `0 0 ${{W}} ${{H}}`);
  const sx = c => ML + (c - 1) / Math.max(1, N - 1) * (W - ML - 10);
  const sy = v => H - MB - v / maxErr * (H - MB - 12);
  // 轴
  const ax = document.createElementNS(ns, "line");
  ax.setAttribute("x1", ML); ax.setAttribute("y1", H - MB);
  ax.setAttribute("x2", W - 10); ax.setAttribute("y2", H - MB);
  ax.setAttribute("stroke", "#33404f");
  svg.appendChild(ax);
  for (const v of [0, maxErr / 2, maxErr]) {{
    const t = document.createElementNS(ns, "text");
    t.setAttribute("x", ML - 6); t.setAttribute("y", sy(v) + 3);
    t.setAttribute("text-anchor", "end"); t.setAttribute("font-size", "9.5");
    t.setAttribute("fill", "#8393a8");
    t.textContent = fmt(v, 0);
    svg.appendChild(t);
    const gl = document.createElementNS(ns, "line");
    gl.setAttribute("x1", ML); gl.setAttribute("y1", sy(v));
    gl.setAttribute("x2", W - 10); gl.setAttribute("y2", sy(v));
    gl.setAttribute("stroke", "#242e3b");
    svg.appendChild(gl);
  }}
  const palette = ["#d62728", "#9467bd", "#2ca02c"];
  keys.forEach((k, idx) => {{
    const pts = FIX.filter(e => e.key === k);
    const d = pts.map(e => `${{sx(e.c)}},${{sy(e.err)}}`).join(" ");
    const pl = document.createElementNS(ns, "polyline");
    pl.setAttribute("points", d);
    pl.setAttribute("fill", "none");
    pl.setAttribute("stroke", palette[idx % palette.length]);
    pl.setAttribute("stroke-width", "1.6");
    svg.appendChild(pl);
    const lab = document.createElementNS(ns, "text");
    lab.setAttribute("x", W - 12); lab.setAttribute("y", 14 + idx * 13);
    lab.setAttribute("text-anchor", "end"); lab.setAttribute("font-size", "10");
    lab.setAttribute("fill", palette[idx % palette.length]);
    const last = pts[pts.length - 1];
    lab.textContent = `T${{k}}：均值 ${{fmt(pts.reduce((a,e)=>a+e.err,0)/pts.length,0)}} m · 最大 ${{fmt(Math.max(...pts.map(e=>e.err)),0)}} m`;
    svg.appendChild(lab);
  }});
  // 时间游标
  const cur = document.createElementNS(ns, "line");
  cur.setAttribute("x1", sx(cycle)); cur.setAttribute("y1", 8);
  cur.setAttribute("x2", sx(cycle)); cur.setAttribute("y2", H - MB);
  cur.setAttribute("stroke", "#5b8db8"); cur.setAttribute("stroke-dasharray", "3 3");
  svg.appendChild(cur);
  const wrap = document.getElementById("fixWrap");
  wrap.innerHTML = "";
  wrap.appendChild(svg);
}}

/* ═══════════════ 交互：拖拽旋转 / 缩放 / 悬停 ═══════════════ */
let dragging = false, lastX = 0, lastY = 0;
cv.addEventListener("mousedown", ev => {{ dragging = true; lastX = ev.clientX; lastY = ev.clientY; }});
window.addEventListener("mouseup", () => dragging = false);
window.addEventListener("mousemove", ev => {{
  if (dragging) {{
    cam.yaw += (ev.clientX - lastX) * 0.008;
    cam.pitch = Math.max(-1.45, Math.min(1.45, cam.pitch + (ev.clientY - lastY) * 0.006));
    lastX = ev.clientX; lastY = ev.clientY;
    render();
  }}
}});
cv.addEventListener("mousemove", ev => {{
  if (dragging) return hideTip();
  const rect = cv.getBoundingClientRect();
  const px = (ev.clientX - rect.left) * cv.width / rect.width;
  const py = (ev.clientY - rect.top) * cv.height / rect.height;
  for (const h of hitTargets) {{
    if (Math.hypot(px - h.x, py - h.y) <= h.r) return showTip(h.tip, ev);
  }}
  hideTip();
}});
cv.addEventListener("mouseleave", hideTip);
cv.addEventListener("wheel", ev => {{
  ev.preventDefault();
  cam.dist = Math.max(2.2 * EARTH_R, Math.min(220000, cam.dist * (ev.deltaY > 0 ? 1.12 : 0.89)));
  render();
}}, {{passive: false}});

function setView(yaw, pitch, dist, target) {{
  cam.yaw = yaw; cam.pitch = pitch; cam.dist = dist;
  cam.target = target || [0, 0, 0];
  render();
}}
// 预设视角：初始全景朝大西洋（星 A 星下点）；A 星后方（沿 A→地心反向）；目标 1。
document.getElementById("viewGlobal").addEventListener("click", () =>
  setView(-0.44, 0.28, 98000));
document.getElementById("viewSatA").addEventListener("click", () => {{
  const s = SATS[0].p;
  setView(Math.atan2(s[1], s[0]), 0.12, 160000);
}});
document.getElementById("viewT1").addEventListener("click", () => {{
  const arr = DATA.truth[TRUTH_KEYS[0]];
  if (!arr) return;
  const p = arr[Math.min(cycle, arr.length) - 1].p;
  setView(Math.atan2(p[1], p[0]), 0.55, 30000, p);
}});

/* ═══════════════ 图层开关与播放 ═══════════════ */
function renderLayerToggles() {{
  const box = document.getElementById("layerToggles");
  box.innerHTML = "";
  for (const l of LAYERS) {{
    const lab = document.createElement("label");
    const cb = document.createElement("input");
    cb.type = "checkbox"; cb.checked = l.on;
    cb.addEventListener("change", () => {{ l.on = cb.checked; render(); }});
    lab.appendChild(cb);
    const sw = document.createElement("span");
    sw.className = "swatch"; sw.style.background = l.color;
    lab.appendChild(sw);
    lab.appendChild(document.createTextNode(l.label));
    box.appendChild(lab);
  }}
}}
function setCycle(c) {{
  cycle = Math.max(1, Math.min(N, c));
  document.getElementById("cycleSlider").value = cycle;
  document.getElementById("cycleReadout").textContent = `cycle ${{cycle}} / ${{N}}`;
  render(); renderLosTable(); renderFixChart();
}}
function play() {{
  if (playing) return;
  playing = true;
  document.getElementById("playBtn").textContent = "⏸ 暂停";
  document.getElementById("playBtn").classList.add("active");
  const step = () => {{
    if (!playing) return;
    if (cycle >= N) {{ pause(); return; }}
    setCycle(cycle + 1);
  }};
  const speed = parseInt(document.getElementById("speedSel").value);
  timer = setInterval(step, Math.max(100, Math.round(1000 / speed)));
}}
function pause() {{
  playing = false;
  if (timer) clearInterval(timer);
  timer = null;
  document.getElementById("playBtn").textContent = "▶ 播放";
  document.getElementById("playBtn").classList.remove("active");
}}

/* ═══════════════ 初始化 ═══════════════ */
(function init() {{
  if (!N) {{
    document.body.insertAdjacentHTML("afterbegin",
      "<p style='color:#e0705a'>数据为空（CSV 缺失或行数不足）</p>");
    return;
  }}
  renderLayerToggles();
  document.getElementById("cycleSlider").addEventListener("input", e => {{
    pause(); setCycle(parseInt(e.target.value));
  }});
  document.getElementById("playBtn").addEventListener("click", () => playing ? pause() : play());
  document.getElementById("speedSel").addEventListener("change", () => {{ if (playing) {{ pause(); play(); }} }});
  // URL 锚点 #cycle=N 直接定位周期（如分享/存档特定帧）。
  const m = location.hash.match(/^#cycle=(\\d+)/);
  setCycle(m ? Math.max(1, Math.min(N, parseInt(m[1]))) : 1);
}})();
</script>
</body>
</html>
"""


# ═══════════════════════════════════════════════════════════════════
# 入口
# ═══════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="从卫星场景 CSV 构建三维可视化查看器（单文件离线 HTML）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    parser.add_argument("data_dir", help="场景输出目录（examples/log/sbirs_triple_sat_fix_messages/）")
    parser.add_argument("--out", default=None,
                        help="输出 HTML 路径（默认 <data_dir>/sbirs_orbit_viewer.html）")
    parser.add_argument("--check", action="store_true",
                        help="仅校验几何复算一致性（不生成 HTML）")
    args = parser.parse_args()

    if args.check:
        problems, warnings = validate(args.data_dir)
        for w in warnings:
            print("note: %s" % w)
        if problems:
            for p in problems:
                sys.stderr.write("CHECK FAIL: %s\n" % p)
            sys.exit(1)
        print("CHECK PASS: %s" % args.data_dir)
        return

    try:
        data = build_data(args.data_dir)
    except RuntimeError as exc:
        sys.stderr.write("错误：%s\n" % exc)
        sys.exit(1)

    cycles = data["meta"]["cycles"]
    if not cycles:
        sys.stderr.write("错误：无有效周期数据（sbirs_truth.csv 为空）\n")
        sys.exit(1)
    out_path = args.out or args.data_dir.rstrip("/") + "/sbirs_orbit_viewer.html"
    data_json = json.dumps(data, ensure_ascii=False, separators=(",", ":"))
    data_json = data_json.replace("</", "<\\/")  # 防止 </script> 提前闭合

    eval_ids = data["meta"]["evalPair"]
    sat_ids = "/".join(s["id"] for s in data["sats"])
    sub_line = ("%s 颗 GEO 卫星（%s） · %d 周期 · 目标 %s · 评估星对 源 %d/%d · "
                "数据目录 %s" % (
                    len(data["sats"]), sat_ids, cycles,
                    "/".join(sorted(data["truth"].keys())), eval_ids[0], eval_ids[1],
                    html.escape(args.data_dir)))
    title = "%d 周期 · %d 星" % (cycles, len(data["sats"]))

    template = HTML_TEMPLATE.replace("{{", "{").replace("}}", "}")
    page = (
        template.replace("{title}", html.escape(title))
        .replace("{sub_line}", sub_line)
        .replace("{cycles}", str(cycles))
        .replace("{earth_r}", "6371")
        .replace("{eval_pair_label}", "星 %s / %s" % (
            data["sats"][0]["id"], data["sats"][1]["id"]))
        .replace("{canvas_w}", "1160").replace("{canvas_h}", "640")
        .replace("{COLOR_EARTH_GRID}", COLOR_EARTH_GRID)
        .replace("{COLOR_EARTH_BACK}", COLOR_EARTH_BACK)
        .replace("{COLOR_EARTH_FILL}", COLOR_EARTH_FILL)
        .replace("{COLOR_FIX}", COLOR_FIX)
        .replace("{COLOR_FUSED}", COLOR_FUSED)
        .replace("{target_palette}", json.dumps(TARGET_PALETTE))
        .replace("{target_palette_len}", str(len(TARGET_PALETTE)))
        .replace("{data_json}", data_json)
    )
    with open(out_path, "w", encoding="utf-8") as fh:
        fh.write(page)

    print("已生成三维查看器：%s" % out_path)
    print("  数据：%s（%d 周期，%d 星，目标 %s）" % (
        args.data_dir, cycles, len(data["sats"]),
        "/".join(sorted(data["truth"].keys()))))
    print("  打开方式：open %s" % out_path)


if __name__ == "__main__":
    main()
