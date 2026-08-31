#!/usr/bin/env python3
"""
rir_scan_viewer.py — 远程识别雷达（RIR）扫描-识别可视化查看器。

从 RIR 场景可执行的 CSV 导出构建**单文件自包含 HTML**（vanilla JS + SVG，
无 CDN、离线可用），还原 RIR 对敌方目标的处置链：

  进入扫描范围（显示扫描扇区）→ 探测（形成航迹）→ 确认航迹 → 识别（站点到
  目标画识别连线，标注大类/型号/置信）。

用法：
  python3 rir_scan_viewer.py <data_dir> [--out viewer.html]
  python3 rir_scan_viewer.py --check <data_dir>   # 仅校验数据（不生成 HTML）

<data_dir> 为 RIR 场景可执行的输出目录（examples/log/<scene>/），须含由
outputs.cpp 落盘的：
  rir_site.csv     —— 站点 LLA + 扫描体积（扇区几何）+ 最大作用距离（一行）
  rir_targets.csv  —— 逐周期逐目标视线方位/俯仰、斜距、航迹状态、识别结论
  target_truth.csv —— 目标真值逐周期 LLA（无航迹阶段的地图定位回退）

本工具与通用查看器 build_viewer.py 解耦，仅依赖 Python 标准库。
"""

import argparse
import csv
import html
import json
import math
import os
import sys

EARTH_RADIUS_M = 6378137.0

# 四阶段配色（与 HTML 图例一致）。
STAGE_NAMES = ["扫描外", "进入扫描范围", "探测", "确认航迹", "识别"]
STAGE_COLORS = ["#9e9e9e", "#e6b800", "#ff7f0e", "#d62728", "#8e24aa"]
COLOR_SITE = "#1f77b4"
COLOR_WEDGE = "#1f77b4"       # 任务扫描扇区（用户指定作战搜索扇区 = 子窗 ∩ 体积）
COLOR_MAXWEDGE = "#90a4ae"   # 最大可扫描体积（硬件 steerable_volume）
COLOR_DWELL = "#ff5722"      # 波束驻留指向（当前周期天线指向）
COLOR_TRUTH = "#607d8b"
# 目标真值轨迹分色（按 target_id 升序取色）。
TARGET_PALETTE = ["#d62728", "#2ca02c", "#1f77b4", "#9467bd", "#ff7f0e",
                  "#17becf", "#8c564b", "#e377c2"]

RECOGNIZED_STATES = {"category_confirmed", "model_confirmed"}


# ═══════════════════════════════════════════════════════════════════
# CSV 读取与数值解析
# ═══════════════════════════════════════════════════════════════════

def load_csv(path):
    """读取 CSV 为 dict 列表；文件缺失返回 None。"""
    try:
        with open(path, newline="") as fh:
            return list(csv.DictReader(fh))
    except OSError:
        return None


def fnum(row, key):
    """取行字段为 float；缺失/空返回 None。"""
    v = row.get(key)
    if v is None or v == "":
        return None
    try:
        return float(v)
    except ValueError:
        return None


def inum(row, key):
    """取行字段为 int；缺失/空返回 None。"""
    v = row.get(key)
    if v is None or v == "":
        return None
    try:
        return int(float(v))
    except ValueError:
        return None


# ═══════════════════════════════════════════════════════════════════
# 几何：本地 ENU 投影 + 扫描扇区判定
# ═══════════════════════════════════════════════════════════════════

def project_enu(lat, lon, origin_lat, origin_lon):
    """lat/lon（度）→ {x, y}（m，东/北），以站点为原点。"""
    lat0 = math.radians(origin_lat)
    x = math.radians(lon - origin_lon) * EARTH_RADIUS_M * math.cos(lat0)
    y = math.radians(lat - origin_lat) * EARTH_RADIUS_M
    return {"x": x, "y": y}


def norm_deg(a):
    """归一化到 (-180, 180]。"""
    a = (a + 180.0) % 360.0 - 180.0
    return 180.0 if a == -180.0 else a


def az_in_wedge(az, lo, hi):
    """az 是否落在方位扇区 [lo, hi]（deg，可跨 ±180 卷绕，宽度 <= 360）。"""
    width = (hi - lo) % 360.0
    if width == 0.0:
        width = 360.0  # az_max == az_min 视为全向（保守）
    delta = (az - lo) % 360.0
    return delta <= width + 1e-6


def compute_stage(row, site):
    """由 rir_targets 行 + 站点扫描体积推四阶段（0 扫描外 … 4 识别）。"""
    present = (row.get("present_in_input") == "1")
    has_track = (row.get("has_track") == "1")
    status = row.get("status") or "no_track"
    reco = row.get("recognition_state") or "disabled"
    if reco in RECOGNIZED_STATES:
        return 4
    if status == "confirmed":
        return 3
    if has_track:
        return 2  # 有航迹但未确认 = 探测（候选航迹）
    # 无航迹：看是否几何进入「任务扫描扇区」= 子窗 ∩ 硬件体积（实际被搜索的范围）。
    look_az = fnum(row, "look_az_deg")
    look_el = fnum(row, "look_el_deg")
    slant = fnum(row, "slant_range_m")
    if not present or look_az is None or look_el is None or slant is None:
        return 0
    az_lo = site["scan_center_az"] + max(site["az_min"], site["win_az_min"])
    az_hi = site["scan_center_az"] + min(site["az_max"], site["win_az_max"])
    el_min = max(site["el_min"], site["win_el_min"])
    el_max = min(site["el_max"], site["win_el_max"])
    in_sector = (az_in_wedge(norm_deg(look_az), norm_deg(az_lo), norm_deg(az_hi)) and
                 el_min - 1e-6 <= look_el <= el_max + 1e-6 and
                 slant <= site["max_range"] + 1e-6)
    return 1 if in_sector else 0


# ═══════════════════════════════════════════════════════════════════
# 数据装配
# ═══════════════════════════════════════════════════════════════════

def read_site(data_dir):
    """读取 rir_site.csv 首行为站点几何字典；缺失返回 None。"""
    rows = load_csv(data_dir + "/rir_site.csv")
    if not rows:
        return None
    r = rows[0]
    # 任务扫描子窗缺省无界 [-180,180]×[-90,90]（旧 CSV 无此列时同样回退无界）：
    # 与硬件体积取交后 = 硬件体积，任务扇区退化为最大扇区（两扇面重合）。
    win_az_min = fnum(r, "scan_win_az_min_deg")
    win_az_max = fnum(r, "scan_win_az_max_deg")
    win_el_min = fnum(r, "scan_win_el_min_deg")
    win_el_max = fnum(r, "scan_win_el_max_deg")
    return {
        "lat": fnum(r, "site_lat_deg"),
        "lon": fnum(r, "site_lon_deg"),
        "alt": fnum(r, "site_alt_m"),
        "scan_center_az": fnum(r, "scan_center_az_deg") or 0.0,
        "scan_center_el": fnum(r, "scan_center_el_deg") or 0.0,
        "az_min": fnum(r, "az_min_deg") or 0.0,
        "az_max": fnum(r, "az_max_deg") or 0.0,
        "el_min": fnum(r, "el_min_deg") or 0.0,
        "el_max": fnum(r, "el_max_deg") or 0.0,
        "max_range": fnum(r, "max_range_m") or 0.0,
        "win_az_min": win_az_min if win_az_min is not None else -180.0,
        "win_az_max": win_az_max if win_az_max is not None else 180.0,
        "win_el_min": win_el_min if win_el_min is not None else -90.0,
        "win_el_max": win_el_max if win_el_max is not None else 90.0,
    }


def build_data(data_dir):
    """读取 RIR CSV 与目标真值，装配为内嵌 JSON 的字典。"""
    site = read_site(data_dir)
    if site is None or site["lat"] is None:
        return None
    origin_lat, origin_lon = site["lat"], site["lon"]

    targets_rows = load_csv(data_dir + "/rir_targets.csv") or []
    truth_rows = load_csv(data_dir + "/target_truth.csv") or []

    # 目标真值：按 id 分组的逐周期 LLA（无航迹阶段的地图定位来源）。
    truth_by_id = {}
    truth_pos = {}  # (cycle, id) -> {x, y, alt}
    for r in truth_rows:
        tid = inum(r, "target_id")
        c = inum(r, "cycle")
        lat = fnum(r, "lat_deg")
        lon = fnum(r, "lon_deg")
        if tid is None or c is None or lat is None or lon is None:
            continue
        enu = project_enu(lat, lon, origin_lat, origin_lon)
        alt = fnum(r, "alt_m")
        truth_by_id.setdefault(tid, []).append({"c": c, "x": enu["x"], "y": enu["y"], "alt": alt})
        truth_pos[(c, tid)] = {"x": enu["x"], "y": enu["y"], "alt": alt}

    # RIR 逐周期逐目标：阶段 + 位置（优先航迹 LLA，回退真值）。
    rir_rows = []
    max_cycle = 0
    # 库上报「实际有效目标最大斜距」逐周期（RirCycleResult.max_detected_slant_range_m，
    # 逐目标行重复；取任一即可）。区别于 max_range_m 径向粗筛门。
    lib_slant_by_cycle = {}
    for r in targets_rows:
        c = inum(r, "cycle")
        tid = inum(r, "target_id")
        if c is None or tid is None:
            continue
        max_cycle = max(max_cycle, c)
        lib_slant = fnum(r, "cycle_max_detected_slant_m")
        if lib_slant is not None:
            lib_slant_by_cycle[c] = lib_slant
        stage = compute_stage(r, site)
        pos_lat = fnum(r, "pos_lat_deg")
        pos_lon = fnum(r, "pos_lon_deg")
        if pos_lat is not None and pos_lon is not None:
            enu = project_enu(pos_lat, pos_lon, origin_lat, origin_lon)
            x, y = enu["x"], enu["y"]
        else:
            tp = truth_pos.get((c, tid))
            x, y = (tp["x"], tp["y"]) if tp else (None, None)
        rir_rows.append({
            "c": c,
            "id": tid,
            "name": r.get("target_name") or "",
            "stage": stage,
            "x": x,
            "y": y,
            "az": fnum(r, "look_az_deg"),
            "el": fnum(r, "look_el_deg"),
            "slant": fnum(r, "slant_range_m"),
            "spd": fnum(r, "speed_mps"),
            "reco": r.get("recognition_state") or "disabled",
            "cat": r.get("target_category") or "unknown",
            "model": r.get("target_model") or "",
            "conf": fnum(r, "confidence"),
            "desig": r.get("designation_active") == "1",
            "dwell_az": fnum(r, "dwell_center_az_deg"),
            "dwell_el": fnum(r, "dwell_center_el_deg"),
        })

    # 每目标斜距/高度时间序列（下方剖面面板）+ 阶段事件首拍。
    ids = sorted({row["id"] for row in rir_rows})
    series = {}
    for tid in ids:
        rows = sorted((r for r in rir_rows if r["id"] == tid), key=lambda z: z["c"])
        first_detect = next((r["c"] for r in rows if r["stage"] >= 2), None)
        first_confirm = next((r["c"] for r in rows if r["stage"] >= 3), None)
        first_reco = next((r["c"] for r in rows if r["stage"] >= 4), None)
        series[str(tid)] = {
            "range": [{"c": r["c"], "v": r["slant"], "stage": r["stage"]} for r in rows if r["slant"] is not None],
            "alt": [{"c": t["c"], "v": t["alt"]} for t in sorted(truth_by_id.get(tid, []), key=lambda z: z["c"]) if t["alt"] is not None],
            "first_detect": first_detect,
            "first_confirm": first_confirm,
            "first_reco": first_reco,
        }

    if not max_cycle:
        max_cycle = max((t["c"] for pts in truth_by_id.values() for t in pts), default=0)

    # 扫描扇区显示半径：按「有处置意义目标（stage>=1）的最大斜距 ×1.3」截断，
    # 避免 max_range（配置粗筛门，可达数千 km）把近距目标压成一个点；真实
    # max_range 仍在图注标注。无 stage>=1 目标时回退到 max_range。
    actionable = [r["slant"] for r in rir_rows if r["stage"] >= 1 and r["slant"] is not None]
    site["display_range"] = (min(site["max_range"], 1.3 * max(actionable))
                             if actionable else site["max_range"])
    if not site["display_range"] or site["display_range"] <= 0:
        site["display_range"] = site["max_range"]

    return {
        "meta": {"cycles": max_cycle, "targets": len(ids)},
        "site": site,
        "truth": {str(k): v for k, v in truth_by_id.items()},
        "rir": rir_rows,
        "ids": ids,
        "series": series,
        "lib_slant": {str(k): v for k, v in lib_slant_by_cycle.items()},
    }


# ═══════════════════════════════════════════════════════════════════
# HTML 模板（vanilla JS + SVG，自包含；令牌替换注入，JS/CSS 花括号原样保留）
# ═══════════════════════════════════════════════════════════════════

HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>RIR 扫描-识别可视化 — __TITLE__</title>
<style>
  :root { --bg:#fafafa; --panel:#fff; --ink:#222; --muted:#888; }
  * { box-sizing:border-box; }
  body { margin:0; padding:16px 20px; background:var(--bg); color:var(--ink);
         font:14px/1.5 -apple-system,"PingFang SC","Helvetica Neue",Arial,sans-serif; }
  h1 { font-size:18px; margin:0 0 4px; }
  .sub { color:var(--muted); font-size:12px; margin-bottom:12px; }
  .panel { background:var(--panel); border:1px solid #e3e3e3; border-radius:8px;
           padding:10px 14px; margin-bottom:14px; }
  .panel h2 { font-size:14px; margin:0 0 6px; color:#444; }
  #controls { display:flex; flex-wrap:wrap; gap:14px; align-items:center; }
  #cycleSlider { width:420px; }
  .btn { padding:4px 14px; border:1px solid #bbb; border-radius:5px; background:#fff;
         cursor:pointer; font-size:13px; }
  .btn:hover { background:#f0f0f0; }
  .btn.active { background:#d62728; color:#fff; border-color:#d62728; }
  #cycleReadout { font-variant-numeric:tabular-nums; min-width:120px; }
  .layers { display:flex; flex-wrap:wrap; gap:10px; }
  .layers label { display:flex; align-items:center; gap:4px; cursor:pointer; }
  .swatch { width:11px; height:11px; border-radius:2px; display:inline-block; }
  svg { display:block; width:100%; height:auto; overflow:hidden; }
  .axis { stroke:#ddd; } .grid { stroke:#eee; }
  .cursor { stroke:#333; stroke-dasharray:4 3; }
  .tooltip { position:fixed; display:none; pointer-events:none; background:rgba(20,20,20,.92);
             color:#fff; font-size:12px; padding:6px 9px; border-radius:5px; z-index:50;
             max-width:340px; white-space:pre-line; }
  .legend { font-size:12px; color:#555; margin-top:6px; }
  .legend span { margin-right:14px; white-space:nowrap; }
</style>
</head>
<body>
<h1>RIR 扫描-识别可视化</h1>
<div class="sub">数据目录：__DATADIR__ · 目标 <b>__TARGETS__</b> 个 · 周期 <b>__CYCLES__</b>
  · 处置链：进入扫描范围 → 探测 → 确认航迹 → 识别连线</div>

<div class="panel">
  <div id="controls">
    <button id="playBtn" class="btn">▶ 播放</button>
    <label>速度
      <select id="speedSel">
        <option value="2">2 周期/s</option>
        <option value="5" selected>5 周期/s</option>
        <option value="10">10 周期/s</option>
        <option value="20">20 周期/s</option>
        <option value="50">50 周期/s</option>
      </select>
    </label>
    <input id="cycleSlider" type="range" min="1" max="__CYCLES__" value="1">
    <span id="cycleReadout">cycle 1 / __CYCLES__</span>
    <span id="libSlant" style="color:#8e24aa;font-variant-numeric:tabular-nums"></span>
    <span class="layers" id="layerToggles"></span>
  </div>
  <div class="legend" id="stageLegend"></div>
</div>

<div class="panel">
  <h2>俯视地图（本地 ENU，东/北，m；原点 = RIR 站点）</h2>
  <div class="legend" id="wedgeNote"></div>
  <div id="mapWrap"></div>
</div>

<div class="panel">
  <h2>斜距 / 高度随周期（实线 = 斜距，虚线 = 高度；▲ 首探测 ◆ 首确认 ★ 首识别；点线 = 最大作用距离）</h2>
  <div id="profileWrap"></div>
</div>

<div class="tooltip" id="tooltip"></div>

<script id="viz-data" type="application/json">__DATA_JSON__</script>
<script id="viz-const" type="application/json">__CONST_JSON__</script>
<script>
"use strict";
const DATA = JSON.parse(document.getElementById("viz-data").textContent);
const C = JSON.parse(document.getElementById("viz-const").textContent);
const SITE = DATA.site;
const N = DATA.meta.cycles || 0;
const RIR = DATA.rir || [];
const TRUTH = DATA.truth || {};
const IDS = DATA.ids || [];
const SERIES = DATA.series || {};
const LIB_SLANT = DATA.lib_slant || {};  // cycle -> 库上报本周期实际有效目标最大斜距(m)
// 实际任务扫描扇区 = 任务子窗 ∩ 硬件可扫描体积（逐轴取交；az 相对、el 绝对）。
const SECTOR = {
  az_min: Math.max(SITE.az_min, SITE.win_az_min),
  az_max: Math.min(SITE.az_max, SITE.win_az_max),
  el_min: Math.max(SITE.el_min, SITE.win_el_min),
  el_max: Math.min(SITE.el_max, SITE.win_el_max),
};
// 任务子窗是否真正收窄（否则两扇面重合，只画最大扇面）。
const HAS_TASK_WINDOW = (SECTOR.az_min > SITE.az_min + 1e-6 || SECTOR.az_max < SITE.az_max - 1e-6 ||
                         SECTOR.el_min > SITE.el_min + 1e-6 || SECTOR.el_max < SITE.el_max - 1e-6);
const STAGE_NAMES = C.stage_names, STAGE_COLORS = C.stage_colors;
const TARGET_PALETTE = C.target_palette;
const tgtColor = tid => TARGET_PALETTE[(IDS.indexOf(tid) + TARGET_PALETTE.length) % TARGET_PALETTE.length];

// 每周期索引：cycle -> [rir rows]
const RIR_BY_CYCLE = {};
for (const r of RIR) (RIR_BY_CYCLE[r.c] = RIR_BY_CYCLE[r.c] || []).push(r);

let cycle = 1, playing = false, timer = null;
const tooltip = document.getElementById("tooltip");
const LAYERS = [
  {key:"max_wedge", label:"最大可扫描体积", color:C.color_maxwedge, on:true},
  {key:"task_wedge", label:"任务扫描扇区", color:C.color_wedge, on:true},
  {key:"dwell", label:"波束驻留指向", color:C.color_dwell, on:true},
  {key:"truth", label:"目标真值轨迹", color:C.color_truth, on:true},
  {key:"targets", label:"目标状态", color:STAGE_COLORS[3], on:true},
  {key:"reco", label:"识别连线", color:STAGE_COLORS[4], on:true},
];

/* ── 小工具 ── */
function fmt(v, d) { return (v === null || v === undefined) ? "-" : v.toFixed(d); }
function showTip(h, ev) {
  tooltip.innerHTML = h; tooltip.style.display = "block";
  const w = tooltip.offsetWidth, hh = tooltip.offsetHeight;
  let x = ev.clientX + 14, y = ev.clientY + 14;
  if (x + w > window.innerWidth - 8) x = ev.clientX - w - 14;
  if (y + hh > window.innerHeight - 8) y = ev.clientY - hh - 14;
  tooltip.style.left = x + "px"; tooltip.style.top = y + "px";
}
function hideTip() { tooltip.style.display = "none"; }
function el(tag, attrs) {
  const e = document.createElementNS("http://www.w3.org/2000/svg", tag);
  for (const k in attrs) e.setAttribute(k, attrs[k]);
  return e;
}
function scale(d0, d1, p0, p1) { return v => p0 + (v - d0) * (p1 - p0) / (d1 - d0); }
// 方位 az(deg) → 单位方向（east=+x, north=+y）：az=0 指东，+az 逆时针向北。
function azUnit(azDeg) { const a = azDeg * Math.PI / 180; return {x: Math.cos(a), y: Math.sin(a)}; }
const CAT_ZH = {ballistic:"弹道目标", near_space:"临近空间", other:"其它", unknown:"未知",
                fighter:"战斗机", bomber:"轰炸机", missile:"导弹"};
function normDeg(a) { a = ((a % 360) + 540) % 360 - 180; return a; }
function azInWedge(az, lo, hi) {
  let width = ((hi - lo) % 360 + 360) % 360; if (width === 0) width = 360;
  let delta = ((az - lo) % 360 + 360) % 360;
  return delta <= width + 1e-6;
}
// 未进任务扫描扇区的成因（俯仰/方位/距离逐项判定）；区分「出硬件体积」与
// 「在体积内但出任务子窗（未被指定则不搜索）」两类门。
function excludeReason(t) {
  if (t.az === null || t.el === null || t.slant === null) return "本周期无视线几何";
  const rs = [];
  if (t.slant > SITE.max_range + 1) rs.push("斜距 " + (t.slant / 1000).toFixed(1) +
    " km 超出配置作用距离 " + (SITE.max_range / 1000).toFixed(0) + " km");
  const hwAzLo = SITE.scan_center_az + SITE.az_min, hwAzHi = SITE.scan_center_az + SITE.az_max;
  const outHwAz = !azInWedge(normDeg(t.az), normDeg(hwAzLo), normDeg(hwAzHi));
  const outHwEl = t.el < SITE.el_min - 1e-6 || t.el > SITE.el_max + 1e-6;
  if (outHwAz) rs.push("方位 " + t.az.toFixed(1) + "° 超出硬件可扫描域 [" +
    hwAzLo.toFixed(0) + "°," + hwAzHi.toFixed(0) + "°]");
  if (t.el < SITE.el_min - 1e-6) rs.push("俯仰 " + t.el.toFixed(2) +
    "° 低于硬件下限 " + SITE.el_min.toFixed(1) + "°（近地/低仰角目标）");
  if (t.el > SITE.el_max + 1e-6) rs.push("俯仰 " + t.el.toFixed(2) + "° 高于硬件上限 " + SITE.el_max.toFixed(1) + "°");
  // 在硬件体积内但被任务子窗排除（非硬件门）：说明它只是未被纳入本次搜索扇区。
  if (!outHwAz && !outHwEl && HAS_TASK_WINDOW) {
    const twAzLo = SITE.scan_center_az + SECTOR.az_min, twAzHi = SITE.scan_center_az + SECTOR.az_max;
    if (!azInWedge(normDeg(t.az), normDeg(twAzLo), normDeg(twAzHi)))
      rs.push("方位 " + t.az.toFixed(1) + "° 在硬件体积内但超出任务扫描扇区 [" +
        twAzLo.toFixed(0) + "°," + twAzHi.toFixed(0) + "°]（未被指定则不搜索）");
    if (t.el < SECTOR.el_min - 1e-6 || t.el > SECTOR.el_max + 1e-6)
      rs.push("俯仰 " + t.el.toFixed(2) + "° 在硬件体积内但超出任务扫描扇区 [" +
        SECTOR.el_min.toFixed(0) + "°," + SECTOR.el_max.toFixed(0) + "°]（未被指定则不搜索）");
  }
  return rs.join("；") || "在任务扫描扇区内、尚未形成航迹";
}

/* ══════════ 俯视地图 ══════════ */
const MAP_W = 980, MAP_H = 600, MAP_M = 56;
let mapSvg = null, mapLayers = {}, mapSx = null, mapSy = null, mapK = 1;

function mapBounds() {
  // 视野拟合「有处置意义」的目标（进入扫描范围及以上，stage>=1）+ 站点，避免被
  // 巨大的 max_range（可达数千 km）压成一个点；扫描扇区仍画到 max_range，由外层
  // <svg> 视口自动裁剪（scan range 的角向覆盖照常显示，半径超界部分裁掉）。
  let xs = [0], ys = [0];  // 站点在原点
  const pick = pred => {
    for (const r of RIR) if (pred(r) && r.x !== null && r.y !== null) { xs.push(r.x); ys.push(r.y); }
  };
  pick(r => r.stage >= 1);
  if (xs.length <= 1) pick(() => true);            // 无 stage>=1：退回全部 RIR 位置
  if (xs.length <= 1) for (const tid of IDS) for (const p of (TRUTH[tid] || [])) { xs.push(p.x); ys.push(p.y); }
  // 纳入扫描扇区弧（按截断显示半径），保证 scan range 角向覆盖完整可见。
  const azLo0 = SITE.scan_center_az + SITE.az_min, azHi0 = SITE.scan_center_az + SITE.az_max;
  const R0 = SITE.display_range;
  for (let i = 0; i <= 24; i++) { const u = azUnit(azLo0 + (azHi0 - azLo0) * i / 24); xs.push(u.x * R0); ys.push(u.y * R0); }
  let x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys);
  const pad = Math.max((x1 - x0), (y1 - y0)) * 0.12 + 1000;
  x0 -= pad; x1 += pad; y0 -= pad; y1 += pad;
  if (x1 - x0 < 1) { x0 -= 500; x1 += 500; }
  if (y1 - y0 < 1) { y0 -= 500; y1 += 500; }
  return {x0, y0, x1, y1};
}

function renderMap() {
  const wrap = document.getElementById("mapWrap");
  wrap.innerHTML = "";
  mapSvg = el("svg", {viewBox: "0 0 " + MAP_W + " " + MAP_H, width: MAP_W, height: MAP_H});
  wrap.appendChild(mapSvg);

  const b = mapBounds();
  const k = Math.min((MAP_W - 2 * MAP_M) / (b.x1 - b.x0), (MAP_H - 2 * MAP_M) / (b.y1 - b.y0));
  const cx = (b.x0 + b.x1) / 2, cy = (b.y0 + b.y1) / 2;
  mapSx = v => MAP_W / 2 + (v - cx) * k;
  mapSy = v => MAP_H / 2 - (v - cy) * k;  // 北在上
  mapK = k;
  const sx = mapSx, sy = mapSy;
  mapLayers = {};

  // 两个扇面（底层：站点 → 弧 → 站点，半透明填充）。半径按截断显示半径画，真实
  // max_range 在 tooltip / 图注标注（避免数千 km 量程压扁近距目标）。
  //   ① 最大可扫描体积（硬件 steerable_volume）：灰蓝、虚线、极淡填充；
  //   ② 任务扫描扇区（子窗 ∩ 体积，用户指定作战搜索扇区）：蓝、实线、淡填充。
  const Rdisp = SITE.display_range;
  const truncated = SITE.display_range < SITE.max_range - 1;
  const wedgePath = (azLoRel, azHiRel) => {
    const azLo = SITE.scan_center_az + azLoRel, azHi = SITE.scan_center_az + azHiRel;
    const steps = 48;
    let d = "M " + sx(0).toFixed(1) + " " + sy(0).toFixed(1);
    for (let i = 0; i <= steps; i++) {
      const u = azUnit(azLo + (azHi - azLo) * i / steps);
      d += " L " + sx(u.x * Rdisp).toFixed(1) + " " + sy(u.y * Rdisp).toFixed(1);
    }
    return d + " Z";
  };
  const truncNote = truncated ? "\n扇区已截断显示至 " + (Rdisp / 1000).toFixed(1) + " km" : "";
  // ① 最大可扫描体积。
  const maxWedge = el("path", {d: wedgePath(SITE.az_min, SITE.az_max), fill: C.color_maxwedge,
    "fill-opacity": 0.06, stroke: C.color_maxwedge, "stroke-width": 1.2,
    "stroke-dasharray": "4 4", "stroke-opacity": 0.8});
  maxWedge.addEventListener("mousemove", ev => showTip(
    "最大可扫描体积（硬件 steerable_volume）\n方位 [" +
    fmt(SITE.scan_center_az + SITE.az_min, 0) + "°," + fmt(SITE.scan_center_az + SITE.az_max, 0) +
    "°] · 俯仰 [" + fmt(SITE.el_min, 0) + "°," + fmt(SITE.el_max, 0) + "°]" +
    "\n配置最大作用距离 " + (SITE.max_range / 1000).toFixed(0) +
    " km（径向粗筛门，非识别距离；真正探测由 SNR 链路预算决定）" + truncNote, ev));
  maxWedge.addEventListener("mouseleave", hideTip);
  mapSvg.appendChild(maxWedge);
  mapLayers["max_wedge"] = [maxWedge];

  // ② 任务扫描扇区（仅当子窗真正收窄时另画；否则与最大扇面重合，只保留最大扇面）。
  mapLayers["task_wedge"] = [];
  if (HAS_TASK_WINDOW) {
    const taskWedge = el("path", {d: wedgePath(SECTOR.az_min, SECTOR.az_max), fill: C.color_wedge,
      "fill-opacity": 0.10, stroke: C.color_wedge, "stroke-width": 1.6, "stroke-opacity": 0.85});
    taskWedge.addEventListener("mousemove", ev => showTip(
      "任务扫描扇区（用户指定作战搜索扇区 = 子窗 ∩ 体积）\n方位 [" +
      fmt(SITE.scan_center_az + SECTOR.az_min, 0) + "°," + fmt(SITE.scan_center_az + SECTOR.az_max, 0) +
      "°] · 俯仰 [" + fmt(SECTOR.el_min, 0) + "°," + fmt(SECTOR.el_max, 0) + "°]" +
      "\n搜索态检测候选按此扇区裁剪；扇区外、体积内的目标仅被『指定识别』时才驻留" + truncNote, ev));
    taskWedge.addEventListener("mouseleave", hideTip);
    mapSvg.appendChild(taskWedge);
    mapLayers["task_wedge"].push(taskWedge);
  }

  // 目标真值轨迹（按 id 分色，faint）。
  mapLayers["truth"] = [];
  for (const tid of IDS) {
    const pts = TRUTH[tid] || [];
    if (pts.length < 2) continue;
    const poly = el("polyline", {points: pts.map(p => sx(p.x).toFixed(1) + "," + sy(p.y).toFixed(1)).join(" "),
      fill: "none", stroke: tgtColor(tid), "stroke-width": 1.4, "stroke-dasharray": "6 4", opacity: 0.55});
    mapSvg.appendChild(poly);
    mapLayers["truth"].push(poly);
  }

  // 站点标记（三角）。
  const st = el("polygon", {points: (sx(0)) + "," + (sy(0) - 8) + " " + (sx(0) - 7) + "," + (sy(0) + 6) +
    " " + (sx(0) + 7) + "," + (sy(0) + 6), fill: C.color_site, stroke: "#fff", "stroke-width": 1.5});
  st.addEventListener("mousemove", ev => showTip(
    "RIR 站点\nLLA (" + fmt(SITE.lat, 5) + ", " + fmt(SITE.lon, 5) + ", " + fmt(SITE.alt, 0) + " m)", ev));
  st.addEventListener("mouseleave", hideTip);
  mapSvg.appendChild(st);

  // 动态层（每周期重画：目标状态点 + 识别连线 + 驻留指向）。
  mapLayers["dynamic"] = el("g", {});
  mapSvg.appendChild(mapLayers["dynamic"]);

  applyLayerVisibility();
  updateMap();
}

function applyLayerVisibility() {
  for (const key of Object.keys(mapLayers)) {
    if (!Array.isArray(mapLayers[key])) continue;
    const def = LAYERS.find(l => l.key === key);
    const on = def ? def.on : true;
    for (const e of mapLayers[key]) e.style.display = on ? "" : "none";
  }
}

function updateMap() {
  const g = mapLayers["dynamic"];
  if (!g) return;
  g.innerHTML = "";
  const sx = mapSx, sy = mapSy;
  const on = LAYERS.filter(l => l.on).map(l => l.key);
  const rows = RIR_BY_CYCLE[cycle] || [];

  // 波束驻留指向（当前周期天线指向 = 库内驻留调度器给的 dwell 中心）：从站点射出
  // 的一条实线。扫描时逐拍在扇区里推进；被『指定识别』时对准指定目标。
  if (on.includes("dwell") && rows.length && rows[0].dwell_az !== null) {
    const u = azUnit(rows[0].dwell_az);
    const r = SITE.display_range;
    const x2 = sx(u.x * r), y2 = sy(u.y * r);
    const ray = el("line", {x1: sx(0), y1: sy(0), x2: x2, y2: y2,
      stroke: C.color_dwell, "stroke-width": 2, opacity: 0.9});
    ray.addEventListener("mousemove", ev => showTip(
      "波束驻留指向（当前周期天线指向）\n方位 " + fmt(rows[0].dwell_az, 1) + "° 俯仰 " +
      fmt(rows[0].dwell_el, 1) + "°\n" + (rows.some(z => z.desig) ?
      "对准指定识别目标" : "按扫描策略逐拍推进"), ev));
    ray.addEventListener("mouseleave", hideTip);
    g.appendChild(ray);
    // 指向端箭头（小三角，指示方向）。
    const ang = Math.atan2(y2 - sy(0), x2 - sx(0));
    const ah = 9, aw = 4;
    const bx = x2 - ah * Math.cos(ang), by = y2 - ah * Math.sin(ang);
    g.appendChild(el("polygon", {points:
      x2.toFixed(1) + "," + y2.toFixed(1) + " " +
      (bx - aw * Math.sin(ang)).toFixed(1) + "," + (by + aw * Math.cos(ang)).toFixed(1) + " " +
      (bx + aw * Math.sin(ang)).toFixed(1) + "," + (by - aw * Math.cos(ang)).toFixed(1),
      fill: C.color_dwell, opacity: 0.9}));
  }

  for (const t of rows) {
    if (t.x === null || t.y === null) continue;
    const px = sx(t.x), py = sy(t.y);
    // 识别连线（阶段 4）：站点 → 目标 + 标签。
    if (t.stage >= 4 && on.includes("reco")) {
      g.appendChild(el("line", {x1: sx(0), y1: sy(0), x2: px, y2: py,
        stroke: STAGE_COLORS[4], "stroke-width": 2.2, opacity: 0.9}));
      const lab = el("text", {x: (sx(0) + px) / 2, y: (sy(0) + py) / 2 - 4, "font-size": 11,
        fill: STAGE_COLORS[4], "text-anchor": "middle"});
      lab.textContent = (CAT_ZH[t.cat] || t.cat) + (t.model ? "/" + t.model : "") +
        " " + fmt(t.conf, 2);
      g.appendChild(lab);
    }
    if (!on.includes("targets")) continue;
    const col = STAGE_COLORS[t.stage];
    // 阶段 1（进入扫描范围）用空心，其余实心。
    const mk = el("circle", {cx: px, cy: py, r: 6, fill: t.stage === 1 ? "#fff" : col,
      stroke: col, "stroke-width": 2.2});
    const reason = t.stage === 0 ? excludeReason(t) : "";
    mk.addEventListener("mousemove", ev => showTip(
      "目标 T" + t.id + (t.name ? "（" + t.name + "）" : "") + " · " + STAGE_NAMES[t.stage] +
      "\n方位 " + fmt(t.az, 1) + "° 俯仰 " + fmt(t.el, 1) + "° 斜距 " +
      (t.slant !== null ? (t.slant / 1000).toFixed(1) + " km" : "-") +
      "\n速度 " + fmt(t.spd, 0) + " m/s" +
      (reason ? "\n未进扫描体积：" + reason : "") +
      (t.reco && t.reco !== "disabled" ? "\n识别 " + t.reco + " 置信 " + fmt(t.conf, 2) : "") +
      (t.desig ? "\n指定识别执行中" : ""), ev));
    mk.addEventListener("mouseleave", hideTip);
    g.appendChild(mk);
    const tl = el("text", {x: px + 9, y: py - 9, "font-size": 11, fill: col});
    tl.textContent = "T" + t.id;
    g.appendChild(tl);
  }
}

/* ══════════ 斜距 / 高度剖面 ══════════ */
let profSvg = null, profSx = null, profY0 = 0, profY1 = 0;
function renderProfile() {
  const wrap = document.getElementById("profileWrap");
  wrap.innerHTML = "";
  const W = 980, H = 320, MX = 64, MY = 26;
  profSvg = el("svg", {viewBox: "0 0 " + W + " " + H, width: W, height: H});
  wrap.appendChild(profSvg);
  const x0 = MX, x1 = W - MX, y0 = MY, y1 = H - MY - 16;
  profY0 = y0; profY1 = y1;
  const sx = scale(1, Math.max(2, N), x0, x1);
  profSx = sx;

  // Y 域：拟合斜距/高度数据本身（不被配置 max_range 撑爆）；max_range 若在域内
  // 画参考线，超出则在顶栏文字标注（同扇区截断口径）。
  let vmax = 0;
  for (const tid of IDS) {
    for (const p of (SERIES[String(tid)].range || [])) if (p.v !== null) vmax = Math.max(vmax, p.v);
    for (const p of (SERIES[String(tid)].alt || [])) if (p.v !== null) vmax = Math.max(vmax, p.v);
  }
  vmax = vmax > 0 ? vmax * 1.12 : SITE.display_range;
  const sy = scale(0, vmax, y1, y0);

  // 坐标轴 + 网格。
  const ticks = [1]; const stepc = Math.max(1, Math.round(N / 10));
  for (let t = stepc; t <= N; t += stepc) ticks.push(t);
  for (const t of ticks) {
    profSvg.appendChild(el("line", {x1: sx(t), y1: y0, x2: sx(t), y2: y1, class: "grid"}));
    const tx = el("text", {x: sx(t), y: y1 + 14, "text-anchor": "middle", "font-size": 10, fill: "#666"});
    tx.textContent = String(t); profSvg.appendChild(tx);
  }
  for (let i = 0; i <= 4; i++) {
    const v = vmax * i / 4;
    profSvg.appendChild(el("line", {x1: x0, y1: sy(v), x2: x1, y2: sy(v), class: "grid"}));
    const ty = el("text", {x: x0 - 6, y: sy(v) + 3, "text-anchor": "end", "font-size": 10, fill: "#666"});
    ty.textContent = (v / 1000).toFixed(0) + "km"; profSvg.appendChild(ty);
  }
  profSvg.appendChild(el("line", {x1: x0, y1: y0, x2: x0, y2: y1, class: "axis"}));
  profSvg.appendChild(el("line", {x1: x0, y1: y1, x2: x1, y2: y1, class: "axis"}));

  // 配置最大作用距离参考线（在纵轴域内才画；超域只在顶栏文字标注）。
  if (SITE.max_range <= vmax) {
    profSvg.appendChild(el("line", {x1: x0, y1: sy(SITE.max_range), x2: x1, y2: sy(SITE.max_range),
      stroke: "#c62828", "stroke-width": 1, "stroke-dasharray": "2 3", opacity: 0.7}));
    const mr = el("text", {x: x1 - 2, y: sy(SITE.max_range) - 3, "text-anchor": "end", "font-size": 10, fill: "#c62828"});
    mr.textContent = "配置最大作用距离 " + (SITE.max_range / 1000).toFixed(0) + " km";
    profSvg.appendChild(mr);
  } else {
    const mr = el("text", {x: x1 - 2, y: y0 - 8, "text-anchor": "end", "font-size": 10, fill: "#c62828"});
    mr.textContent = "配置最大作用距离 " + (SITE.max_range / 1000).toFixed(0) + " km（远超纵轴域，未画线）";
    profSvg.appendChild(mr);
  }

  // 逐目标：斜距实线（阶段配色点）+ 高度虚线 + 事件标记。
  for (const tid of IDS) {
    const s = SERIES[String(tid)];
    const col = tgtColor(tid);
    if (s.range && s.range.length) {
      const poly = el("polyline", {points: s.range.map(p => sx(p.c).toFixed(1) + "," + sy(p.v).toFixed(1)).join(" "),
        fill: "none", stroke: col, "stroke-width": 1.8});
      profSvg.appendChild(poly);
      for (const p of s.range) {
        const dot = el("circle", {cx: sx(p.c), cy: sy(p.v), r: 2.2, fill: STAGE_COLORS[p.stage]});
        profSvg.appendChild(dot);
      }
    }
    if (s.alt && s.alt.length) {
      profSvg.appendChild(el("polyline", {points: s.alt.map(p => sx(p.c).toFixed(1) + "," + sy(p.v).toFixed(1)).join(" "),
        fill: "none", stroke: col, "stroke-width": 1.4, "stroke-dasharray": "5 4", opacity: 0.75}));
    }
    // 事件标记：▲ 首探测 ◆ 首确认 ★ 首识别（落在斜距线上）。
    const rangeAt = c => { const p = (s.range || []).find(q => q.c === c); return p ? p.v : null; };
    const mark = (c, glyph, color) => {
      if (c === null) return;
      const v = rangeAt(c); if (v === null) return;
      const tx = el("text", {x: sx(c), y: sy(v) - 7, "text-anchor": "middle", "font-size": 13, fill: color});
      tx.textContent = glyph; profSvg.appendChild(tx);
    };
    mark(s.first_detect, "▲", STAGE_COLORS[2]);
    mark(s.first_confirm, "◆", STAGE_COLORS[3]);
    mark(s.first_reco, "★", STAGE_COLORS[4]);
    // 目标标签（斜距线末端）。
    if (s.range && s.range.length) {
      const last = s.range[s.range.length - 1];
      const tl = el("text", {x: sx(last.c) + 4, y: sy(last.v) + 3, "font-size": 10, fill: col});
      tl.textContent = "T" + tid; profSvg.appendChild(tl);
    }
  }
}

/* ══════════ 时间游标 ══════════ */
let cursorEl = null;
function updateCursor() {
  if (cursorEl) { cursorEl.remove(); cursorEl = null; }
  if (profSvg && profSx) {
    cursorEl = el("line", {x1: profSx(cycle), y1: profY0, x2: profSx(cycle), y2: profY1, class: "cursor"});
    profSvg.appendChild(cursorEl);
  }
}

/* ══════════ 图层开关 + 图例 ══════════ */
function renderLayerToggles() {
  const box = document.getElementById("layerToggles");
  box.innerHTML = "";
  for (const l of LAYERS) {
    const lab = document.createElement("label");
    lab.innerHTML = '<span class="swatch" style="background:' + l.color + '"></span>' + l.label;
    const cb = document.createElement("input");
    cb.type = "checkbox"; cb.checked = l.on;
    cb.addEventListener("change", () => { l.on = cb.checked; applyLayerVisibility(); updateMap(); });
    lab.prepend(cb);
    box.appendChild(lab);
  }
  const leg = document.getElementById("stageLegend");
  leg.innerHTML = "处置阶段：" + STAGE_NAMES.map((n, i) =>
    '<span><span class="swatch" style="background:' + STAGE_COLORS[i] + '"></span>' + n + "</span>").join("");
}

/* ══════════ 播放控制 ══════════ */
function setCycle(c) {
  cycle = Math.max(1, Math.min(N, c));
  document.getElementById("cycleSlider").value = cycle;
  document.getElementById("cycleReadout").textContent = "cycle " + cycle + " / " + N;
  // 库上报本周期「实际有效目标最大斜距」（RirCycleResult.max_detected_slant_range_m）。
  const ls = LIB_SLANT[String(cycle)];
  document.getElementById("libSlant").textContent =
    (ls !== undefined && ls > 0) ? "· 实际有效最大斜距 " + (ls / 1000).toFixed(1) + " km" : "· 本周期无航迹";
  updateMap(); updateCursor();
}
function play() {
  if (playing) return;
  playing = true;
  const btn = document.getElementById("playBtn");
  btn.textContent = "⏸ 暂停"; btn.classList.add("active");
  const step = () => { if (!playing) return; if (cycle >= N) { pause(); return; } setCycle(cycle + 1); };
  const speed = parseInt(document.getElementById("speedSel").value);
  timer = setInterval(step, Math.max(20, Math.round(1000 / speed)));
}
function pause() {
  playing = false;
  if (timer) clearInterval(timer);
  timer = null;
  const btn = document.getElementById("playBtn");
  btn.textContent = "▶ 播放"; btn.classList.remove("active");
}

/* ══════════ 初始化 ══════════ */
function init() {
  if (!N) {
    document.body.insertAdjacentHTML("afterbegin", "<p style='color:#c00'>数据为空（rir_targets.csv 缺失或无周期）</p>");
    return;
  }
  renderLayerToggles();
  const note = document.getElementById("wedgeNote");
  const trunc = SITE.display_range < SITE.max_range - 1;
  const maxTxt = "最大可扫描体积（硬件）：方位 [" + (SITE.scan_center_az + SITE.az_min).toFixed(0) +
    "°," + (SITE.scan_center_az + SITE.az_max).toFixed(0) + "°] · 俯仰 [" + SITE.el_min.toFixed(0) +
    "°," + SITE.el_max.toFixed(0) + "°]";
  const taskTxt = HAS_TASK_WINDOW
    ? " · <span style='color:" + C.color_wedge + "'>任务扫描扇区（用户指定）：方位 [" +
      (SITE.scan_center_az + SECTOR.az_min).toFixed(0) + "°," +
      (SITE.scan_center_az + SECTOR.az_max).toFixed(0) + "°] · 俯仰 [" + SECTOR.el_min.toFixed(0) +
      "°," + SECTOR.el_max.toFixed(0) + "°]</span>"
    : " · 任务扫描子窗未收窄（= 最大体积，两扇面重合）";
  note.innerHTML = maxTxt + taskTxt +
    "<br>配置 max_range_m = <b>" + (SITE.max_range / 1000).toFixed(0) +
    " km</b>（径向粗筛门，非探测距离） · 实际探测距离看库上报 <b>max_detected_slant_range_m</b>" +
    "（顶栏『实际有效最大斜距』，随周期变化）" +
    (trunc ? " · 扇面已截断显示至 <b>" + (SITE.display_range / 1000).toFixed(1) +
      " km</b>（否则近距目标会被压成一点）" : "");
  renderMap();
  renderProfile();
  document.getElementById("cycleSlider").addEventListener("input", e => { pause(); setCycle(parseInt(e.target.value)); });
  document.getElementById("playBtn").addEventListener("click", () => playing ? pause() : play());
  document.getElementById("speedSel").addEventListener("change", () => { if (playing) { pause(); play(); } });
  setCycle(1);
}
init();
</script>
</body>
</html>
"""


# ═══════════════════════════════════════════════════════════════════
# 校验（--check）
# ═══════════════════════════════════════════════════════════════════

EXPECTED_HEADERS = {
    "rir_site.csv": "site_lat_deg,site_lon_deg,site_alt_m,scan_center_az_deg,"
                    "scan_center_el_deg,az_min_deg,az_max_deg,el_min_deg,el_max_deg,max_range_m,"
                    "scan_win_az_min_deg,scan_win_az_max_deg,scan_win_el_min_deg,scan_win_el_max_deg",
    "rir_targets.csv": "cycle,t_sec,target_id,target_name,present_in_input,has_track,status,"
                       "look_az_deg,look_el_deg,slant_range_m,pos_lat_deg,pos_lon_deg,pos_alt_m,"
                       "speed_mps,recognition_state,target_category,target_model,confidence,"
                       "designation_active,dwell_center_az_deg,dwell_center_el_deg,"
                       "cycle_max_detected_slant_m,cycle_snr_db",
    "target_truth.csv": "cycle,t_sec,target_id,entity_type,lat_deg,lon_deg,alt_m,rcs",
}


def validate_data(data_dir):
    """回归校验：CSV 表头 + 基本不变量（站点唯一、周期非空、每周期目标行数一致）。"""
    problems = []
    base = data_dir.rstrip("/")
    for name, expected in EXPECTED_HEADERS.items():
        path = base + "/" + name
        try:
            with open(path, newline="") as fh:
                header = fh.readline().strip()
        except OSError:
            problems.append("%s 缺失（本工具为 RIR 场景专用，需先跑 RIR 场景可执行）" % name)
            continue
        if header != expected:
            problems.append("%s 表头不匹配：%s" % (name, header))
    if problems:
        return problems

    site_rows = load_csv(base + "/rir_site.csv")
    if not site_rows or len(site_rows) != 1:
        problems.append("rir_site.csv 应恰好一行站点几何（实际 %d 行）"
                        % (len(site_rows) if site_rows else 0))
    data = build_data(base)
    if data is None or not data["meta"]["cycles"]:
        problems.append("rir_targets.csv 无有效周期数据")
        return problems
    # 每周期目标行数应一致（逐周期逐目标落盘）。
    per_cycle = {}
    for r in load_csv(base + "/rir_targets.csv") or []:
        c = inum(r, "cycle")
        if c is not None:
            per_cycle[c] = per_cycle.get(c, 0) + 1
    counts = set(per_cycle.values())
    if len(counts) > 1:
        problems.append("rir_targets.csv 各周期目标行数不一致：%s" % sorted(counts))
    return problems


# ═══════════════════════════════════════════════════════════════════
# 入口
# ═══════════════════════════════════════════════════════════════════

def render_html(data, data_dir):
    """把装配数据渲染为自包含 HTML 字符串。"""
    data_json = json.dumps(data, ensure_ascii=False, separators=(",", ":")).replace("</", "<\\/")
    const = {
        "stage_names": STAGE_NAMES,
        "stage_colors": STAGE_COLORS,
        "target_palette": TARGET_PALETTE,
        "color_site": COLOR_SITE,
        "color_wedge": COLOR_WEDGE,
        "color_maxwedge": COLOR_MAXWEDGE,
        "color_dwell": COLOR_DWELL,
        "color_truth": COLOR_TRUTH,
    }
    const_json = json.dumps(const, ensure_ascii=False, separators=(",", ":")).replace("</", "<\\/")
    cycles = data["meta"]["cycles"]
    targets = data["meta"]["targets"]
    title = str(targets) + " 目标 · " + str(cycles) + " 周期"
    page = (HTML_TEMPLATE
            .replace("__TITLE__", html.escape(title))
            .replace("__DATADIR__", html.escape(data_dir))
            .replace("__TARGETS__", str(targets))
            .replace("__CYCLES__", str(cycles))
            .replace("__DATA_JSON__", data_json)
            .replace("__CONST_JSON__", const_json))
    return page


def main():
    parser = argparse.ArgumentParser(
        description="从 RIR 场景 CSV 导出构建扫描-识别可视化 HTML",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    parser.add_argument("data_dir", help="RIR 场景可执行输出目录（examples/log/<scene>/）")
    parser.add_argument("--out", default=None, help="输出 HTML 路径（默认 <data_dir>/rir_scan_viewer.html）")
    parser.add_argument("--check", action="store_true", help="仅校验数据（不生成 HTML），供回归")
    args = parser.parse_args()

    if args.check:
        problems = validate_data(args.data_dir)
        if problems:
            for p in problems:
                sys.stderr.write("CHECK FAIL: %s\n" % p)
            sys.exit(1)
        data = build_data(args.data_dir)
        print("CHECK PASS: %s（%d 周期，%d 目标）"
              % (args.data_dir, data["meta"]["cycles"], data["meta"]["targets"]))
        return

    data = build_data(args.data_dir)
    if data is None:
        sys.stderr.write("错误：无法从 %s 读取 rir_site.csv（本工具为 RIR 场景专用）\n" % args.data_dir)
        sys.exit(1)
    if not data["meta"]["cycles"]:
        sys.stderr.write("错误：rir_targets.csv 无有效周期数据（%s）\n" % args.data_dir)
        sys.exit(1)

    out_path = args.out or args.data_dir.rstrip("/") + "/rir_scan_viewer.html"
    with open(out_path, "w", encoding="utf-8") as fh:
        fh.write(render_html(data, args.data_dir))
    print("已生成 RIR 查看器：%s" % out_path)
    print("  数据：%s（%d 周期，%d 目标）" % (args.data_dir, data["meta"]["cycles"], data["meta"]["targets"]))
    print("  打开方式：open %s" % out_path)


if __name__ == "__main__":
    main()
