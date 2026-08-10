#!/usr/bin/env python3
"""
build_viewer.py — 从示例（component_entt / component_attachment）的 CSV 导出
构建交互式 HTML 查看器。

用法：
  python3 build_viewer.py <data_dir> [--out viewer.html]

data_dir 由示例生成（--output-dir），统一可视化契约 v2：
  platform_track（多机，aircraft_id 列）/ target_truth（entity_type 列）/
  ar_tracks / eos_detections / esr_hypotheses / fused_tracks / route_plan
  （多机）/ waypoint_events / zones（巡逻区域多边形/圆）。
传感器/融合/航点文件缺省可缺省——查看器自动跳过对应图层。

产物为单文件自包含 HTML（vanilla JS + SVG，无 CDN、离线可用），提供：
  - 时间轴：周期滑杆 + 播放/暂停 + 速度调节 + 图层开关，跨面板联动游标
  - 俯视地图：多机轨迹（按 aircraft_id 分色）、巡逻区域（多边形/圆）、
    航路点、目标真值（空中/地面不同线型与标记）、融合目标、
    当前周期 AR 航迹点 / EOS 距离-方位射线 / ESR 方位线，悬停详情
  - 传感器日志时间线（Gantt）：真值目标与融合键在各通道（AR/EOS/ESR/融合）
    的活跃色段 + 新目标/消失标记，短命键聚合为"churn"行
  - 融合置信度曲线 + 各通道采样量堆叠面积
  - 飞行剖面：高度 / 速度 / 航向（多机分线）

仅依赖 Python 标准库（csv/json/argparse）。
"""

import argparse
import csv
import html
import json
import math
import os
import sys

# ═══════════════════════════════════════════════════════════════════
# 常量
# ═══════════════════════════════════════════════════════════════════

EARTH_RADIUS_M = 6378137.0
DEFAULT_MIN_SAMPLES = 20  # 融合键进入时间线/置信度图的采样数下限（查看器内可调）
ESR_RAY_LENGTH_M = 25000.0  # ESR 方位线显示长度（m，场景约 20 km）

# 通道配色（与查看器图例一致）
COLOR_PLATFORM = "#1f77b4"
COLOR_AR = "#1f77b4"
COLOR_EOS = "#2ca02c"
COLOR_ESR = "#ff7f0e"
COLOR_FUSED = "#d62728"
COLOR_CHURN = "#999999"
COLOR_ZONE = "#666666"
TRUTH_PALETTE = ["#d62728", "#2ca02c", "#9467bd"]
# 多机飞行器配色（tab10 风格；按 aircraft_id 升序取色，与轨迹/航路/剖面一致）。
AIRCRAFT_PALETTE = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd",
                    "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf"]

CSV_FILES = [
    "platform_track.csv",
    "target_truth.csv",
    "ar_tracks.csv",
    "eos_detections.csv",
    "esr_hypotheses.csv",
    "fused_tracks.csv",
    "route_plan.csv",
    "waypoint_events.csv",
    "zones.csv",
]


# ═══════════════════════════════════════════════════════════════════
# CSV 读取与数值解析
# ═══════════════════════════════════════════════════════════════════

def load_csv(path):
    """读取 CSV 为 dict 列表；空字段转为 None。文件缺失返回 None。"""
    try:
        with open(path, newline="") as f:
            return list(csv.DictReader(f))
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


# ═══════════════════════════════════════════════════════════════════
# 本地 ENU 投影（以首行平台位置为原点，与 orbit_visualize 同量纲）
# ═══════════════════════════════════════════════════════════════════

def project_enu(rows, origin_lat=None, origin_lon=None, lat_key="lat_deg", lon_key="lon_deg"):
    """把 lat/lon（度）行列表投影为 {x, y}（m，东/北）。

    原点缺省时取首个非空行（向后搜索，容忍首行空字段）；显式传入
    origin_lat/origin_lon 时所有数据集共享同一原点（平台初始位置），
    保证 platform/truth/fused/route 各图层落在同一本地系。空字段行返回 None。
    """
    if not rows:
        return []
    if origin_lat is None or origin_lon is None:
        origin_lat = origin_lon = None
        for r in rows:
            lat = f(r, lat_key)
            lon = f(r, lon_key)
            if lat is not None and lon is not None:
                origin_lat, origin_lon = lat, lon
                break
        if origin_lat is None:
            return [None] * len(rows)
    lat0 = math.radians(origin_lat)
    lon0 = math.radians(origin_lon)
    out = []
    for r in rows:
        lat = f(r, lat_key)
        lon = f(r, lon_key)
        if lat is None or lon is None:
            out.append(None)
            continue
        x = (math.radians(lon) - lon0) * EARTH_RADIUS_M * math.cos(lat0)
        y = (math.radians(lat) - lat0) * EARTH_RADIUS_M
        out.append({"x": x, "y": y})
    return out


# ═══════════════════════════════════════════════════════════════════
# 数据装配
# ═══════════════════════════════════════════════════════════════════

def build_data(data_dir):
    """读取全部 CSV 并装配为内嵌 JSON 的字典。缺失文件对应键为 None。"""
    def read(name):
        return load_csv(data_dir + "/" + name)

    platform = read("platform_track.csv")
    truth = read("target_truth.csv")
    ar = read("ar_tracks.csv")
    eos = read("eos_detections.csv")
    esr = read("esr_hypotheses.csv")
    fused = read("fused_tracks.csv")
    route = read("route_plan.csv")
    wp_events = read("waypoint_events.csv")
    zones = read("zones.csv")

    meta = {"model": platform[0]["model"] if platform else "unknown"}
    if platform:
        # 多机契约：原点/模型取 aircraft_id 最小的机（主平台）首行。
        ac_min = min(int(r["aircraft_id"]) for r in platform)
        first_ac = next(r for r in platform if int(r["aircraft_id"]) == ac_min)
        meta["cycles"] = max(int(r["cycle"]) for r in platform)
        meta["aircraft"] = len({int(r["aircraft_id"]) for r in platform})
        meta["origin_lat"] = f(first_ac, "lat_deg")
        meta["origin_lon"] = f(first_ac, "lon_deg")
    else:
        meta["cycles"] = 0
        meta["aircraft"] = 0
    # 统一 ENU 原点 = 主平台初始位置：全部图层共享同一本地系（地图坐标才可叠加）。
    origin = (meta.get("origin_lat"), meta.get("origin_lon"))

    data = {"meta": meta}

    # 平台轨迹（含 ENU 投影；多机按 aircraft_id 分组着色）。
    if platform:
        enu = project_enu(platform, *origin)
        data["platform"] = [
            {
                "c": int(r["cycle"]),
                "t": f(r, "t_sec"),
                "ac": int(r["aircraft_id"]),
                "x": enu[i]["x"],
                "y": enu[i]["y"],
                "alt": f(r, "alt_m"),
                "hdg": f(r, "heading_deg"),
                "spd": f(r, "speed_mps"),
                "wp": to_int(r, "wp_index"),
                "wpc": to_int(r, "wp_count"),
            }
            for i, r in enumerate(platform)
        ]

    # 目标真值（按 id 分组轨迹；entity_type 区分空中/地面）。
    if truth:
        truth_enu = project_enu(truth, *origin)
        data["truth"] = [
            {
                "c": int(r["cycle"]),
                "id": int(float(r["target_id"])),
                "et": r.get("entity_type") or "air",
                "x": truth_enu[i]["x"],
                "y": truth_enu[i]["y"],
                "alt": f(r, "alt_m"),
                "rcs": f(r, "rcs"),
            }
            for i, r in enumerate(truth)
        ]

    # AR 航迹（雷达局部 ENU 直接叠加到平台世界位姿）。
    if ar:
        data["ar"] = [
            {
                "c": int(r["cycle"]),
                "key": int(float(r["key"])),
                "tid": int(float(r["target_id"])),
                "st": r["status"],
                "x": f(r, "pos_x_m"),
                "y": f(r, "pos_y_m"),
                "spd": f(r, "speed_mps"),
                "rcs": f(r, "rcs"),
            }
            for r in ar
        ]

    # EOS 探测（平台局部方位，az 0 = 东）。
    if eos:
        data["eos"] = [
            {
                "c": int(r["cycle"]),
                "det": int(float(r["det_id"])),
                "tid": to_int(r, "target_id"),
                "range": f(r, "range_m"),
                "az": f(r, "az_deg"),
                "el": f(r, "el_deg"),
                "snr": f(r, "snr_db"),
                "detected": int(r["detected"]) == 1,
            }
            for r in eos
        ]

    # ESR 假设（方位线，az 0 = 东）。
    if esr:
        data["esr"] = [
            {
                "c": int(r["cycle"]),
                "hyp": int(float(r["hyp_id"])),
                "az": f(r, "bearing_az_deg"),
                "el": f(r, "bearing_el_deg"),
                "conf": f(r, "confidence"),
                "mode": r["mode"],
                "threat": r["threat_level"],
            }
            for r in esr
        ]

    # 融合态势（含通道采样数与位置量测；位置/方位可为空）。
    if fused:
        fused_enu = project_enu(fused, *origin)
        data["fused"] = [
            {
                "c": int(r["cycle"]),
                "key": int(float(r["key"])),
                "conf": f(r, "confidence"),
                "luc": to_int(r, "last_update_cycle"),
                "a": to_int(r, "ar_samples"),
                "e": to_int(r, "esr_samples"),
                "o": to_int(r, "eos_samples"),
                "x": fused_enu[i]["x"] if fused_enu[i] else None,
                "y": fused_enu[i]["y"] if fused_enu[i] else None,
                "alt": f(r, "alt_m"),
                "bz": f(r, "bearing_az_deg"),
            }
            for i, r in enumerate(fused)
        ]

    # 航路点（多机按 aircraft_id 分组着色）与航点完成事件。
    if route:
        route_enu = project_enu(route, *origin)
        data["waypoints"] = [
            {
                "i": int(r["index"]),
                "ac": to_int(r, "aircraft_id"),
                "x": route_enu[i]["x"],
                "y": route_enu[i]["y"],
                "alt": f(r, "alt_m"),
                "spd": f(r, "speed_mps"),
                "r": f(r, "radius_m"),
            }
            for i, r in enumerate(route)
        ]
    if wp_events:
        data["wp_events"] = [
            {
                "t": f(r, "t_sec"),
                "c": int(math.ceil(f(r, "t_sec") - 1e-9)) if f(r, "t_sec") is not None else None,
                "i": to_int(r, "waypoint_index"),
                "inter": int(r["intermediate"]) == 1,
                "gate": r["gate"],
                "d": f(r, "distance_m"),
                "ct": f(r, "cross_track_m"),
                "at": f(r, "along_track_m"),
                "thr": f(r, "threshold_m"),
            }
            for r in wp_events
        ]

    # 巡逻区域（zones.csv）：polygon 每顶点一行（同 name 聚合成闭合线），
    # circle 一行（radius_m 有效）。与各图层共享统一 ENU 原点。
    if zones:
        zone_groups = {}
        for r in zones:
            name = r["name"]
            z = zone_groups.setdefault(name, {"kind": r["kind"], "pts": [], "radius": f(r, "radius_m")})
            enu = project_enu([r], *origin)
            z["pts"].append({"x": enu[0]["x"], "y": enu[0]["y"]})
        data["zones"] = [
            {"name": name, "kind": z["kind"], "pts": z["pts"], "radius": z["radius"]}
            for name, z in zone_groups.items()
        ]

    return data


# ═══════════════════════════════════════════════════════════════════
# HTML 模板（vanilla JS + SVG，自包含）
# ═══════════════════════════════════════════════════════════════════

HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>Behavior Layer 可视化 — {title}</title>
<style>
  :root {{
    --bg:#fafafa; --panel:#ffffff; --ink:#222; --muted:#888;
    --ar:{COLOR_AR}; --eos:{COLOR_EOS}; --esr:{COLOR_ESR}; --fused:{COLOR_FUSED};
  }}
  * {{ box-sizing:border-box; }}
  body {{ margin:0; padding:16px 20px; background:var(--bg); color:var(--ink);
         font:14px/1.5 -apple-system,"PingFang SC","Helvetica Neue",Arial,sans-serif; }}
  h1 {{ font-size:18px; margin:0 0 4px; }}
  .sub {{ color:var(--muted); font-size:12px; margin-bottom:12px; }}
  .panel {{ background:var(--panel); border:1px solid #e3e3e3; border-radius:8px;
           padding:10px 14px; margin-bottom:14px; }}
  .panel h2 {{ font-size:14px; margin:0 0 6px; color:#444; }}
  #controls {{ display:flex; flex-wrap:wrap; gap:14px; align-items:center; }}
  #controls label {{ font-size:13px; color:#555; }}
  #cycleSlider {{ width:420px; }}
  .btn {{ padding:4px 14px; border:1px solid #bbb; border-radius:5px; background:#fff;
         cursor:pointer; font-size:13px; }}
  .btn:hover {{ background:#f0f0f0; }}
  .btn.active {{ background:var(--fused); color:#fff; border-color:var(--fused); }}
  #cycleReadout {{ font-variant-numeric:tabular-nums; min-width:120px; }}
  .layers {{ display:flex; flex-wrap:wrap; gap:10px; }}
  .layers label {{ display:flex; align-items:center; gap:4px; cursor:pointer; }}
  .swatch {{ width:11px; height:11px; border-radius:2px; display:inline-block; }}
  svg {{ display:block; width:100%; height:auto; }}
  .axis {{ stroke:#ddd; }}
  .grid {{ stroke:#eee; }}
  .tooltip {{ position:fixed; display:none; pointer-events:none; background:rgba(20,20,20,.92);
             color:#fff; font-size:12px; padding:6px 9px; border-radius:5px; z-index:50;
             max-width:340px; white-space:pre-line; }}
  .cursor {{ stroke:#333; stroke-dasharray:4 3; }}
  .legend {{ font-size:12px; color:#555; margin-top:4px; }}
  .legend span {{ margin-right:12px; }}
  #tlFilter {{ width:70px; }}
</style>
</head>
<body>
<h1>Behavior Layer 交互式可视化</h1>
<div class="sub">数据目录：{data_dir}（平台/AR/EOS/ESR/融合/航路/航点事件/巡逻区域） · 飞行器：<b>{aircraft} 架</b> · 平台动力学：<b>{model}</b></div>

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
    <input id="cycleSlider" type="range" min="1" max="{cycles}" value="1">
    <span id="cycleReadout">cycle 1 / {cycles}</span>
    <span class="layers" id="layerToggles"></span>
  </div>
</div>

<div class="panel">
  <h2>俯视地图（本地 ENU，东/北，m；原点 = 平台初始位置）</h2>
  <div id="mapWrap"></div>
</div>

<div class="panel">
  <h2>传感器日志时间线（色段 = 该通道在对应周期看到目标；
      <span style="color:var(--ar)">AR 航迹</span> /
      <span style="color:var(--eos)">EOS 探测</span> /
      <span style="color:var(--esr)">ESR 假设</span> /
      <span style="color:var(--fused)">融合活跃</span>；
      ▲ 新目标 ▼ 消失；短命融合键聚合为 churn 行）</h2>
  <div style="margin-bottom:6px">
    <label>融合键采样数下限
      <input id="tlFilter" type="number" min="1" value="{min_samples}">
    </label>
    <span class="sub" id="tlFilterHint"></span>
  </div>
  <div id="timelineWrap"></div>
</div>

<div class="panel">
  <h2>融合置信度（稳定键）与通道采样量</h2>
  <div id="confWrap"></div>
</div>

<div class="panel">
  <h2>飞行剖面（高度 / 速度 / 航向）</h2>
  <div id="profileWrap"></div>
</div>

<div class="tooltip" id="tooltip"></div>

<script id="viz-data" type="application/json">{data_json}</script>
<script>
"use strict";
/* ═══════════════ 数据与状态 ═══════════════ */
const DATA = JSON.parse(document.getElementById("viz-data").textContent);
const META = DATA.meta || {{}};
const N = META.cycles || 0;
const P = DATA.platform || [];
const TRUTH = DATA.truth || [];
const AR = DATA.ar || [];
const EOS = DATA.eos || [];
const ESR = DATA.esr || [];
const FUSED = DATA.fused || [];
const WAYPOINTS = DATA.waypoints || [];
const WP_EVENTS = DATA.wp_events || [];
const ZONES = DATA.zones || [];
const MIN_SAMPLES_DEFAULT = {min_samples};
// 配色与常量（与 CSS 占位符同源）。
const COLOR_PLATFORM = "{COLOR_PLATFORM}";
const COLOR_AR = "{COLOR_AR}";
const COLOR_EOS = "{COLOR_EOS}";
const COLOR_ESR = "{COLOR_ESR}";
const COLOR_FUSED = "{COLOR_FUSED}";
const COLOR_CHURN = "{COLOR_CHURN}";
const COLOR_ZONE = "{COLOR_ZONE}";
const COLOR_GROUND = "#8c564b";  // 地面目标（entity_type=ground）
const TRUTH_PALETTE = ["#d62728", "#2ca02c", "#9467bd"];
const AIRCRAFT_PALETTE = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd",
                          "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf"];
const ESR_RAY_LENGTH_M = {esr_ray_len};
// 多机分组：aircraft_id 升序；各机颜色与航路/剖面一致。
const AC_IDS = [...new Set(P.map(q => q.ac))].sort((a, b) => a - b);
const acColor = ac => AIRCRAFT_PALETTE[(AC_IDS.indexOf(ac) + AIRCRAFT_PALETTE.length) % AIRCRAFT_PALETTE.length];
const P_BY_AC = {{}};
for (const q of P) (P_BY_AC[q.ac] = P_BY_AC[q.ac] || []).push(q);
const WP_BY_AC = {{}};
for (const q of WAYPOINTS) (WP_BY_AC[q.ac] = WP_BY_AC[q.ac] || []).push(q);
const LAYERS = [
  {{key:"zones", label:"巡逻区域", color:COLOR_ZONE, on:true}},
  {{key:"platform", label:"平台轨迹", color:"{COLOR_PLATFORM}", on:true}},
  {{key:"waypoints", label:"航路点", color:"#000", on:true}},
  {{key:"truth", label:"目标真值", color:"{COLOR_FUSED}", on:true}},
  {{key:"fused", label:"融合目标", color:"{COLOR_FUSED}", on:true}},
  {{key:"ar", label:"AR 航迹", color:"{COLOR_AR}", on:true}},
  {{key:"eos", label:"EOS 探测", color:"{COLOR_EOS}", on:true}},
  {{key:"esr", label:"ESR 假设", color:"{COLOR_ESR}", on:true}},
];
let cycle = 1, playing = false, timer = null;
const tooltip = document.getElementById("tooltip");

/* ═══════════════ 小工具 ═══════════════ */
function fmt(v, d) {{ return (v === null || v === undefined) ? "-" : v.toFixed(d); }}
function pt(x, y) {{ return x.toFixed(1) + "," + y.toFixed(1); }}
function showTip(html, ev) {{
  tooltip.innerHTML = html;
  tooltip.style.display = "block";
  const w = tooltip.offsetWidth, h = tooltip.offsetHeight;
  let x = ev.clientX + 14, y = ev.clientY + 14;
  if (x + w > window.innerWidth - 8) x = ev.clientX - w - 14;
  if (y + h > window.innerHeight - 8) y = ev.clientY - h - 14;
  tooltip.style.left = x + "px"; tooltip.style.top = y + "px";
}}
function hideTip() {{ tooltip.style.display = "none"; }}

/* ═══════════════ 通用 SVG 坐标轴 ═══════════════ */
// 把数据区间 [d0,d1] 映射到像素 [p0,p1]（可选反向）。
function scale(d0, d1, p0, p1) {{
  return function(v) {{ return p0 + (v - d0) * (p1 - p0) / (d1 - d0); }};
}}
function makeAxes(svg, W, H, x0, y0, x1, y1, xticks, yticks, xlabels, ylabels, yfmt) {{
  const ns = "http://www.w3.org/2000/svg";
  const g = document.createElementNS(ns, "g");
  for (const t of xticks) {{
    const l = document.createElementNS(ns, "line");
    l.setAttribute("x1", x0 + (x1 - x0) * (t - xticks[0]) / (xticks[xticks.length-1] - xticks[0]));
    l.setAttribute("y1", y0); l.setAttribute("x2", l.getAttribute("x1")); l.setAttribute("y2", y1);
    l.setAttribute("class", "grid"); g.appendChild(l);
  }}
  for (const t of yticks) {{
    const l = document.createElementNS(ns, "line");
    l.setAttribute("x1", x0); l.setAttribute("y1", y0 + (y1 - y0) * (t - yticks[0]) / (yticks[yticks.length-1] - yticks[0]));
    l.setAttribute("x2", x1); l.setAttribute("y2", l.getAttribute("y1"));
    l.setAttribute("class", "grid"); g.appendChild(l);
  }}
  const ax = document.createElementNS(ns, "line"); ax.setAttribute("x1", x0); ax.setAttribute("y1", y0);
  ax.setAttribute("x2", x1); ax.setAttribute("y2", y0); ax.setAttribute("class", "axis"); g.appendChild(ax);
  const ay = document.createElementNS(ns, "line"); ay.setAttribute("x1", x0); ay.setAttribute("y1", y0);
  ay.setAttribute("x2", x0); ay.setAttribute("y2", y1); ay.setAttribute("class", "axis"); g.appendChild(ay);
  const sx = scale(xticks[0], xticks[xticks.length-1], x0, x1);
  const sy = scale(yticks[0], yticks[yticks.length-1], y1, y0); // y 反向
  for (let i = 0; i < xticks.length; i++) {{
    const t = document.createElementNS(ns, "text");
    t.setAttribute("x", sx(xticks[i])); t.setAttribute("y", y0 + 14);
    t.setAttribute("text-anchor", "middle"); t.setAttribute("font-size", "10"); t.setAttribute("fill", "#666");
    t.textContent = xlabels ? xlabels[i] : xticks[i];
    g.appendChild(t);
  }}
  for (let i = 0; i < yticks.length; i++) {{
    const t = document.createElementNS(ns, "text");
    t.setAttribute("x", x0 - 6); t.setAttribute("y", sy(yticks[i]) + 3);
    t.setAttribute("text-anchor", "end"); t.setAttribute("font-size", "10"); t.setAttribute("fill", "#666");
    t.textContent = yfmt ? yfmt(yticks[i]) : yticks[i];
    g.appendChild(t);
  }}
  svg.appendChild(g);
  return {{sx, sy}};
}}
function lineEl(x1, y1, x2, y2, cls, attrs) {{
  const ns = "http://www.w3.org/2000/svg";
  const l = document.createElementNS(ns, "line");
  l.setAttribute("x1", x1); l.setAttribute("y1", y1); l.setAttribute("x2", x2); l.setAttribute("y2", y2);
  l.setAttribute("class", cls);
  for (const k in attrs) l.setAttribute(k, attrs[k]);
  return l;
}}
function polyEl(points, cls, attrs) {{
  const ns = "http://www.w3.org/2000/svg";
  const p = document.createElementNS(ns, "polyline");
  p.setAttribute("points", points.map(pt => pt.x.toFixed(1) + "," + pt.y.toFixed(1)).join(" "));
  p.setAttribute("class", cls);
  for (const k in attrs) p.setAttribute(k, attrs[k]);
  return p;
}}

/* ═══════════════ 俯视地图 ═══════════════ */
const MAP_W = 980, MAP_H = 560, MAP_M = 56;
let mapSvg = null, mapLayers = {{}};
function mapBounds() {{
  let xs = [], ys = [];
  for (const q of P) {{ xs.push(q.x); ys.push(q.y); }}
  for (const q of TRUTH) {{ xs.push(q.x); ys.push(q.y); }}
  for (const q of FUSED) {{ if (q.x !== null) {{ xs.push(q.x); ys.push(q.y); }} }}
  for (const q of WAYPOINTS) {{ xs.push(q.x); ys.push(q.y); }}
  for (const z of ZONES) for (const p of z.pts) {{ xs.push(p.x); ys.push(p.y); }}
  const pad = 600;
  let x0 = Math.min(...xs) - pad, x1 = Math.max(...xs) + pad;
  let y0 = Math.min(...ys) - pad, y1 = Math.max(...ys) + pad;
  const dx = x1 - x0, dy = y1 - y0;
  if (dx < 1) {{ x0 -= 500; x1 += 500; }}
  if (dy < 1) {{ y0 -= 500; y1 += 500; }}
  return {{x0, y0, x1, y1}};
}}
function renderMap() {{
  const ns = "http://www.w3.org/2000/svg";
  const wrap = document.getElementById("mapWrap");
  wrap.innerHTML = "";
  mapSvg = document.createElementNS(ns, "svg");
  mapSvg.setAttribute("viewBox", "0 0 " + MAP_W + " " + MAP_H);
  mapSvg.setAttribute("width", MAP_W); mapSvg.setAttribute("height", MAP_H);
  wrap.appendChild(mapSvg);

  // EOS 射线箭头标记。
  const defs = document.createElementNS(ns, "defs");
  const marker = document.createElementNS(ns, "marker");
  marker.setAttribute("id", "arrowEos");
  marker.setAttribute("viewBox", "0 0 10 10");
  marker.setAttribute("refX", "9"); marker.setAttribute("refY", "5");
  marker.setAttribute("markerWidth", "6"); marker.setAttribute("markerHeight", "6");
  marker.setAttribute("orient", "auto-start-reverse");
  const arrowPath = document.createElementNS(ns, "path");
  arrowPath.setAttribute("d", "M 0 0 L 10 5 L 0 10 z");
  arrowPath.setAttribute("fill", COLOR_EOS);
  marker.appendChild(arrowPath);
  defs.appendChild(marker);
  mapSvg.appendChild(defs);

  const b = mapBounds();
  // 统一缩放因子（锁定宽高比）：方位线/航向指示/到达半径圆的几何不变形。
  const k = Math.min((MAP_W - 2 * MAP_M) / (b.x1 - b.x0), (MAP_H - 2 * MAP_M) / (b.y1 - b.y0));
  const cx0 = (b.x0 + b.x1) / 2, cy0 = (b.y0 + b.y1) / 2;
  const sx = v => MAP_W / 2 + (v - cx0) * k;
  const sy = v => MAP_H / 2 - (v - cy0) * k; // y 反向（北在上）
  mapLayers = {{}};

  // 巡逻区域（底层：多边形闭合线 + 半透明填充 + 名称标注；圆形 → SVG circle）。
  for (const z of ZONES) {{
    const g = document.createElementNS(ns, "g");
    if (z.kind === "circle" && z.pts.length) {{
      const c = document.createElementNS(ns, "circle");
      c.setAttribute("cx", sx(z.pts[0].x)); c.setAttribute("cy", sy(z.pts[0].y));
      c.setAttribute("r", Math.max(2, z.radius * k));
      c.setAttribute("fill", COLOR_ZONE); c.setAttribute("fill-opacity", "0.06");
      c.setAttribute("stroke", COLOR_ZONE); c.setAttribute("stroke-width", "1.4");
      c.setAttribute("stroke-dasharray", "5 3");
      c.addEventListener("mousemove", ev => showTip(
        "巡逻区域 " + z.name + "（圆形）\\n半径 " + fmt(z.radius, 0) + " m", ev));
      c.addEventListener("mouseleave", hideTip);
      g.appendChild(c);
      const t = document.createElementNS(ns, "text");
      t.setAttribute("x", sx(z.pts[0].x) + 8); t.setAttribute("y", sy(z.pts[0].y) - 8);
      t.setAttribute("font-size", "11"); t.setAttribute("fill", COLOR_ZONE);
      t.textContent = z.name;
      g.appendChild(t);
    }} else if (z.pts.length >= 2) {{
      const closed = z.pts.concat([z.pts[0]]);
      const el = polyEl(closed.map(p => ({{x: sx(p.x), y: sy(p.y)}})), "zone", {{
        fill: COLOR_ZONE, "fill-opacity": 0.06, stroke: COLOR_ZONE,
        "stroke-width": 1.4, "stroke-dasharray": "5 3",
      }});
      el.addEventListener("mousemove", ev => showTip(
        "巡逻区域 " + z.name + "（多边形，顶点 " + z.pts.length + "）", ev));
      el.addEventListener("mouseleave", hideTip);
      g.appendChild(el);
      // 质心名称标注（顶点均值）。
      let cx = 0, cy = 0;
      for (const p of z.pts) {{ cx += sx(p.x); cy += sy(p.y); }}
      const t = document.createElementNS(ns, "text");
      t.setAttribute("x", cx / z.pts.length + 6); t.setAttribute("y", cy / z.pts.length - 6);
      t.setAttribute("font-size", "11"); t.setAttribute("fill", COLOR_ZONE);
      t.textContent = z.name;
      g.appendChild(t);
    }}
    mapSvg.appendChild(g);
    mapLayers["zones"] = mapLayers["zones"] || [];
    mapLayers["zones"].push(g);
  }}

  // 目标真值轨迹（按 id 分组着色；空中 = 虚线，地面 = 点线 + 方块终点标记）。
  const truthById = {{}};
  for (const q of TRUTH) {{ (truthById[q.id] = truthById[q.id] || []).push(q); }}
  Object.keys(truthById).forEach((id, idx) => {{
    const pts = truthById[id];
    const isGround = pts[pts.length - 1].et === "ground";
    const color = isGround ? COLOR_GROUND : TRUTH_PALETTE[idx % TRUTH_PALETTE.length];
    const el = polyEl(pts.map(q => ({{x: sx(q.x), y: sy(q.y)}})), "truth", {{
      fill: "none", stroke: color, "stroke-width": 1.5,
      "stroke-dasharray": isGround ? "2 4" : "6 4", opacity: 0.75,
    }});
    mapSvg.appendChild(el);
    mapLayers["truth"] = mapLayers["truth"] || [];
    mapLayers["truth"].push(el);
    // 终点标记 + 悬停（地面 = 方块，空中 = 圆点）。
    const last = pts[pts.length - 1];
    const kindLabel = isGround ? "地面目标" : "目标真值";
    const mk = document.createElementNS(ns, isGround ? "rect" : "circle");
    if (isGround) {{
      mk.setAttribute("x", sx(last.x) - 4); mk.setAttribute("y", sy(last.y) - 4);
      mk.setAttribute("width", 8); mk.setAttribute("height", 8);
    }} else {{
      mk.setAttribute("cx", sx(last.x)); mk.setAttribute("cy", sy(last.y)); mk.setAttribute("r", 4);
    }}
    mk.setAttribute("fill", color);
    mk.addEventListener("mousemove", ev => showTip(
      kindLabel + " T" + id + (isGround ? "（地面）" : "") + "\\n终点 cycle " + last.c +
      "，alt " + fmt(last.alt, 1) + " m，rcs " + fmt(last.rcs, 2) + " m²", ev));
    mk.addEventListener("mouseleave", hideTip);
    mapSvg.appendChild(mk);
  }});

  // 融合目标位置（全周期点，低透明度）。
  for (const q of FUSED) {{
    if (q.x === null) continue;
    const c = document.createElementNS(ns, "circle");
    c.setAttribute("cx", sx(q.x)); c.setAttribute("cy", sy(q.y)); c.setAttribute("r", 2.2);
    c.setAttribute("fill", COLOR_FUSED); c.setAttribute("opacity", 0.28);
    c.addEventListener("mousemove", ev => showTip(
      "融合 key " + q.key + " @ cycle " + q.c + "\\nconf " + fmt(q.conf, 3) +
      "，AR/ESR/EOS 采样 " + q.a + "/" + q.e + "/" + q.o, ev));
    c.addEventListener("mouseleave", hideTip);
    mapSvg.appendChild(c);
    mapLayers["fused"] = mapLayers["fused"] || [];
    mapLayers["fused"].push(c);
  }}

  // 航路点（编号圆圈 + 到达半径；按机分组着色，编号 = 机内航点索引）。
  AC_IDS.forEach(ac => {{
    const wps = WP_BY_AC[ac] || [];
    const acCol = acColor(ac);
    wps.forEach((wp, idx) => {{
      const g = document.createElementNS(ns, "g");
      const r = document.createElementNS(ns, "circle");
      r.setAttribute("cx", sx(wp.x)); r.setAttribute("cy", sy(wp.y)); r.setAttribute("r", 10);
      r.setAttribute("fill", "none"); r.setAttribute("stroke", acCol); r.setAttribute("stroke-dasharray", "2 2");
      r.setAttribute("opacity", 0.5);
      g.appendChild(r);
      const c = document.createElementNS(ns, "circle");
      c.setAttribute("cx", sx(wp.x)); c.setAttribute("cy", sy(wp.y)); c.setAttribute("r", 5);
      c.setAttribute("fill", acCol);
      c.addEventListener("mousemove", ev => showTip(
        "飞行器 " + ac + " 航路点 " + idx + "\\nalt " + fmt(wp.alt, 1) + " m，speed " + fmt(wp.spd, 1) +
        " m/s，半径 " + fmt(wp.r, 1) + " m", ev));
      c.addEventListener("mouseleave", hideTip);
      g.appendChild(c);
      const t = document.createElementNS(ns, "text");
      t.setAttribute("x", sx(wp.x) + 8); t.setAttribute("y", sy(wp.y) - 6);
      t.setAttribute("font-size", "11"); t.setAttribute("fill", acCol);
      t.textContent = String(idx);
      g.appendChild(t);
      mapSvg.appendChild(g);
      mapLayers["waypoints"] = mapLayers["waypoints"] || [];
      mapLayers["waypoints"].push(g);
    }});
  }});

  // 平台轨迹（按机分组多色全轨迹；动态段在 updateMap 中重画）。
  for (const ac of AC_IDS) {{
    const qs = P_BY_AC[ac] || [];
    if (!qs.length) continue;
    const plat = polyEl(qs.map(q => ({{x: sx(q.x), y: sy(q.y)}})), "platform", {{
      fill: "none", stroke: acColor(ac), "stroke-width": 2,
    }});
    mapSvg.appendChild(plat);
    mapLayers["platform"] = mapLayers["platform"] || [];
    mapLayers["platform"].push(plat);
  }}

  // 动态层（每周期更新）：平台当前位置、EOS/ESR 射线、AR 航迹点。
  mapLayers["dynamic"] = document.createElementNS(ns, "g");
  mapSvg.appendChild(mapLayers["dynamic"]);

  const ax = makeAxes(mapSvg, MAP_W, MAP_H, MAP_M, MAP_H - MAP_M, MAP_W - MAP_M, MAP_M,
    [], [], []);
  mapLayers["sx"] = sx; mapLayers["sy"] = sy; mapLayers["k"] = k;
  applyLayerVisibility();
  updateMap();
}}

// 静态图层显隐（平台全轨迹/航路点/真值/融合点）。
function applyLayerVisibility() {{
  for (const key of Object.keys(mapLayers)) {{
    if (!Array.isArray(mapLayers[key])) continue; // 非元素集合（缩放因子等）跳过
    const def = LAYERS.find(l => l.key === key);
    const on = def ? def.on : true;
    for (const el of mapLayers[key]) el.style.display = on ? "" : "none";
  }}
}}

// 当前周期动态元素（平台标记 + 射线 + AR 航迹点 + 航点事件标注）。
function updateMap() {{
  const ns = "http://www.w3.org/2000/svg";
  const g = mapLayers["dynamic"];
  if (!g) return;
  g.innerHTML = "";
  const sx = mapLayers["sx"], sy = mapLayers["sy"], k = mapLayers["k"];
  const layersOn = LAYERS.filter(l => l.on).map(l => l.key);

  // 各机位置（当前周期索引 = cycle-1；传感器射线以主平台 = aircraft_id 最小者为本）。
  const platByAc = {{}};
  for (const ac of AC_IDS) {{
    const qs = P_BY_AC[ac] || [];
    if (qs.length) platByAc[ac] = qs[Math.min(cycle - 1, qs.length - 1)];
  }}
  if (!AC_IDS.length) return;
  const mainAc = AC_IDS[0];
  const platPt = {{}};
  for (const ac of AC_IDS) {{
    const plat = platByAc[ac];
    if (!plat) continue;
    platPt[ac] = {{x: sx(plat.x), y: sy(plat.y)}};
  }}

  if (layersOn.includes("platform")) {{
    for (const ac of AC_IDS) {{
      const plat = platByAc[ac];
      if (!plat) continue;
      const pt = platPt[ac];
      const col = acColor(ac);
      // 轨迹截至当前周期。
      const qs = P_BY_AC[ac];
      const trail = qs.slice(0, Math.min(cycle, qs.length)).map(q => ({{x: sx(q.x), y: sy(q.y)}}));
      const tl = polyEl(trail, "platform-trail", {{fill: "none", stroke: col, "stroke-width": 3}});
      g.appendChild(tl);
      const m = document.createElementNS(ns, "circle");
      m.setAttribute("cx", pt.x); m.setAttribute("cy", pt.y); m.setAttribute("r", 6);
      m.setAttribute("fill", col); m.setAttribute("stroke", "#fff"); m.setAttribute("stroke-width", 2);
      m.addEventListener("mousemove", ev => showTip(
        "飞行器 " + ac + " @ cycle " + cycle + "\\nalt " + fmt(plat.alt, 1) + " m，航向 " + fmt(plat.hdg, 1) +
        "°，速度 " + fmt(plat.spd, 1) + " m/s\\n航点进度 " + plat.wp + "/" + plat.wpc, ev));
      m.addEventListener("mouseleave", hideTip);
      g.appendChild(m);
      // 航向指示线
      const hdg = plat.hdg * Math.PI / 180;
      const hl = lineEl(pt.x, pt.y, pt.x + 900 * Math.sin(hdg), pt.y - 900 * Math.cos(hdg),
        "", {{stroke: col, "stroke-width": 1.5, opacity: 0.6}});
      g.appendChild(hl);
    }}
  }}

  // EOS/ESR/AR 射线：单平台视角（主平台，ac = AC_IDS[0]）。
  const mainPt = platPt[mainAc];

  // EOS 探测射线（az 0 = 东：east = cos, north = sin；平台局部系）。
  if (layersOn.includes("eos")) {{
    for (const d of EOS) {{
      if (d.c !== cycle || !d.detected || d.range === null) continue;
      const az = d.az * Math.PI / 180;
      const ex = mainPt.x + d.range * Math.cos(az) * k;
      const ey = mainPt.y - d.range * Math.sin(az) * k;
      const l = lineEl(mainPt.x, mainPt.y, ex, ey, "", {{
        stroke: COLOR_EOS, "stroke-width": 2, opacity: 0.85,
        "marker-end": "url(#arrowEos)",
      }});
      l.addEventListener("mousemove", ev => showTip(
        "EOS 探测 det " + d.det + "\\n目标 T" + (d.tid ?? "-") + "，range " + fmt(d.range, 1) +
        " m，az " + fmt(d.az, 2) + "° el " + fmt(d.el, 2) + "°\\nSNR " + fmt(d.snr, 1) + " dB", ev));
      l.addEventListener("mouseleave", hideTip);
      g.appendChild(l);
    }}
  }}

  // ESR 方位线（固定长度）。
  if (layersOn.includes("esr")) {{
    for (const h of ESR) {{
      if (h.c !== cycle) continue;
      const az = h.az * Math.PI / 180;
      const ex = mainPt.x + ESR_RAY_LENGTH_M * Math.cos(az) * k;
      const ey = mainPt.y - ESR_RAY_LENGTH_M * Math.sin(az) * k;
      const l = lineEl(mainPt.x, mainPt.y, ex, ey, "", {{
        stroke: COLOR_ESR, "stroke-width": 1.5, opacity: 0.7, "stroke-dasharray": "3 3",
      }});
      l.addEventListener("mousemove", ev => showTip(
        "ESR 假设 hyp " + h.hyp + "\\nbearing " + fmt(h.az, 2) + "°，conf " + fmt(h.conf, 3) +
        "，模式 " + h.mode + "，威胁 " + h.threat, ev));
      l.addEventListener("mouseleave", hideTip);
      g.appendChild(l);
    }}
  }}

  // AR 航迹点（雷达局部 ENU 叠加到平台世界位姿）。
  // 注：demo 以零姿态构造 AR 位姿（systems.cpp MakePlatformPose），雷达局部系即平台
  // ENU 切平面（x=东/y=北），故直接平移即可；非零姿态场景需按航向旋转再叠加。
  if (layersOn.includes("ar")) {{
    for (const t of AR) {{
      if (t.c !== cycle || t.st !== "confirmed") continue;
      const tx = mainPt.x + t.x * k;
      const ty = mainPt.y - t.y * k;
      const r = document.createElementNS(ns, "rect");
      r.setAttribute("x", tx - 5); r.setAttribute("y", ty - 5); r.setAttribute("width", 10); r.setAttribute("height", 10);
      r.setAttribute("fill", COLOR_AR); r.setAttribute("stroke", "#fff"); r.setAttribute("stroke-width", 1);
      r.addEventListener("mousemove", ev => showTip(
        "AR 航迹 key " + t.key + "（目标 T" + t.tid + "）\\nstatus " + t.st +
        "，speed " + fmt(t.spd, 1) + " m/s，rcs " + fmt(t.rcs, 2) + " m²", ev));
      r.addEventListener("mouseleave", hideTip);
      g.appendChild(r);
    }}
  }}

  // 航点完成事件标注（本周期内完成的航点）。
  for (const e of WP_EVENTS) {{
    if (e.c === cycle) {{
      const wp = WAYPOINTS[e.i];
      if (!wp) continue;
      const mark = document.createElementNS(ns, "circle");
      mark.setAttribute("cx", sx(wp.x)); mark.setAttribute("cy", sy(wp.y)); mark.setAttribute("r", 9);
      mark.setAttribute("fill", "none"); mark.setAttribute("stroke", "#f0f");
      mark.setAttribute("stroke-width", 2.5);
      mark.addEventListener("mousemove", ev => showTip(
        "航点 " + e.i + " 完成 @ t=" + fmt(e.t, 1) + " s（" + e.gate + "）\\n距离 " + fmt(e.d, 1) +
        " m，侧距 " + fmt(e.ct, 1) + " m，沿航迹 " + fmt(e.at, 1) + " m，阈值 " + fmt(e.thr, 1) + " m", ev));
      mark.addEventListener("mouseleave", hideTip);
      g.appendChild(mark);
    }}
  }}
}}

/* ═══════════════ 传感器日志时间线 ═══════════════ */
const TL_H = 26; // 每行高
function buildTimelineRows(minSamples) {{
  // 真值目标行：AR 航迹（confirmed）+ EOS 探测（detected=1）。
  const truthIds = [...new Set(TRUTH.map(q => q.id))].sort((a, b) => a - b);
  const rows = truthIds.map(id => ({{
    kind: "truth", id: id, label: "T" + id,
    segs: {{ar: [], eos: []}}, found: [], lost: [], active: [],
  }}));
  const rowById = {{}};
  rows.forEach(r => rowById[r.id] = r);
  for (const t of AR) {{
    if (t.st !== "confirmed") continue;
    const r = rowById[t.tid];
    if (r) {{ r.segs.ar.push(t.c); r.active.push(t.c); }}
  }}
  for (const d of EOS) {{
    if (!d.detected || d.tid === null) continue;
    const r = rowById[d.tid];
    if (r) {{ r.segs.eos.push(d.c); }}
  }}

  // 融合键行：活跃周期 + 各通道采样 >0 的周期；found/lost 从活跃序列推导。
  const keyCycles = {{}};
  const keyMeta = {{}};
  for (const q of FUSED) {{
    (keyCycles[q.key] = keyCycles[q.key] || []).push(q.c);
    keyMeta[q.key] = keyMeta[q.key] || {{a: 0, e: 0, o: 0, first: q.c, last: 0}};
    keyMeta[q.key].a += q.a || 0; keyMeta[q.key].e += q.e || 0; keyMeta[q.key].o += q.o || 0;
    keyMeta[q.key].last = Math.max(keyMeta[q.key].last, q.c);
  }}
  const keys = Object.keys(keyCycles)
    .map(Number)
    .filter(k => keyCycles[k].length >= minSamples)
    .sort((a, b) => keyMeta[a].first - keyMeta[b].first || keyCycles[b].length - keyCycles[a].length);
  for (const k of keys) {{
    const cs = new Set(keyCycles[k]);
    const row = {{
      kind: "fused", id: k, label: "key " + k,
      segs: {{ar: [], eos: [], esr: [], fused: []}}, found: [], lost: [], active: [...cs],
    }};
    for (const q of FUSED) {{
      if (q.key !== k) continue;
      row.segs.fused.push(q.c);
      if (q.a > 0) row.segs.ar.push(q.c);
      if (q.e > 0) row.segs.esr.push(q.c);
      if (q.o > 0) row.segs.eos.push(q.c);
    }}
    const sorted = keyCycles[k].slice().sort((a, b) => a - b);
    for (let i = 0; i < sorted.length; i++) {{
      if (i === 0 || sorted[i] - sorted[i - 1] > 1) row.found.push(sorted[i]);
      if (i === sorted.length - 1 || sorted[i + 1] - sorted[i] > 1) row.lost.push(sorted[i]);
    }}
    rows.push(row);
  }}

  // churn 行：采样数 < minSamples 的短命融合键，逐周期活跃计数。
  const churnByCycle = new Array(N + 1).fill(0);
  for (const k of Object.keys(keyCycles).map(Number)) {{
    if (keyCycles[k].length >= minSamples) continue;
    for (const c of keyCycles[k]) churnByCycle[c]++;
  }}
  rows.push({{kind: "churn", id: -1, label: "churn(<" + minSamples + ")", segs: {{}}, found: [], lost: [],
             active: [], churn: churnByCycle}});
  return rows;
}}
function renderTimeline() {{
  const minSamples = Math.max(1, parseInt(document.getElementById("tlFilter").value) || MIN_SAMPLES_DEFAULT);
  const rows = buildTimelineRows(minSamples);
  const ns = "http://www.w3.org/2000/svg";
  const wrap = document.getElementById("timelineWrap");
  wrap.innerHTML = "";
  const W = 980, M = 58, x0 = M, x1 = W - 8;
  const H = rows.length * TL_H + 30;
  const svg = document.createElementNS(ns, "svg");
  svg.setAttribute("viewBox", "0 0 " + W + " " + H);
  svg.setAttribute("width", W); svg.setAttribute("height", H);
  wrap.appendChild(svg);

  const sx = scale(1, N, x0, x1);
  const segColors = {{ar: COLOR_AR, eos: COLOR_EOS, esr: COLOR_ESR, fused: COLOR_FUSED}};
  const segOrder = ["ar", "esr", "eos", "fused"];
  const segNames = {{ar: "AR", eos: "EOS", esr: "ESR", fused: "融合"}};

  rows.forEach((row, ri) => {{
    const yTop = 16 + ri * TL_H;
    // 行标签
    const lab = document.createElementNS(ns, "text");
    lab.setAttribute("x", M - 8); lab.setAttribute("y", yTop + TL_H / 2 + 3);
    lab.setAttribute("text-anchor", "end"); lab.setAttribute("font-size", "11");
    lab.setAttribute("fill", row.kind === "churn" ? "#999" : "#333");
    lab.textContent = row.label;
    svg.appendChild(lab);
    // 轨道背景
    const bg = document.createElementNS(ns, "rect");
    bg.setAttribute("x", x0); bg.setAttribute("y", yTop + 1); bg.setAttribute("width", x1 - x0);
    bg.setAttribute("height", TL_H - 2); bg.setAttribute("fill", "#f7f7f7");
    svg.appendChild(bg);
    // 各通道色段
    if (row.kind === "churn") {{
      for (let c = 1; c <= N; c++) {{
        if (row.churn[c]) {{
          const r = document.createElementNS(ns, "rect");
          r.setAttribute("x", sx(c)); r.setAttribute("y", yTop + 6);
          r.setAttribute("width", Math.max(2, sx(c + 1) - sx(c) - 1));
          r.setAttribute("height", TL_H - 12); r.setAttribute("fill", COLOR_CHURN);
          r.setAttribute("opacity", Math.min(1, 0.25 + row.churn[c] * 0.1));
          r.addEventListener("mousemove", ev => showTip(
            "cycle " + c + "：短命融合键 " + row.churn[c] + " 个（采样 < " + minSamples + "）", ev));
          r.addEventListener("mouseleave", hideTip);
          svg.appendChild(r);
        }}
      }}
    }} else {{
      for (const ch of segOrder) {{
        const cs = row.segs[ch];
        if (!cs || !cs.length) continue;
        const sorted = cs.slice().sort((a, b) => a - b);
        let segStart = sorted[0], prev = sorted[0];
        const segs = [];
        for (let i = 1; i <= sorted.length; i++) {{
          const c = sorted[i];
          if (c !== prev + 1) {{ segs.push([segStart, prev]); segStart = c; }}
          prev = c;
        }}
        for (const [s0, s1] of segs) {{
          const r = document.createElementNS(ns, "rect");
          r.setAttribute("x", sx(s0) + 0.5); r.setAttribute("y", yTop + 5);
          r.setAttribute("width", Math.max(2, sx(s1 + 1) - sx(s0) - 1));
          r.setAttribute("height", TL_H - 10);
          r.setAttribute("fill", segColors[ch]); r.setAttribute("opacity", 0.85);
          r.addEventListener("mousemove", ev => showTip(
            row.label + " · " + segNames[ch] + " 活跃 cycle " + s0 + "–" + s1, ev));
          r.addEventListener("mouseleave", hideTip);
          svg.appendChild(r);
        }}
      }}
      // found/lost 标记
      for (const c of row.found) {{
        const t = document.createElementNS(ns, "polygon");
        const x = sx(c);
        t.setAttribute("points", pt(x, yTop + 2) + " " + pt(x + 5, yTop + 11) + " " + pt(x - 5, yTop + 11));
        t.setAttribute("fill", COLOR_FUSED);
        svg.appendChild(t);
      }}
      for (const c of row.lost) {{
        const t = document.createElementNS(ns, "polygon");
        const x = sx(c);
        t.setAttribute("points", pt(x, yTop + 13) + " " + pt(x + 5, yTop + 4) + " " + pt(x - 5, yTop + 4));
        t.setAttribute("fill", "#555");
        svg.appendChild(t);
      }}
    }}
  }});

  // 时间轴刻度：垂直网格 + 底部标签。
  const ticks = [1];
  for (let t = 10; t <= N; t += 10) ticks.push(t);
  const yTopAx = 10, yBotAx = 16 + rows.length * TL_H;
  for (const t of ticks) {{
    const gl = lineEl(sx(t), yTopAx, sx(t), yBotAx, "grid", {{}});
    svg.appendChild(gl);
    const txt = document.createElementNS(ns, "text");
    txt.setAttribute("x", sx(t)); txt.setAttribute("y", yBotAx + 14);
    txt.setAttribute("text-anchor", "middle"); txt.setAttribute("font-size", "10");
    txt.setAttribute("fill", "#666");
    txt.textContent = String(t);
    svg.appendChild(txt);
  }}

  document.getElementById("tlFilterHint").textContent =
    "显示 " + rows.length + " 行（真值目标 + 融合键 ≥" + minSamples + " 采样 + churn 聚合行）";
  return {{svg, sx, rowCount: rows.length}};
}}
let tlState = null;

/* ═══════════════ 融合置信度 + 通道采样 ═══════════════ */
function renderConf() {{
  const minSamples = Math.max(1, parseInt(document.getElementById("tlFilter").value) || MIN_SAMPLES_DEFAULT);
  const ns = "http://www.w3.org/2000/svg";
  const wrap = document.getElementById("confWrap");
  wrap.innerHTML = "";
  const W = 980, M = {x: 58, y: 30}, H = 460;
  const svg = document.createElementNS(ns, "svg");
  svg.setAttribute("viewBox", "0 0 " + W + " " + H);
  svg.setAttribute("width", W); svg.setAttribute("height", H);
  wrap.appendChild(svg);

  // 稳定键（≥ minSamples）：置信度折线。
  const keyCycles = {{}};
  for (const q of FUSED) (keyCycles[q.key] = keyCycles[q.key] || []).push(q.c);
  const keys = Object.keys(keyCycles).map(Number).filter(k => keyCycles[k].length >= minSamples)
    .sort((a, b) => keyCycles[b].length - keyCycles[a].length);
  const confByKey = {{}};
  for (const k of keys) confByKey[k] = [];
  for (const q of FUSED) if (confByKey[q.key]) confByKey[q.key].push(q);

  const y0 = M.y, y1 = H - M.y - 30, x0 = M.x, x1 = W - M.x;
  const sx = scale(1, N, x0, x1);
  const sy = scale(0, 3, y1, y0); // conf 范围 0..3（滑窗 Σ 判决值）
  const ticks = [1]; for (let t = 10; t <= N; t += 10) ticks.push(t);
  const ax = makeAxes(svg, W, H, x0, y0, x1, y1, ticks, [0, 1, 2, 3], ticks, [0, 1, 2, 3], null);
  const ylab = document.createElementNS(ns, "text");
  ylab.setAttribute("x", 10); ylab.setAttribute("y", (y0 + y1) / 2);
  ylab.setAttribute("transform", "rotate(-90 10 " + (y0 + y1) / 2 + ")");
  ylab.setAttribute("font-size", "11"); ylab.setAttribute("fill", "#666");
  ylab.textContent = "融合置信度";
  svg.appendChild(ylab);

  const palette = ["#d62728", "#2ca02c", "#1f77b4", "#9467bd", "#ff7f0e", "#17becf", "#8c564b"];
  keys.forEach((k, idx) => {{
    const pts = confByKey[k].map(q => ({{x: sx(q.c), y: sy(Math.min(3, q.conf))}}));
    const el = polyEl(pts, "", {{fill: "none", stroke: palette[idx % palette.length], "stroke-width": 1.8}});
    el.addEventListener("mousemove", ev => showTip(
      "融合 key " + k + "（" + confByKey[k].length + " 周期）\\nconf " + fmt(confByKey[k][confByKey[k].length-1].conf, 3), ev));
    el.addEventListener("mouseleave", hideTip);
    svg.appendChild(el);
    const lab = document.createElementNS(ns, "text");
    lab.setAttribute("x", sx(confByKey[k][confByKey[k].length-1].c) + 3);
    lab.setAttribute("y", sy(Math.min(3, confByKey[k][confByKey[k].length-1].conf)) - 3);
    lab.setAttribute("font-size", "10"); lab.setAttribute("fill", palette[idx % palette.length]);
    lab.textContent = "key " + k;
    svg.appendChild(lab);
  }});

  // 通道采样量（全融合键求和，堆叠面积）。
  const aTot = new Array(N + 1).fill(0), eTot = new Array(N + 1).fill(0), oTot = new Array(N + 1).fill(0);
  for (const q of FUSED) {{
    aTot[q.c] += q.a || 0; eTot[q.c] += q.e || 0; oTot[q.c] += q.o || 0;
  }}
  const y2_0 = H - 170, y2_1 = H - M.y;
  const maxV = Math.max(3, ...aTot, ...eTot, ...oTot, 1);
  const sy2b = scale(0, maxV, y2_1, y2_0);
  const hLine = lineEl(x0, y2_0, x1, y2_0, "", {{stroke: "#ccc"}});
  svg.appendChild(hLine);
  const lay = document.createElementNS(ns, "text");
  lay.setAttribute("x", x0); lay.setAttribute("y", y2_0 - 8);
  lay.setAttribute("font-size", "11"); lay.setAttribute("fill", "#666");
  lay.textContent = "各通道采样量（全融合键求和）";
  svg.appendChild(lay);
  const series = [
    {{name: "AR", color: COLOR_AR, v: aTot}},
    {{name: "ESR", color: COLOR_ESR, v: eTot}},
    {{name: "EOS", color: COLOR_EOS, v: oTot}},
  ];
  let cumulative = new Array(N + 1).fill(0);
  for (const s of series) {{
    const pts = [];
    for (let c = 1; c <= N; c++) {{
      pts.push({{x: sx(c), y: sy2b(cumulative[c])}});
      cumulative[c] += s.v[c];
    }}
    for (let c = N; c >= 1; c--) pts.push({{x: sx(c), y: sy2b(cumulative[c])}});
    const poly = document.createElementNS(ns, "polygon");
    poly.setAttribute("points", pts.map(p => p.x.toFixed(1) + "," + p.y.toFixed(1)).join(" "));
    poly.setAttribute("fill", s.color); poly.setAttribute("opacity", 0.55);
    svg.appendChild(poly);
  }}
  // 通道采样图例
  const leg = document.createElementNS(ns, "text");
  leg.setAttribute("x", x0); leg.setAttribute("y", H - 6);
  leg.setAttribute("font-size", "11"); leg.setAttribute("fill", "#666");
  leg.textContent = "图例：";
  svg.appendChild(leg);
  let lx = x0 + 40;
  for (const s of series) {{
    const sw = document.createElementNS(ns, "rect");
    sw.setAttribute("x", lx); sw.setAttribute("y", H - 15); sw.setAttribute("width", 11); sw.setAttribute("height", 11);
    sw.setAttribute("fill", s.color); sw.setAttribute("opacity", 0.6);
    svg.appendChild(sw);
    const t = document.createElementNS(ns, "text");
    t.setAttribute("x", lx + 15); t.setAttribute("y", H - 6);
    t.setAttribute("font-size", "11"); t.setAttribute("fill", "#333");
    t.textContent = s.name;
    svg.appendChild(t);
    lx += 55;
  }}
  return {{svg, sx, yTop: M.y, yBot: H - M.y}};
}}
let confState = null;

/* ═══════════════ 飞行剖面 ═══════════════ */
function renderProfile() {{
  const ns = "http://www.w3.org/2000/svg";
  const wrap = document.getElementById("profileWrap");
  wrap.innerHTML = "";
  const W = 980, M = {x: 58, y: 24}, H = 230;
  const svg = document.createElementNS(ns, "svg");
  svg.setAttribute("viewBox", "0 0 " + W + " " + H);
  svg.setAttribute("width", W); svg.setAttribute("height", H);
  wrap.appendChild(svg);
  const x0 = M.x, x1 = W - M.x;
  const sx = scale(1, N, x0, x1);
  const ticks = [1]; for (let t = 10; t <= N; t += 10) ticks.push(t);

  const series = [
    {{name: "高度 (m)", color: "#8c564b", get: q => q.alt, y0: 0, y1: 0, unit: " m"}},
    {{name: "速度 (m/s)", color: "#1f77b4", get: q => q.spd, y0: 0, y1: 0, unit: " m/s"}},
    {{name: "航向 (°)", color: "#2ca02c", get: q => q.hdg, y0: 0, y1: 0, unit: "°"}},
  ];
  // 每行一个子图：高度 / 速度 / 航向（多机分线，色与地图一致）。
  series.forEach((s, idx) => {{
    const yTop = idx * 68, yBot = yTop + 52;
    const vals = P.map(q => s.get(q)).filter(v => v !== null && v !== undefined);
    if (!vals.length) return;  // 该指标无数据（如 FD dump 无速度列）：跳过子图
    const lo = Math.min(...vals), hi = Math.max(...vals);
    const pad = Math.max((hi - lo) * 0.12, 0.5);
    s.y0 = lo - pad; s.y1 = hi + pad;
    const sy = scale(s.y0, s.y1, yBot, yTop);
    const ax = makeAxes(svg, W, H, x0, yBot, x1, yTop, ticks, [], ticks, [], null);
    AC_IDS.forEach(ac => {{
      const qs = P_BY_AC[ac] || [];
      if (!qs.length) return;
      const pts = qs.map(q => ({{x: sx(q.c), y: sy(s.get(q))}}));
      const el = polyEl(pts, "", {{fill: "none", stroke: acColor(ac), "stroke-width": 1.8}});
      el.addEventListener("mousemove", ev => showTip(
        "飞行器 " + ac + " · " + s.name.replace(" (°)", "") + "：终点 " +
        fmt(s.get(qs[qs.length - 1]), 1) + s.unit, ev));
      el.addEventListener("mouseleave", hideTip);
      svg.appendChild(el);
    }});
    const lab = document.createElementNS(ns, "text");
    lab.setAttribute("x", x0); lab.setAttribute("y", yTop + 11);
    lab.setAttribute("font-size", "11"); lab.setAttribute("fill", "#666");
    lab.textContent = s.name + "（" + fmt(lo, 1) + "–" + fmt(hi, 1) + s.unit + "）";
    svg.appendChild(lab);
    // 多机图例（机号色标）。
    if (idx === 0 && AC_IDS.length > 1) {{
      let lx = x0 + 160;
      for (const ac of AC_IDS) {{
        const sw = document.createElementNS(ns, "rect");
        sw.setAttribute("x", lx); sw.setAttribute("y", yTop + 3); sw.setAttribute("width", 10); sw.setAttribute("height", 10);
        sw.setAttribute("fill", acColor(ac));
        svg.appendChild(sw);
        const t = document.createElementNS(ns, "text");
        t.setAttribute("x", lx + 14); t.setAttribute("y", yTop + 12);
        t.setAttribute("font-size", "10"); t.setAttribute("fill", "#333");
        t.textContent = "机 " + ac;
        svg.appendChild(t);
        lx += 52;
      }}
    }}
  }});
  return {{svg, sx}};
}}
let profState = null;

/* ═══════════════ 时间游标（跨面板联动） ═══════════════ */
let cursorEls = [];
function updateCursor() {{
  // 移除旧游标
  for (const c of cursorEls) c.remove();
  cursorEls = [];
  const addTo = (svg, x, yTop, yBot) => {{
    const ns = "http://www.w3.org/2000/svg";
    const l = lineEl(x, yTop, x, yBot, "cursor", {{}});
    svg.appendChild(l);
    cursorEls.push(l);
  }};
  if (tlState) addTo(tlState.svg, tlState.sx(cycle), 10, 16 + tlState.rowCount * 26);
  if (confState) addTo(confState.svg, confState.sx(cycle), confState.yTop, confState.yBot);
  if (profState) addTo(profState.svg, profState.sx(cycle), 0, 210);
}}

/* ═══════════════ 图层开关 ═══════════════ */
function renderLayerToggles() {{
  const box = document.getElementById("layerToggles");
  box.innerHTML = "";
  for (const l of LAYERS) {{
    const lab = document.createElement("label");
    lab.innerHTML = '<span class="swatch" style="background:' + l.color + '"></span>' + l.label;
    const cb = document.createElement("input");
    cb.type = "checkbox"; cb.checked = l.on;
    cb.addEventListener("change", () => {{
      l.on = cb.checked;
      applyLayerVisibility();
      updateMap(); updateCursor();
    }});
    lab.prepend(cb);
    box.appendChild(lab);
  }}
}}

/* ═══════════════ 播放控制 ═══════════════ */
function setCycle(c) {{
  cycle = Math.max(1, Math.min(N, c));
  document.getElementById("cycleSlider").value = cycle;
  document.getElementById("cycleReadout").textContent = "cycle " + cycle + " / " + N;
  updateMap(); updateCursor();
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
  timer = setInterval(step, Math.max(20, Math.round(1000 / speed)));
}}
function pause() {{
  playing = false;
  if (timer) clearInterval(timer);
  timer = null;
  document.getElementById("playBtn").textContent = "▶ 播放";
  document.getElementById("playBtn").classList.remove("active");
}}

/* ═══════════════ 初始化 ═══════════════ */
function init() {{
  if (!N) {{
    document.body.insertAdjacentHTML("afterbegin", "<p style='color:#c00'>数据为空（CSV 缺失或行数不足）</p>");
    return;
  }}
  renderLayerToggles();
  renderMap();
  tlState = renderTimeline();
  confState = renderConf();
  profState = renderProfile();
  document.getElementById("cycleSlider").addEventListener("input", e => {{
    pause(); setCycle(parseInt(e.target.value));
  }});
  document.getElementById("playBtn").addEventListener("click", () => playing ? pause() : play());
  document.getElementById("tlFilter").addEventListener("change", () => {{
    tlState = renderTimeline(); confState = renderConf();
    updateCursor();
  }});
  document.getElementById("speedSel").addEventListener("change", () => {{ if (playing) {{ pause(); play(); }} }});
  setCycle(1);
}}
init();
</script>
</body>
</html>
"""


# ═══════════════════════════════════════════════════════════════════
# 入口
# ═══════════════════════════════════════════════════════════════════

EXPECTED_HEADERS = {
    "platform_track.csv": "cycle,t_sec,aircraft_id,lat_deg,lon_deg,alt_m,heading_deg,"
                          "speed_mps,wp_index,wp_count,model",
    "target_truth.csv": "cycle,t_sec,target_id,entity_type,lat_deg,lon_deg,alt_m,rcs",
    "ar_tracks.csv": "cycle,t_sec,key,target_id,status,pos_x_m,pos_y_m,pos_z_m,"
                     "speed_mps,rcs,hit_count,miss_count",
    "eos_detections.csv": "cycle,t_sec,det_id,target_id,range_m,az_deg,el_deg,snr_db,detected",
    "esr_hypotheses.csv": "cycle,t_sec,hyp_id,bearing_az_deg,bearing_el_deg,"
                          "confidence,mode,threat_level,last_seen_cycle",
    "fused_tracks.csv": "cycle,t_sec,key,confidence,last_update_cycle,"
                        "ar_samples,esr_samples,eos_samples,lat_deg,lon_deg,alt_m,"
                        "bearing_az_deg",
    "route_plan.csv": "aircraft_id,index,lat_deg,lon_deg,alt_m,speed_mps,radius_m",
    "waypoint_events.csv": "t_sec,waypoint_index,intermediate,gate,distance_m,"
                           "cross_track_m,along_track_m,threshold_m",
    "zones.csv": "name,kind,lat_deg,lon_deg,alt_m,radius_m",
}

# 可选文件：缺省时查看器自动跳过对应图层。传感器/融合/航点事件为
# component_entt 独有；zones 仅巡逻场景产出；component_attachment 只落盘
# platform_track/target_truth/route_plan（+ 可选 zones）。
OPTIONAL_FILES = {
    "ar_tracks.csv",
    "eos_detections.csv",
    "esr_hypotheses.csv",
    "fused_tracks.csv",
    "waypoint_events.csv",
    "zones.csv",
}


def validate_data(data_dir):
    """回归校验（--check 模式，供 ctest 调用）：CSV 结构 + 统一 ENU 原点不变量。

    不校验具体数值语义（场景相关），只封住三类回归：
    1. CSV 缺失/表头列名与 build_viewer 解析不一致 → 查看器静默空白；
    2. 行数异常（每机 platform 行数 ≠ 周期数）；
    3. 各数据集 ENU 投影原点不统一 → 真值/融合图层与平台轨迹重叠。
    """
    problems = []
    for name, expected in EXPECTED_HEADERS.items():
        path = data_dir.rstrip("/") + "/" + name
        rows = load_csv(path)
        if rows is None:
            if name in OPTIONAL_FILES:
                continue  # 可选文件：查看器跳过对应图层
            problems.append("%s 缺失" % name)
            continue
        # 表头取文件首行（header-only 文件无数据行，如 FD OFF 时 waypoint_events）。
        with open(path, newline="") as f:
            header = f.readline().strip()
        if header != expected:
            problems.append("%s 表头不匹配：%s" % (name, header))
    if problems:
        return problems

    data = build_data(data_dir)
    meta = data["meta"]
    if not meta.get("cycles"):
        problems.append("platform_track.csv 为空")
        return problems
    # 每机行数 == 周期数（多机契约：各机独立轨迹）。
    per_ac = {}
    for q in data["platform"]:
        per_ac.setdefault(q["ac"], 0)
        per_ac[q["ac"]] += 1
    for ac, count in sorted(per_ac.items()):
        if count != meta["cycles"]:
            problems.append("platform_track.csv 飞行器 %d 行数 %d != 周期数 %d"
                            % (ac, count, meta["cycles"]))
    # 统一原点不变量：真值/融合点必须落在平台原点之外（>1 km），
    # 否则说明投影又退回"各数据集各自原点"。
    for key, label in (("truth", "target_truth.csv"), ("fused", "fused_tracks.csv")):
        pts = [p for p in data.get(key, []) if p.get("x") is not None]
        if pts and not any((p["x"] ** 2 + p["y"] ** 2) ** 0.5 > 1000.0 for p in pts):
            problems.append("%s 所有点距平台原点 < 1 km（ENU 原点疑似未统一）" % label)
    return problems


def main():
    parser = argparse.ArgumentParser(
        description="从 component_entt_demo 的 CSV 导出构建交互式 HTML 查看器",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    parser.add_argument("data_dir", help="component_entt_demo 的 CSV 输出目录（--output-dir）")
    parser.add_argument("--out", default=None,
                        help="输出 HTML 路径（默认 <data_dir>/component_entt_viewer.html）")
    parser.add_argument("--check", action="store_true",
                        help="仅校验数据与投影（不生成 HTML），供 ctest 回归")
    args = parser.parse_args()

    if args.check:
        problems = validate_data(args.data_dir)
        if problems:
            for p in problems:
                sys.stderr.write("CHECK FAIL: %s\n" % p)
            sys.exit(1)
        print("CHECK PASS: %s（%d 周期）" % (args.data_dir,
                                           build_data(args.data_dir)["meta"]["cycles"]))
        return

    missing = [name for name in CSV_FILES if not os.path.exists(
        args.data_dir.rstrip("/") + "/" + name)]
    if missing:
        sys.stderr.write("警告：数据目录缺少 CSV（查看器将跳过对应图层）：%s\n" % ", ".join(missing))

    data = build_data(args.data_dir)
    cycles = data["meta"]["cycles"]
    if not cycles:
        sys.stderr.write("错误：无法从 %s 读取有效数据（platform_track.csv 缺失或为空）\n" % args.data_dir)
        sys.exit(1)

    out_path = args.out or args.data_dir.rstrip("/") + "/component_entt_viewer.html"
    data_json = json.dumps(data, ensure_ascii=False, separators=(",", ":"))
    data_json = data_json.replace("</", "<\\/")  # 防止 </script> 提前闭合

    model = data["meta"].get("model", "unknown")
    aircraft = data["meta"].get("aircraft", 1)
    title = model + " · " + str(cycles) + " 周期 · " + str(aircraft) + " 机"
    # 模板用 {{ }} 转义 JS/CSS 花括号：先解转义，再替换占位符（JSON 花括号不受影响）。
    # model/data_dir 来自 CSV/命令行，按不可信输入转义后嵌入 HTML。
    template = HTML_TEMPLATE.replace("{{", "{").replace("}}", "}")
    page = (
        template.replace("{title}", html.escape(title))
        .replace("{data_dir}", html.escape(args.data_dir))
        .replace("{model}", html.escape(model))
        .replace("{aircraft}", str(aircraft))
        .replace("{cycles}", str(cycles))
        .replace("{min_samples}", str(DEFAULT_MIN_SAMPLES))
        .replace("{COLOR_PLATFORM}", COLOR_PLATFORM)
        .replace("{COLOR_AR}", COLOR_AR)
        .replace("{COLOR_EOS}", COLOR_EOS)
        .replace("{COLOR_ESR}", COLOR_ESR)
        .replace("{COLOR_FUSED}", COLOR_FUSED)
        .replace("{COLOR_CHURN}", COLOR_CHURN)
        .replace("{COLOR_ZONE}", COLOR_ZONE)
        .replace("{esr_ray_len}", str(int(ESR_RAY_LENGTH_M)))
        .replace("{data_json}", data_json)
    )
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(page)

    print("已生成查看器：%s" % out_path)
    print("  数据：%s（%d 周期，平台模型 %s）" % (args.data_dir, cycles, model))
    print("  打开方式：open %s" % out_path)


if __name__ == "__main__":
    main()
