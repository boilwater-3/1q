#!/usr/bin/env python3
"""
盘旋（Orbit）轨迹可视化工具 — 用法：
  python3 orbit_visualize.py <trace_csv>

trace_csv 由 orbit_trace_csv 工具生成，包含逐点 lat_rad/lon_rad/dist_m/err_pct/alt_m

输出：
  - /tmp/orbit_trace.png (4面板可视化)
  - /tmp/orbit_trajectory.kml (Google Earth 轨迹)
"""

import csv
import math
import sys

KML_TEMPLATE = """<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
<Document>
  <name>Orbit Trajectory</name>
  {placemarks}
</Document>
</kml>
"""

POINT_TPL = """  <Placemark>
    <name>{label}</name>
    <Point><coordinates>{lon},{lat},{alt}</coordinates></Point>
  </Placemark>"""

TRACK_TPL = """  <Placemark>
    <name>Trajectory</name>
    <LineString>
      <tessellate>1</tessellate>
      <coordinates>
{coords}
      </coordinates>
    </LineString>
  </Placemark>"""

CIRCLE_TPL = """  <Placemark>
    <name>Target Orbit (radius={r}m)</name>
    <LineString>
      <tessellate>1</tessellate>
      <altitudeMode>absolute</altitudeMode>
      <coordinates>
{coords}
      </coordinates>
    </LineString>
  </Placemark>"""


def lonlat_to_meters(center_lat_rad, center_lon_rad, target_lat_rad, target_lon_rad):
    """Approximate local Cartesian offset from center (m)."""
    R = 6378137.0
    cos_lat = math.cos(center_lat_rad)
    dn = (target_lat_rad - center_lat_rad) * R
    de = (target_lon_rad - center_lon_rad) * R * cos_lat
    return dn, de


def generate_circle(center_lat_rad, center_lon_rad, radius_m, alt_m, n_pts=72):
    """Generate points of a circle in lat/lon around center."""
    R = 6378137.0
    cos_lat = math.cos(center_lat_rad)
    pts = []
    for i in range(n_pts + 1):
        angle = 2.0 * math.pi * i / n_pts
        dn = radius_m * math.cos(angle)
        de = radius_m * math.sin(angle)
        lat = center_lat_rad + dn / R
        lon = center_lon_rad + de / (R * cos_lat)
        pts.append((lat, lon))
    return pts


def write_kml(trace_csv, out_kml):
    """Convert trace CSV to KML."""
    rows = []
    with open(trace_csv) as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)

    if not rows:
        print("  No data")
        return

    # 从第1行读目标半径
    radius_m = float(rows[0]["radius_m"])
    center_lat = float(rows[0]["lat_rad"]) -  radius_m / 6378137.0  # approximate
    center_lon = float(rows[0]["lon_rad"])

    # 轨迹坐标
    coords_track = "\n".join(
        f"      {r['lon_rad']*180/math.pi:.8f},{r['lat_rad']*180/math.pi:.8f},{r['alt_m']:.1f}"
        for r in rows[::5])  # 每5行采样一次降低 KML 大小

    # 期望圆
    circle_pts = generate_circle(center_lat, center_lon, radius_m, float(rows[0]["alt_m"]))
    coords_circle = "\n".join(
        f"      {lon*180/math.pi:.8f},{lat*180/math.pi:.8f},{rows[0]['alt_m']}"
        for lat, lon in circle_pts)

    # 中心点
    center_pt = POINT_TPL.format(
        label=f"Center (r={radius_m:.0f}m)",
        lon=center_lon * 180 / math.pi,
        lat=center_lat * 180 / math.pi,
        alt=rows[0]["alt_m"])

    placemarks = (
        TRACK_TPL.format(coords=coords_track) + "\n" +
        CIRCLE_TPL.format(r=radius_m, coords=coords_circle) + "\n" +
        center_pt
    )

    kml = KML_TEMPLATE.replace("{placemarks}", placemarks)
    with open(out_kml, "w") as f:
        f.write(kml)
    print(f"  KML written to {out_kml}")


def print_plot_cmd(trace_csv):
    """Print matplotlib command."""
    print(f"""
  Plot with: python3 << 'EOF'
import pandas as pd, matplotlib.pyplot as plt
d = pd.read_csv('{trace_csv}')
r = d.radius_m[0]

fig, ((a1, a2), (a3, a4)) = plt.subplots(2, 2, figsize=(14, 8))
fig.suptitle('Orbit Trace: r={r:.0f}m, {len(open(trace_csv).readlines())-1} samples')

# 距离 vs 时间
a1.plot(d.sim_time_sec, d.dist_m, lw=0.8)
a1.axhline(r, c='r', ls='--', lw=1, label=f'target r={{r:.0f}}')
a1.set_ylabel('Distance to center (m)')
a1.set_xlabel('Time (s)')
a1.legend(); a1.grid(True, alpha=0.3)

# 误差百分比
a2.plot(d.sim_time_sec, d.err_pct, lw=0.8)
a2.axhline(0, c='r', ls='--', lw=1)
a2.set_ylabel('Error (%)')
a2.set_xlabel('Time (s)')
a2.grid(True, alpha=0.3)

# 轨迹 (lat/lon, 颜色=距离)
sc = a3.scatter(d.lon_rad, d.lat_rad, s=2, c=d.dist_m, cmap='viridis')
plt.colorbar(sc, ax=a3, label='dist (m)')
a3.set_title('Trajectory (color=distance)')
a3.set_aspect('equal')
a3.grid(True, alpha=0.3)

# 高度
a4.plot(d.sim_time_sec, d.alt_m, lw=0.8, color='orange')
a4.set_ylabel('Altitude (m)')
a4.set_xlabel('Time (s)')
a4.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('/tmp/orbit_trace.png', dpi=150)
print('Saved /tmp/orbit_trace.png')
EOF
""")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    trace_csv = sys.argv[1]
    print(f"Analyzing {trace_csv}...")

    write_kml(trace_csv, "/tmp/orbit_trajectory.kml")
    print_plot_cmd(trace_csv)

    print(f"""
  1. KML: Open /tmp/orbit_trajectory.kml in Google Earth
     → 红圈 = 期望轨道，蓝线 = 实际轨迹，标记 = 中心点

  2. PNG: Run the printed matplotlib command above
     → 4面板: 距离/误差/轨迹/高度
""")
