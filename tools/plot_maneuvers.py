#!/usr/bin/env python3
import os
import glob
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Earth radius in meters
R_EARTH = 6378137.0

def parse_csv(filepath):
    with open(filepath, 'r') as f:
        lines = f.readlines()
    
    target_wp = None
    target_val = None
    data_lines = []
    
    for line in lines:
        line = line.strip()
        if not line:
            continue
        if line.startswith("#TARGET,"):
            parts = line.split(',')
            target_wp = (float(parts[1]), float(parts[2]), float(parts[3]))
        elif line.startswith("#TARGET_VAL,"):
            target_val = float(line.split(',')[1])
        else:
            data_lines.append(line)
            
    import io
    df = pd.read_csv(io.StringIO("\n".join(data_lines)))
    return df, target_wp, target_val

def plot_trajectory(csv_file, output_dir):
    filename = os.path.basename(csv_file)
    name_no_ext = os.path.splitext(filename)[0]
    
    parts = name_no_ext.split('_')
    maneuver = parts[0]
    model = "_".join(parts[1:])
    
    df, target_wp, target_val = parse_csv(csv_file)
    if len(df) == 0:
        print(f"Skipping empty data for {filename}")
        return
        
    fig, ax = plt.subplots(figsize=(8, 6))
    title = f"Maneuver: {maneuver} | Model: {model}"
    ax.set_title(title)
    
    if maneuver in ["FlyToWaypoint", "Orbit", "SetHeading", "SetRoll"]:
        # Top-down view (Local X/Y)
        lat0 = df['lat_rad'].iloc[0]
        lon0 = df['lon_rad'].iloc[0]
        
        x = (df['lon_rad'] - lon0) * R_EARTH * np.cos(lat0)
        y = (df['lat_rad'] - lat0) * R_EARTH
        
        ax.plot(x, y, label="Trajectory", color='blue')
        ax.scatter([x.iloc[0]], [y.iloc[0]], color='green', s=100, label='Start', zorder=5)
        ax.scatter([x.iloc[-1]], [y.iloc[-1]], color='red', s=100, label='End', zorder=5)
        
        if target_wp:
            tx = (target_wp[1] - lon0) * R_EARTH * np.cos(lat0)
            ty = (target_wp[0] - lat0) * R_EARTH
            ax.scatter([tx], [ty], color='orange', s=150, marker='*', label='Target WP', zorder=5)
            
        ax.set_xlabel("Local X (m)")
        ax.set_ylabel("Local Y (m)")
        ax.grid(True)
        ax.legend()
        ax.axis('equal')
        
    elif maneuver == "SetAltitudeClimb":
        # Side view: Altitude vs Time
        ax.plot(df['time_sec'], df['alt_m'], label="Altitude", color='blue')
        ax.scatter([df['time_sec'].iloc[0]], [df['alt_m'].iloc[0]], color='green', s=100, label='Start', zorder=5)
        ax.scatter([df['time_sec'].iloc[-1]], [df['alt_m'].iloc[-1]], color='red', s=100, label='End', zorder=5)
        
        if target_val is not None:
            ax.axhline(target_val, color='orange', linestyle='--', label=f"Target Alt ({target_val}m)")
            
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Altitude (m)")
        ax.grid(True)
        ax.legend()
        
    elif maneuver == "SetPitch":
        ax.plot(df['time_sec'], np.degrees(df['pitch_rad']), label="Pitch", color='blue')
        ax.scatter([df['time_sec'].iloc[0]], [np.degrees(df['pitch_rad'].iloc[0])], color='green', s=100, label='Start', zorder=5)
        ax.scatter([df['time_sec'].iloc[-1]], [np.degrees(df['pitch_rad'].iloc[-1])], color='red', s=100, label='End', zorder=5)
        
        if target_val is not None:
            ax.axhline(target_val, color='orange', linestyle='--', label=f"Target Pitch ({target_val} deg)")
            
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Pitch (deg)")
        ax.grid(True)
        ax.legend()
        
    elif maneuver == "SetRoll":
        ax.plot(df['time_sec'], np.degrees(df['roll_rad']), label="Roll", color='blue')
        ax.scatter([df['time_sec'].iloc[0]], [np.degrees(df['roll_rad'].iloc[0])], color='green', s=100, label='Start', zorder=5)
        ax.scatter([df['time_sec'].iloc[-1]], [np.degrees(df['roll_rad'].iloc[-1])], color='red', s=100, label='End', zorder=5)
        
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Roll (deg)")
        ax.grid(True)
        ax.legend()
        
    maneuver_dir = os.path.join(output_dir, maneuver)
    os.makedirs(maneuver_dir, exist_ok=True)
    
    out_path = os.path.join(maneuver_dir, f"{model}.png")
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Generated {out_path}")

def main():
    traj_dir = "/tmp/1q_trajectories"
    out_dir = "/Users/aurora/Code/1q/docs/maneuvers"
    
    if not os.path.exists(traj_dir):
        print(f"Directory {traj_dir} not found. Run tests with DUMP_MANEUVER_TRAJECTORY=1 first.")
        return
        
    csv_files = glob.glob(os.path.join(traj_dir, "*.csv"))
    if not csv_files:
        print(f"No CSV files found in {traj_dir}.")
        return
        
    for csv_file in csv_files:
        plot_trajectory(csv_file, out_dir)

if __name__ == "__main__":
    main()
