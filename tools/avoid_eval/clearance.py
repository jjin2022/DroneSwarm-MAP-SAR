#!/usr/bin/env python3
"""How close did the drone really get to obstacles? Measured against the world file.

Reads the drone's position from a recorded flight and computes, at every sample
while flying at patrol altitude, the horizontal distance from the drone's centre
to the nearest obstacle surface (cylinders and boxes in the Gazebo world). This
is ground truth, independent of what the lidar or map believed.

    ros2 bag record -o ~/my_flight /fmu/out/vehicle_local_position_v1 \
        --qos-profile-overrides-path qos.yaml      # PX4 publishes best-effort
    python3 clearance.py ~/my_flight \
        ~/dev/DroneSwarm-MAP-SAR/src/las_offboard/worlds/forest_pillars.sdf

qos.yaml (needed, or the recorder gets nothing):
    /fmu/out/vehicle_local_position_v1: {reliability: best_effort, durability: volatile, history: keep_last, depth: 10}

Prints the closest approach, time spent inside each clearance band, and every
pass closer than --clearance (default 0.8 m = patrol_lidar's avoid_radius_m),
with where the obstacle was relative to the direction of travel. The x500's
propeller tips reach about 0.38 m from its centre, so below that is contact.

Assumes home (local position 0, 0) is the world origin, as when PX4 spawns the
drone at (0, 0). Needs ROS 2 and px4_msgs sourced.
"""
import argparse
import math
import xml.etree.ElementTree as ET

import numpy as np
import rosbag2_py
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from rclpy.serialization import deserialize_message

PROP_TIP_M = 0.38


def obstacles(world_file):
    """[(name, kind, x, y, yaw, a, b)]: cylinder a = radius; box a, b = half sizes."""
    world = ET.parse(world_file).getroot().find('world')
    out = []
    for model in world.findall('model'):
        pose = [float(v) for v in (model.findtext('pose') or '0 0 0 0 0 0').split()]
        pose += [0.0] * (6 - len(pose))
        geom = model.find('link/collision/geometry')
        if geom is None:
            continue
        if geom.find('cylinder') is not None:
            out.append((model.get('name'), 'cyl', pose[0], pose[1], pose[5],
                        float(geom.findtext('cylinder/radius')), 0.0))
        elif geom.find('box') is not None:
            sx, sy = (float(v) for v in geom.findtext('box/size').split()[:2])
            out.append((model.get('name'), 'box', pose[0], pose[1], pose[5], sx / 2, sy / 2))
    return out


def surface_distance(e, n, obs):
    """Distance (N,) from points (e, n) to one obstacle's surface (0 inside)."""
    _, kind, x, y, yaw, a, b = obs
    if kind == 'cyl':
        return np.maximum(np.hypot(e - x, n - y) - a, 0.0)
    c, s = math.cos(-yaw), math.sin(-yaw)          # into the box's own frame
    u = c * (e - x) - s * (n - y)
    v = s * (e - x) + c * (n - y)
    return np.hypot(np.maximum(np.abs(u) - a, 0.0), np.maximum(np.abs(v) - b, 0.0))


def read_positions(bag):
    """(t, e, n, down, ve, vn) rows, and the source used.

    Uses the simulator's true pose (/sim/ground_truth/odometry, world frame) when it
    was recorded: without GPS, PX4's own estimate drifts and would hide the very
    errors being measured. Otherwise PX4's local position (accurate with sim GPS;
    assumes the drone spawned at the world origin).
    """
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    truth, local = [], []
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if topic == '/sim/ground_truth/odometry':
            p = deserialize_message(raw, Odometry).pose.pose.position
            truth.append((stamp * 1e-9, p.x, p.y, -p.z))
        elif topic.endswith('vehicle_local_position_v1'):
            m = deserialize_message(raw, VehicleLocalPosition)
            local.append((stamp * 1e-9, m.y, m.x, m.z, m.vy, m.vx))   # ENU e, n; z down
    if truth:
        a = np.array(truth)
        ve = np.gradient(a[:, 1], a[:, 0])
        vn = np.gradient(a[:, 2], a[:, 0])
        return np.column_stack([a, ve, vn]), 'simulator ground truth'
    if not local:
        raise SystemExit(f'No vehicle_local_position_v1 or ground truth in {bag} '
                         '(see qos.yaml in --help)')
    return np.array(local), "PX4's position estimate"



def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('world')
    ap.add_argument('--clearance', type=float, default=0.8)
    ap.add_argument('--min-alt', type=float, default=1.5,
                    help='only count samples above this height (m), i.e. patrolling')
    a = ap.parse_args()

    obs = obstacles(a.world)
    pos, source = read_positions(a.bag)
    pos = pos[-pos[:, 3] > a.min_alt]
    if not len(pos):
        raise SystemExit('The drone never flew above --min-alt.')
    t, e, n, ve, vn = pos[:, 0] - pos[0, 0], pos[:, 1], pos[:, 2], pos[:, 4], pos[:, 5]
    dist = np.stack([surface_distance(e, n, o) for o in obs], axis=1)
    nearest = dist.argmin(axis=1)
    c = dist[np.arange(len(dist)), nearest]
    # Time each sample stands for. The recorder sometimes drops best-effort
    # messages for seconds at a time; cap each sample at 3x the normal spacing so
    # a gap isn't counted as time spent wherever the drone was before it.
    step = np.diff(t, append=t[-1])
    normal = float(np.median(step[:-1])) if len(step) > 1 else 0.02
    dt = np.minimum(step, 3 * normal)
    gaps = step > 0.5

    print(f'{len(c)} samples over {t[-1]:.0f} s in the air (from {source})')
    if gaps.any():
        print(f'note: {gaps.sum()} gaps in the recording ({step[gaps].sum():.0f} s with no'
              f' position data, longest {step.max():.1f} s); times below cover the rest')
    print(f'closest approach: {c.min():.2f} m to {obs[nearest[c.argmin()]][0]}'
          f'  (propeller tips at ~{PROP_TIP_M} m: below that is contact)')
    for band in (PROP_TIP_M, 0.5, a.clearance):
        print(f'  time closer than {band:.2f} m: {dt[c < band].sum():5.1f} s')

    passes, i = [], 0
    while i < len(c):
        if c[i] >= a.clearance:
            i += 1
            continue
        j = i
        while j < len(c) and c[j] < a.clearance:
            j += 1
        k = i + int(np.argmin(c[i:j]))
        o = obs[nearest[k]]
        rel = np.array([o[2] - e[k], o[3] - n[k]])
        vel = np.array([ve[k], vn[k]])
        speed = float(np.hypot(*vel))
        bearing = (abs(math.degrees(math.atan2(vel[0] * rel[1] - vel[1] * rel[0], vel @ rel)))
                   if speed > 0.1 else float('nan'))
        passes.append((c[k], t[k], o[0], e[k], n[k], speed, bearing, t[j - 1] - t[i]))
        i = j
    print(f'\n{len(passes)} passes closer than {a.clearance} m'
          f' (bearing: obstacle direction vs travel, 0 = ahead, 90 = beside, 180 = behind)')
    for cm, tk, name, pe, pn, sp, br, dur in sorted(passes)[:15]:
        print(f'  {cm:.2f} m  t={tk:6.1f}s  {name:14s} drone at E={pe:5.1f} N={pn:5.1f}'
              f'  {sp:.1f} m/s  bearing {br:4.0f} deg  for {dur:.1f} s')


if __name__ == '__main__':
    main()
