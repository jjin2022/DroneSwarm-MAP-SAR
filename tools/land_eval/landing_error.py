#!/usr/bin/env python3
"""How close to the pad did the drone land? Measured against the world file.

For each recorded flight: where the drone came to rest (its last position) versus
the true pad centre (the `landing_pad` model in the Gazebo world), plus how good
pad_detector's estimates were if /landing_pad was recorded too.

    ros2 bag record -o ~/land_1 /fmu/out/vehicle_local_position_v1 /landing_pad \
        --qos-profile-overrides-path ../avoid_eval/qos.yaml
    python3 landing_error.py <world.sdf> ~/land_1 [~/land_2 ...]

Uses the simulator's true pose (/sim/ground_truth/odometry) when it was recorded:
needed without GPS, where PX4's own estimate drifts. Otherwise uses PX4's local
position and assumes the drone spawned at the world origin (--spawn E N if not).
If robot_sim's /sim/ground_truth/robot was recorded (the robot moved during the
flight), the pad position is where the robot ended up, not where the world file
put it. Note: pad_detector's estimates are in PX4's frame, so with drift their
"off by" includes the drift.
Needs ROS 2 and px4_msgs sourced.
"""
import argparse
import xml.etree.ElementTree as ET

import numpy as np
import rosbag2_py
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from rclpy.serialization import deserialize_message

PAD_HALF = 0.25     # PX4's arucotag is 0.5 x 0.5 m


def pad_position(world_file):
    """(E, N) of the pad: the include/model named landing_pad, or the robot_dock
    (pad centred on its back)."""
    world = ET.parse(world_file).getroot().find('world')
    for el in list(world.findall('include')) + list(world.findall('model')):
        name = el.findtext('name') if el.tag == 'include' else el.get('name')
        if name in ('landing_pad', 'robot_dock'):
            e, n = (float(v) for v in el.findtext('pose').split()[:2])
            return e, n
    raise SystemExit(f'No landing_pad or robot_dock in {world_file}')


def read(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    pos, pads, truth, robot = [], [], [], None
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic == '/sim/ground_truth/robot':
            m = deserialize_message(raw, PoseStamped).pose.position
            robot = (m.x, m.y)                                  # world E, N (latest wins)
        elif topic == '/sim/ground_truth/odometry':
            p = deserialize_message(raw, Odometry).pose.pose.position
            truth.append((p.x, p.y, -p.z))                      # world E, N, down
        elif topic.endswith('vehicle_local_position_v1'):
            m = deserialize_message(raw, VehicleLocalPosition)
            pos.append((m.y, m.x, m.z))                       # E, N, down
        elif topic.endswith('landing_pad'):
            m = deserialize_message(raw, PointStamped)
            pads.append((m.point.y, m.point.x))               # E, N
    if truth:          # true pose, world frame: needs no spawn offset
        return np.array(truth), np.array(pads), True, robot
    if not pos:
        raise SystemExit(f'No vehicle_local_position_v1 or ground truth in {bag}')
    return np.array(pos), np.array(pads), False, robot


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('world')
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--spawn', type=float, nargs=2, default=(0.0, 0.0), metavar=('E', 'N'))
    a = ap.parse_args()

    pad_world = pad_position(a.world)
    print(f'pad centre in the world: E={pad_world[0]:+.2f} N={pad_world[1]:+.2f}')
    print(f'{"flight":24s} {"touchdown E, N":>16s} {"error":>8s} {"on pad":>7s} '
          f'{"took off":>9s} {"pad estimates":>22s}')
    errors = []
    for bag in a.bags:
        pos, pads, is_truth, robot = read(bag)
        # The robot may have moved (robot_sim records where): the pad is on its back.
        pad = robot if robot is not None else pad_world
        # Ground truth is in world coordinates; PX4's estimate is relative to spawn.
        pe, pn = pad if is_truth else (pad[0] - a.spawn[0], pad[1] - a.spawn[1])
        if robot is not None:
            print(f'  {bag.rstrip("/").split("/")[-1]}: robot (pad) ended at E={pe:+.2f} N={pn:+.2f}')
        end, start = pos[-1], pos[0]
        if end[2] - pos[:, 2].min() < 1.0:
            print(f'{bag.rstrip("/").split("/")[-1]:24s} never flew; skipped')
            continue
        de, dn = end[0] - pe, end[1] - pn
        err = float(np.hypot(de, dn))
        errors.append(err)
        on_pad = abs(de) <= PAD_HALF and abs(dn) <= PAD_HALF
        took_off = float(np.hypot(start[0] - pe, start[1] - pn))
        est = (f'{len(pads)}, off by {np.hypot(pads[:, 0].mean() - pe, pads[:, 1].mean() - pn) * 100:.1f} cm'
               if len(pads) else '-')
        print(f'{bag.rstrip("/").split("/")[-1]:24s} {end[0]:+7.2f}, {end[1]:+6.2f} '
              f'{err * 100:6.1f} cm {"yes" if on_pad else "NO":>7s} {took_off:7.2f} m {est:>22s}'
              f'{"" if is_truth else "  (PX4 estimate)"}')
    if len(errors) > 1:
        e = np.array(errors) * 100
        print(f'\n{len(e)} landings: mean {e.mean():.1f} cm, max {e.max():.1f} cm, '
              f'{sum(x <= PAD_HALF * 100 for x in e)} within {PAD_HALF * 100:.0f} cm')
    print('\n"took off" = distance from take-off to the pad; "pad estimates" = how far '
          "pad_detector's average was from the true centre.")


if __name__ == '__main__':
    main()
