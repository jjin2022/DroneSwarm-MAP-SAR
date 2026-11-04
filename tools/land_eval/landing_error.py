#!/usr/bin/env python3
"""How close to the pad did the drone land? Measured against the world file.

For each recorded flight: where the drone came to rest (its last position) versus
the true pad centre (the `landing_pad` model in the Gazebo world), plus how good
pad_detector's estimates were if /landing_pad was recorded too.

    ros2 bag record -o ~/land_1 /fmu/out/vehicle_local_position_v1 /landing_pad \
        --qos-profile-overrides-path ../avoid_eval/qos.yaml
    python3 landing_error.py <world.sdf> ~/land_1 [~/land_2 ...]

Assumes the drone spawned at the world origin (PX4's default), so PX4's local
frame (NED) and the world (ENU) differ only in axis order. Use --spawn E N if not.
Needs ROS 2 and px4_msgs sourced.
"""
import argparse
import xml.etree.ElementTree as ET

import numpy as np
import rosbag2_py
from geometry_msgs.msg import PointStamped
from px4_msgs.msg import VehicleLocalPosition
from rclpy.serialization import deserialize_message

PAD_HALF = 0.25     # PX4's arucotag is 0.5 x 0.5 m


def pad_position(world_file):
    """(E, N) of the include or model named landing_pad."""
    world = ET.parse(world_file).getroot().find('world')
    for el in list(world.findall('include')) + list(world.findall('model')):
        name = el.findtext('name') if el.tag == 'include' else el.get('name')
        if name == 'landing_pad':
            e, n = (float(v) for v in el.findtext('pose').split()[:2])
            return e, n
    raise SystemExit(f'No landing_pad in {world_file}')


def read(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    pos, pads = [], []
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic.endswith('vehicle_local_position_v1'):
            m = deserialize_message(raw, VehicleLocalPosition)
            pos.append((m.y, m.x, m.z))                       # E, N, down
        elif topic.endswith('landing_pad'):
            m = deserialize_message(raw, PointStamped)
            pads.append((m.point.y, m.point.x))               # E, N
    if not pos:
        raise SystemExit(f'No vehicle_local_position_v1 in {bag}')
    return np.array(pos), np.array(pads)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('world')
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--spawn', type=float, nargs=2, default=(0.0, 0.0), metavar=('E', 'N'))
    a = ap.parse_args()

    pe, pn = pad_position(a.world)
    pe, pn = pe - a.spawn[0], pn - a.spawn[1]                 # into PX4's local frame
    print(f'pad centre: E={pe:+.2f} N={pn:+.2f} m from the spawn point')
    print(f'{"flight":24s} {"touchdown E, N":>16s} {"error":>8s} {"on pad":>7s} '
          f'{"took off":>9s} {"pad estimates":>22s}')
    errors = []
    for bag in a.bags:
        pos, pads = read(bag)
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
              f'{err * 100:6.1f} cm {"yes" if on_pad else "NO":>7s} {took_off:7.2f} m {est:>22s}')
    if len(errors) > 1:
        e = np.array(errors) * 100
        print(f'\n{len(e)} landings: mean {e.mean():.1f} cm, max {e.max():.1f} cm, '
              f'{sum(x <= PAD_HALF * 100 for x in e)} within {PAD_HALF * 100:.0f} cm')
    print('\n"took off" = distance from take-off to the pad; "pad estimates" = how far '
          "pad_detector's average was from the true centre.")


if __name__ == '__main__':
    main()
