#!/usr/bin/env python3
"""Turn a recorded flight into a small tracer test dataset.

Reads the /map messages (nav_msgs/OccupancyGrid, published by patrol_lidar at
1 Hz) from a ros2 bag, keeps only which cells were occupied at each update, and
stores that with the world's ground truth (where each real object is and which
label it should get) in a compressed .npz. A 40 MB bag becomes well under 1 MB.

    ros2 bag record -o my_flight /map            # while patrol_lidar flies
    python3 make_dataset.py my_flight <world.sdf> data/<name>.npz [--radius 1.0]

--radius: how close (m) an object's footprint must come to a real object to
count as finding it. 1.0 for trunks and boxes; 3.0 for leafy trees, whose canopy
spreads around the trunk position given in the world file.

Needs ROS 2 sourced (rosbag2_py, nav_msgs).
"""
import argparse
import xml.etree.ElementTree as ET

import numpy as np
import rosbag2_py
from nav_msgs.msg import OccupancyGrid
from rclpy.serialization import deserialize_message

# patrol_lidar publishes round(p * 100); its "occupied" test is log-odds > 1.0,
# i.e. p > 0.731, i.e. a published value >= 74.
OCCUPIED_MIN = 74


def ground_truth(world_file):
    """[(x, y, expected_kinds)] for every object in a Gazebo world file.

    Plain static models are labelled from their collision shape, with the same
    size rules as TracedObject.kind(); <include>d Fuel tree models are 'tree'
    (pieces of a canopy may also come out as post or object).
    """
    world = ET.parse(world_file).getroot().find('world')
    truth = []
    for model in world.findall('model'):
        pose = (model.findtext('pose') or '0 0 0').split()
        geom = model.find('link/collision/geometry')
        if geom is None or geom.find('plane') is not None:
            continue                                   # ground
        if geom.find('cylinder') is not None:
            d = 2 * float(geom.findtext('cylinder/radius'))
            length, thick = d, d
        elif geom.find('box') is not None:
            sx, sy = (float(v) for v in geom.findtext('box/size').split()[:2])
            length, thick = max(sx, sy), min(sx, sy)
        else:
            continue
        if length < 1.0:
            kinds = 'post'
        elif length >= 1.5:
            kinds = 'wall' if length >= 4.0 * thick else 'tree'
        else:
            kinds = 'object'
        truth.append((float(pose[0]), float(pose[1]), kinds))
    for inc in world.findall('include'):
        uri = inc.findtext('uri') or ''
        if 'Tree' in uri:
            x, y = (float(v) for v in inc.findtext('pose').split()[:2])
            truth.append((x, y, 'tree|post|object'))
    return truth


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('world')
    ap.add_argument('out')
    ap.add_argument('--radius', type=float, default=1.0)
    a = ap.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    cells, offsets, info = [], [0], None
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic != '/map':
            continue
        msg = deserialize_message(raw, OccupancyGrid)
        info = msg.info
        occ = np.flatnonzero(np.asarray(msg.data, dtype=np.int16) >= OCCUPIED_MIN)
        cells.append(occ.astype(np.int32))
        offsets.append(offsets[-1] + occ.size)
    if info is None:
        raise SystemExit(f'No /map messages in {a.bag}')

    truth = ground_truth(a.world)
    np.savez_compressed(
        a.out,
        res=info.resolution, n=info.width, origin=info.origin.position.x,
        cells=np.concatenate(cells), offsets=np.array(offsets, dtype=np.int64),
        truth_xy=np.array([(x, y) for x, y, _ in truth]),
        truth_kinds=np.array([k for _, _, k in truth]),
        radius=a.radius)
    print(f'{a.out}: {len(offsets) - 1} map updates, {len(truth)} ground-truth objects')


if __name__ == '__main__':
    main()
