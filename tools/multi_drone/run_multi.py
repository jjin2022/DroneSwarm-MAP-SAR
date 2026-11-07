#!/usr/bin/env python3
"""Two drones patrol the forest from one robot, each its own strip, and land back
on their own pads. Phase 5 groundwork (multi-drone), in simulation.

    python3 tools/multi_drone/run_multi.py            # watch it in Gazebo
    python3 tools/multi_drone/run_multi.py --headless # no windows (testing)

Source ROS, px4_ws and this workspace first (as for tools/sortie_runner). Stops by
itself when both drones are done, or on Ctrl-C. Logs, a bag and a report go to
--out (default recordings/multi_<time>/).

THE SETUP
---------
World forest_pillars_robot_duo: the 30-trunk forest with robot_dock_duo, a robot
carrying two ArUco pads 1 m apart (ID 0 on its left, ID 1 on its right).

                     N
          drone 1    |    drone 0
          west strip |    east strip         each strip 4.25 x 10 m, flown as
         E -5..-0.75 |    E 0.75..5          a rectangle; the gap between the
            3.5 m    |    2.0 m              strips keeps them apart sideways,
                  [pad0]                     the altitudes keep them out of each
                  [pad1]  robot              other's (flat) lidar scan
                     |

Per drone (drone 0 uses the single-drone names; drone 1 the /px4_1 namespace):
PX4 instance (scripts/px4_nogps.sh, PX4_INSTANCE), lidar/camera bridge, sim_vio,
pad_detector (its marker ID), uwb_sim + uwb_locator (its pad offset) and
patrol_lidar (its strip, altitude and the robot's report for its pad). Shared:
one DDS agent, Gazebo, robot_sim (relocates the robot once both are airborne and
reports both pads). Drone 1 starts takeoff_delay_s after drone 0, so they don't
climb off the robot side by side at the same moment.

SCORING (report.txt), per drone: did it patrol, return, precision-land and
finish; is it resting on ITS pad (within the 0.5 m pad); closest trunk (< 0.38 m
counts as touching, as in the sortie runner); and for the pair, the closest the
two drones came while either was flying (< 0.75 m between centres: props touch).
"""
import argparse
import importlib.util
import math
import os
import re
import shutil
import sys
import time

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


rs = load(os.path.join(REPO, 'tools/sortie_runner/run_sorties.py'), 'run_sorties')
WORLD = 'forest_pillars_robot_duo'
WORLD_FILE = os.path.join(REPO, 'src/las_offboard/worlds', WORLD + '.sdf')
rs.WORLD, rs.WORLD_FILE = WORLD, WORLD_FILE          # rs.gz_world_up() waits for this world

# Pads in the robot's body frame (x forward, y left): must match robot_dock_duo.
PADS = [(0.0, 0.5), (0.0, -0.5)]
STRIP_E, STRIP_N = 4.25, 10.0                       # each drone's patrol rectangle
DRONES = [
    dict(ns='', alt=2.0, strip_e=2.875),             # east strip
    dict(ns='px4_1', alt=3.5, strip_e=-2.875),       # west strip
]
PLATFORM_Z = 0.67                                    # spawn height on the platform
PAD_HALF = 0.25
TOUCH_TRUNK_M = 0.38
TOUCH_DRONE_M = 0.75


def truth_topic(i):
    return (f'/{DRONES[i]["ns"]}' if DRONES[i]['ns'] else '') + '/sim/ground_truth/odometry'


def ros_topics():
    """Exact topic names (rs.ros_has matches substrings: /scan in /px4_1/scan)."""
    import subprocess
    try:
        out = subprocess.run(['bash', '-c', rs.ROS_ENV + 'ros2 topic list --no-daemon --spin-time 3'],
                             capture_output=True, text=True, timeout=30).stdout
    except subprocess.TimeoutExpired:
        return set()
    return set(out.split())


def fly(s, a):
    """Start everything and wait for both drones. Returns an error text or None."""
    step = rs.step
    s.start('agent', 'MicroXRCEAgent udp4 -p 8888', ros=False)
    s.start('gazebo', f'python3 {rs.PX4_DIR}/Tools/simulation/gz/simulation-gazebo --world {WORLD} '
            f'--model_store {rs.PX4_DIR}/Tools/simulation/gz' + (' --headless' if a.headless else ''),
            env=rs.clean_env(), ros=False)
    if not rs.wait_for(rs.gz_world_up, 120):
        return 'gazebo did not start'
    step('gazebo up')
    for i, (px, py) in enumerate(PADS):
        env = dict(os.environ, PX4_DAEMON='1', PX4_INSTANCE=str(i))
        s.start(f'px4_{i}', f'{REPO}/scripts/px4_nogps.sh {WORLD} {px},{py},{PLATFORM_Z}',
                env=env, ros=False)
        status = (f'/{DRONES[i]["ns"]}' if DRONES[i]['ns'] else '') + '/fmu/out/vehicle_status'
        if not rs.wait_for(lambda: rs.ros_has(status), 120, 3):
            return f'PX4 instance {i} did not start'
        step(f'PX4 {i} up')
        # No ground station: turn off the GCS-loss action (see run_sorties.py).
        if not rs.set_px4_param('NAV_DLL_ACT', 0, url=f'udpin:0.0.0.0:{14540 + i}'):
            return f'could not set NAV_DLL_ACT on PX4 {i}'
        step(f'PX4 {i}: NAV_DLL_ACT=0 set')

    seed = a.seed
    pads = ', '.join(f'{v:.2f}' for pad in PADS for v in pad)
    s.start('robot_sim', 'ros2 run las_offboard robot_sim --ros-args '
            f'-p world:={WORLD} -p world_file:={WORLD_FILE} -p seed:={seed} '
            f'-p relocate_after_s:={a.relocate_after} -p "pads:=[{pads}]" -p clearance_m:=1.5')
    for i, d in enumerate(DRONES):
        ns_arg = f'-r __ns:=/{d["ns"]}' if d['ns'] else ''
        tag = f'_{i}'
        px, py = PADS[i]
        s.start('bridge' + tag, f'ros2 launch las_offboard lidar_bridge.launch.py world:={WORLD} '
                f'model:=x500_lidar_cam_down_{i} camera:=true vio:=true'
                + (f' ns:={d["ns"]}' if d['ns'] else ''))   # launch rejects an empty ns:=
        s.start('sim_vio' + tag, f'ros2 run las_offboard sim_vio --ros-args {ns_arg} '
                f'-p seed:={seed + 100 * i}')
        s.start('pad_detector' + tag, f'ros2 run las_offboard pad_detector --ros-args {ns_arg} '
                f'-p marker_id:={i}')
        s.start('uwb_sim' + tag, f'ros2 run las_offboard uwb_sim --ros-args {ns_arg} '
                f'-p seed:={seed + 100 * i}')
        s.start('uwb_locator' + tag, f'ros2 run las_offboard uwb_locator --ros-args {ns_arg} '
                f'-p pad_x_m:={px} -p pad_y_m:={py}')
    need = {f'{pre}/{t}' for pre in ('', '/px4_1') for t in ('scan', 'camera/image_raw')}
    if not rs.wait_for(lambda: need <= ros_topics(), 60, 3):
        return 'bridges did not start'
    step('bridges up')
    topics = ['/sim/ground_truth/robot', '/robot/dock_position', '/robot/dock_position_1']
    topics += [truth_topic(i) for i in range(len(DRONES))]
    qos = os.path.join(REPO, 'tools/avoid_eval/qos.yaml')
    s.start('record', f'ros2 bag record -o {s.folder}/bag ' + ' '.join(topics)
            + f' --qos-profile-overrides-path {qos}')
    time.sleep(3)
    for i, d in enumerate(DRONES):
        if i:
            time.sleep(a.takeoff_delay)
        ns_arg = f'-r __ns:=/{d["ns"]}' if d['ns'] else ''
        # Strip centre from this drone's home (its pad), ENU.
        ce, cn = d['strip_e'] - PADS[i][0], -PADS[i][1]
        dock = '/robot/dock_position' + (f'_{i}' if i else '')
        s.start(f'patrol_lidar_{i}', f'ros2 run las_offboard patrol_lidar --ros-args {ns_arg} '
                f'-p altitude_m:={d["alt"]} -p "map_dir:=\'\'" -p return_battery:={a.return_battery} '
                f'-p patrol_size_m:={STRIP_E} -p patrol_size_n_m:={STRIP_N} '
                f'-p patrol_center_e_m:={ce} -p patrol_center_n_m:={cn} -p dock_topic:={dock}')
        step(f'patrol_lidar {i} started')
    logs = [os.path.join(s.folder, f'patrol_lidar_{i}.log') for i in range(len(DRONES))]
    finished = lambda: all(re.search(r'-> DONE|-> HANDOFF', open(f).read()) for f in logs)
    done = rs.wait_for(finished, a.timeout, 3)
    step('flight over' if done else 'flight timed out')
    return None if done else f'timed out after {a.timeout:.0f} s'


def read_bag(bag):
    import numpy as np
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    names = {truth_topic(i): i for i in range(len(DRONES))}
    tracks, robot = {i: [] for i in range(len(DRONES))}, None
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if topic in names:
            p = deserialize_message(raw, Odometry).pose.pose.position
            tracks[names[topic]].append((stamp * 1e-9, p.x, p.y, p.z))
        elif topic == '/sim/ground_truth/robot':
            m = deserialize_message(raw, PoseStamped).pose
            robot = (m.position.x, m.position.y, 2 * math.atan2(m.orientation.z, m.orientation.w))
    return {i: np.array(t) for i, t in tracks.items()}, robot


def score(folder):
    """(passed, report lines)."""
    import numpy as np
    clear = load(os.path.join(REPO, 'tools/avoid_eval/clearance.py'), 'avoid_eval')
    lines, ok = [], True
    try:
        tracks, robot = read_bag(os.path.join(folder, 'bag'))
    except Exception as exc:                          # a broken recording is a failure
        return False, [f'could not read the bag: {exc}']
    trunks = [o for o in clear.obstacles(WORLD_FILE) if o[0].startswith('trunk')]
    for i in range(len(DRONES)):
        log = open(os.path.join(folder, f'patrol_lidar_{i}.log')).read()
        why = [w for need, w in (('TAKEOFF -> PATROL', 'never patrolled'),
                                 ('PATROL -> RETURN', 'never returned'),
                                 ('PRECISION_LAND -> LAND', 'no precision landing'),
                                 ('LAND -> DONE', 'did not finish')) if need not in log]
        why += [w for bad, w in (('GO_HOME', 'gave up finding the robot'),
                                 ('HANDOFF', 'handed control back')) if bad in log]
        laps = len(re.findall(r'Lap \d+ done', log))
        src = (re.findall(r'from (UWB|robot radio|last known[^)]*\)|setting)', log) or ['?'])[-1]
        tr = tracks[i]
        if not len(tr) or robot is None:
            why.append('no ground truth recorded')
            lines.append(f'drone {i}: FAIL ({"; ".join(why)})')
            ok = False
            continue
        # Touchdown in the robot's frame, against this drone's pad.
        de, dn = tr[-1, 1] - robot[0], tr[-1, 2] - robot[1]
        c, s_ = math.cos(robot[2]), math.sin(robot[2])
        bx, by = c * de + s_ * dn - PADS[i][0], -s_ * de + c * dn - PADS[i][1]
        td = 100 * math.hypot(bx, by)
        if not (abs(bx) <= PAD_HALF and abs(by) <= PAD_HALF):
            why.append(f'not on its pad ({td:.0f} cm off)')
        if tr[-1, 3] < 0.4:
            why.append('not resting on the robot')
        air = tr[tr[:, 3] > 1.0]
        dmin = min(float(np.min(clear.surface_distance(air[:, 1], air[:, 2], o))) for o in trunks) \
            if len(air) else float('nan')
        if dmin < TOUCH_TRUNK_M:
            why.append(f'touched a trunk ({dmin:.2f} m)')
        ok &= not why
        lines.append(f'drone {i}: {"PASS" if not why else "FAIL"}  laps {laps}, dock from {src}, '
                     f'touchdown {td:.1f} cm from its pad, closest trunk {dmin:.2f} m'
                     + (f'  <- {"; ".join(why)}' if why else ''))
    # Closest approach between the two, on drone 0's timestamps, while either flies.
    a, b = tracks[0], tracks[1]
    if len(a) and len(b):
        bi = np.column_stack([np.interp(a[:, 0], b[:, 0], b[:, k]) for k in (1, 2, 3)])
        flying = (a[:, 3] > 1.0) | (bi[:, 2] > 1.0)
        flying &= (a[:, 0] >= b[0, 0]) & (a[:, 0] <= b[-1, 0])
        if flying.any():
            d = np.linalg.norm(a[flying, 1:4] - bi[flying], axis=1)
            k = int(np.argmin(d))
            near = d[k] < TOUCH_DRONE_M
            ok &= not near
            lines.append(f'pair: closest {d[k]:.2f} m apart (centres) at '
                         f'{a[flying][k, 0] - a[0, 0]:.0f} s' + ('  <- too close' if near else ''))
    if robot is not None:
        lines.append(f'robot ended at E={robot[0]:.1f} N={robot[1]:.1f}, heading {math.degrees(robot[2]):.0f} deg')
    return ok, lines


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--headless', action='store_true', help='no Gazebo window')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--return-battery', type=float, default=0.80)
    ap.add_argument('--takeoff-delay', type=float, default=10.0,
                    help='s between starting drone 0 and drone 1')
    ap.add_argument('--relocate-after', type=float, default=35.0,
                    help='robot moves this many s after drone 0 arms (both must be airborne)')
    ap.add_argument('--timeout', type=float, default=900.0)
    ap.add_argument('--out', default=os.path.join(REPO, 'recordings',
                                                  time.strftime('multi_%Y%m%d_%H%M%S')))
    a = ap.parse_args()
    a.out = os.path.abspath(a.out)
    # Keep the computer awake (a suspend freezes the simulator), as run_sorties does.
    if not os.environ.get('SORTIE_RUNNER_AWAKE') and shutil.which('systemd-inhibit'):
        os.environ['SORTIE_RUNNER_AWAKE'] = '1'
        os.execvp('systemd-inhibit', ['systemd-inhibit', '--what=sleep:idle:handle-lid-switch',
                                      '--who=multi-drone patrol', '--why=simulation running',
                                      sys.executable, os.path.abspath(__file__), *sys.argv[1:],
                                      f'--out={a.out}'])
    if rs.running_leftovers():
        sys.exit('A simulator or las_offboard node is already running; stop it first:\n  '
                 + '\n  '.join(f'{pid} {cmd}' for pid, cmd in rs.running_leftovers()))
    os.makedirs(a.out, exist_ok=True)
    rs.RUN_LOG = os.path.join(a.out, 'runner.log')
    s = rs.Sortie(a.out)
    print(f'Two-drone patrol in {WORLD}; logs in {a.out}', flush=True)
    try:
        err = fly(s, a)
    except KeyboardInterrupt:
        err = 'stopped by Ctrl-C'
    finally:
        s.stop()
        rs.step('stopped')
    ok, lines = score(a.out)
    if err:
        ok, lines = False, [err] + lines
    report = '\n'.join([f'{"PASS" if ok else "FAIL"}: two-drone patrol'] + lines)
    open(os.path.join(a.out, 'report.txt'), 'w').write(report + '\n')
    print(report)
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
