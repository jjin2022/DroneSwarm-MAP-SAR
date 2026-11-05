#!/usr/bin/env python3
"""Phase 1 exit test: fly sorties back to back, unattended, and count successes in a row.

Each sortie (all decided in docs/running_sim.md / the Phase 1 plan):
  - GPS off; the drone flies on simulated visual odometry (sim_vio, 1% drift)
  - starts on the robot (forest_pillars_robot); the robot relocates mid-sortie to a
    random clear spot, its odometry 3% off; UWB anchors on it
  - about half the sorties get ONE random fault (1/6 each):
      radio_drop    the robot's radio goes silent at a random time
      bad_odometry  the robot's radio report is 2-4 m wrong
      uwb_lost      UWB ranging stops at a random time (exercises the radio /
                    last-known / camera-search fallbacks, which UWB otherwise masks)
  - patrol_lidar patrols, returns, finds the robot and lands on it

A sortie PASSES only if (Q3):
  - mission done without a human: patrolled, returned and landed through precision
    landing (no give-up landing, no handoff, no timeout)
  - the drone came to rest on the pad (centre within the 0.5 m marker), measured
    against the robot's TRUE position
  - it never came closer than 0.38 m (propeller tips) to a trunk, TRUE positions
Any failure resets the count (Q18). The run stops at --target in a row.

Everything runs without windows (Gazebo headless, PX4 without its console, no
QGroundControl: NAV_DLL_ACT=0 in the test runs' parameters). Each
sortie's logs, recording and score go into <run dir>/sortie_NNN/, plus a running
report.csv and summary.md in <run dir>.

    source /opt/ros/jazzy/setup.bash; source ~/dev/px4_ws/install/setup.bash
    source ~/dev/DroneSwarm-MAP-SAR/install/setup.bash
    python3 tools/sortie_runner/run_sorties.py                 # until 50 in a row
    python3 tools/sortie_runner/run_sorties.py --sorties 3     # a short dry run

Refuses to start while another simulator is running. Your normal PX4 parameters
are checked after every sortie and the run stops if they ever change.
"""
import argparse
import csv
import hashlib
import importlib.util
import math
import os
import random
import re
import shutil
import signal
import subprocess
import sys
import time

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
PX4_DIR = os.path.expanduser(os.environ.get('PX4_DIR', '~/dev/PX4-Autopilot'))
WORLD = 'forest_pillars_robot'
WORLD_FILE = os.path.join(REPO, 'src/las_offboard/worlds', WORLD + '.sdf')
SPAWN = '0,0,0.67'
NORMAL_PARAMS = os.path.join(PX4_DIR, 'build/px4_sitl_default/rootfs/parameters.bson')
ROS_ENV = ('source /opt/ros/jazzy/setup.bash; source ~/dev/px4_ws/install/setup.bash; '
           f'source {REPO}/install/setup.bash; ')
RECORD_TOPICS = ['/sim/ground_truth/odometry', '/sim/ground_truth/robot',
                 '/fmu/out/vehicle_local_position_v1', '/robot/dock_position', '/uwb/dock_fix']
# Our programs, to check nothing is left between sorties (patterns can't match this file).
LEFTOVERS = re.compile(r'bin/px4|gz sim|MicroXRCEAgent|parameter_bridge|lib/las_offboard/'
                       r'|ros2 bag record')


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def clean_env():
    """Environment without ROS: Gazebo's own gz must not be shadowed by ROS's."""
    env = dict(os.environ)
    for k in ('PATH', 'LD_LIBRARY_PATH', 'PYTHONPATH'):
        env[k] = ':'.join(p for p in env.get(k, '').split(':') if p and not p.startswith('/opt/ros'))
    env.pop('GZ_CONFIG_PATH', None)
    return env


def running_leftovers():
    me = os.getpid()
    out = subprocess.run(['ps', '-eo', 'pid=,args='], capture_output=True, text=True).stdout
    found = []
    for line in out.splitlines():
        pid, _, args = line.strip().partition(' ')
        if int(pid) != me and LEFTOVERS.search(args) and 'run_sorties' not in args:
            found.append((int(pid), args[:100]))
    return found


def sha(path):
    with open(path, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


class Sortie:
    """One sortie's processes, each in its own process group with its own log."""

    def __init__(self, folder):
        self.folder = folder
        self.procs = []

    def start(self, name, cmd, env=None, ros=True):
        log = open(os.path.join(self.folder, name + '.log'), 'w')
        full = ['bash', '-c', (ROS_ENV if ros else '') + 'exec ' + cmd]
        p = subprocess.Popen(full, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                             env=env, start_new_session=True, cwd=self.folder)
        self.procs.append((name, p))
        return p

    def stop(self):
        # patrol_lidar and the recorder first (clean shutdown), then everything else.
        order = sorted(self.procs, key=lambda np: 0 if np[0] in ('patrol_lidar', 'record') else 1)
        for name, p in order:
            if p.poll() is None:
                try:
                    os.killpg(p.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
            if name in ('patrol_lidar', 'record'):
                try:
                    p.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    pass
        time.sleep(4)
        for _, p in self.procs:
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        time.sleep(1)


RUN_LOG = None


def step(text):
    """Timestamped line in the sortie's runner.log (where did the time go?)."""
    if RUN_LOG:
        with open(RUN_LOG, 'a') as f:
            f.write(time.strftime('%H:%M:%S ') + text + '\n')


def set_px4_param(name, value, url='udpin:0.0.0.0:14540'):
    from pymavlink import mavutil
    link = mavutil.mavlink_connection(url)
    try:
        if link.wait_heartbeat(timeout=15) is None:
            return False
        for _ in range(5):
            link.mav.param_set_send(link.target_system, 1, name.encode(), value,
                                    mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            ack = link.recv_match(type='PARAM_VALUE', blocking=True, timeout=2,
                                  condition=f'PARAM_VALUE.param_id=="{name}"')
            if ack is not None and int(ack.param_value) == value:
                return True
        return False
    finally:
        link.close()


def wait_for(check, timeout, step=2.0):
    end = time.time() + timeout
    while time.time() < end:
        if check():
            return True
        time.sleep(step)
    return False


def gz_world_up():
    try:
        out = subprocess.run(['gz', 'topic', '-l'], capture_output=True, text=True,
                             env=clean_env(), timeout=20).stdout
    except subprocess.TimeoutExpired:
        return False
    return f'/world/{WORLD}/clock' in out


def ros_has(topic_substring):
    try:
        # --no-daemon: the ROS 2 daemon's cached discovery can go stale when a
        # sortie's programs are killed hard, and then never shows the next one's topics.
        out = subprocess.run(['bash', '-c', ROS_ENV + 'ros2 topic list --no-daemon --spin-time 3'],
                             capture_output=True, text=True, timeout=30).stdout
    except subprocess.TimeoutExpired:
        return False
    return topic_substring in out


FAULT_PICK = {'radio_drop': 0.0, 'bad_odometry': 1 / 6, 'uwb_lost': 2 / 6, 'none': 0.9}


def plan(rng, force=None):
    """The random parts of one sortie; force= always picks that fault."""
    p = {'seed': rng.randrange(1, 10 ** 6), 'relocate_after_s': round(rng.uniform(10, 30), 1)}
    r = rng.random()                       # half the sorties get one fault
    if force:
        r = FAULT_PICK[force]
    if r < 1 / 6:
        p['fault'] = 'radio_drop'
        p['radio_silent_after_s'] = round(rng.uniform(5, 45), 1)
    elif r < 2 / 6:
        p['fault'] = 'bad_odometry'
        p['odometry_bias_m'] = round(rng.uniform(2, 4), 1)
    elif r < 3 / 6:
        p['fault'] = 'uwb_lost'
        p['uwb_lost_after_s'] = round(rng.uniform(5, 45), 1)
    else:
        p['fault'] = 'none'
    return p


def fly(folder, p, args):
    s = Sortie(folder)
    seed = p['seed']
    robot_args = (f"-p world:={WORLD} -p world_file:={WORLD_FILE} -p seed:={seed} "
                  f"-p relocate_after_s:={p['relocate_after_s']}")
    if p['fault'] == 'radio_drop':
        robot_args += f" -p radio_silent_after_s:={p['radio_silent_after_s']}"
    if p['fault'] == 'bad_odometry':
        robot_args += f" -p odometry_bias_m:={p['odometry_bias_m']}"
    global RUN_LOG
    RUN_LOG = os.path.join(folder, 'runner.log')
    step('start')
    try:
        s.start('agent', 'MicroXRCEAgent udp4 -p 8888', ros=False)
        s.start('gazebo', f'python3 {PX4_DIR}/Tools/simulation/gz/simulation-gazebo --world {WORLD} '
                f'--model_store {PX4_DIR}/Tools/simulation/gz --headless', env=clean_env(), ros=False)
        if not wait_for(gz_world_up, 120):
            return 'gazebo did not start'
        step('gazebo up')
        env = dict(os.environ, PX4_DAEMON='1')
        s.start('px4', f'{REPO}/scripts/px4_nogps.sh {WORLD} {SPAWN}', env=env, ros=False)
        if not wait_for(lambda: ros_has('fmu/out/vehicle_status'), 120, 3):
            return 'PX4 did not start'
        step('PX4 up')
        # No QGroundControl in an unattended run, but the x500 airframe turns on the
        # GCS-loss action, so "no ground station" blocks arming. Turn it off after
        # boot: a PX4_PARAM_ value of 0 at startup is taken as "default" and then
        # overridden by the airframe's set-default. Lands in the PX4 script's own
        # parameter folder, never your normal one.
        if not set_px4_param('NAV_DLL_ACT', 0):
            return 'could not set NAV_DLL_ACT'
        step('NAV_DLL_ACT=0 set')
        s.start('bridge', f'ros2 launch las_offboard lidar_bridge.launch.py world:={WORLD} '
                'model:=x500_lidar_cam_down_0 camera:=true vio:=true')
        s.start('sim_vio', f'ros2 run las_offboard sim_vio --ros-args -p seed:={seed}')
        s.start('pad_detector', 'ros2 run las_offboard pad_detector')
        s.start('robot_sim', f'ros2 run las_offboard robot_sim --ros-args {robot_args}')
        uwb_args = f"-p seed:={seed}"
        if p['fault'] == 'uwb_lost':
            uwb_args += f" -p lost_after_s:={p['uwb_lost_after_s']}"
        s.start('uwb_sim', f'ros2 run las_offboard uwb_sim --ros-args {uwb_args}')
        s.start('uwb_locator', 'ros2 run las_offboard uwb_locator')
        if not wait_for(lambda: ros_has('/scan') and ros_has('/camera/image_raw'), 60, 3):
            return 'bridge did not start'
        step('bridge up')
        qos = os.path.join(REPO, 'tools/avoid_eval/qos.yaml')
        s.start('record', f'ros2 bag record -o {folder}/bag ' + ' '.join(RECORD_TOPICS)
                + f' --qos-profile-overrides-path {qos}')
        time.sleep(3)
        s.start('patrol_lidar', 'ros2 run las_offboard patrol_lidar --ros-args -p altitude_m:=2.0 '
                f'-p "map_dir:=\'\'" -p return_battery:={args.return_battery}')
        step('patrol_lidar started')
        log = os.path.join(folder, 'patrol_lidar.log')
        done = wait_for(lambda: re.search(r'-> DONE|-> HANDOFF', open(log).read()) is not None,
                        args.timeout, 3)
        step('flight over' if done else 'flight timed out')
        return None if done else f'timed out after {args.timeout:.0f} s'
    finally:
        s.stop()
        step('stopped')


def score(folder, launch_error, land, clear):
    """(passed, reasons, numbers) for one sortie."""
    reasons, nums = [], {}
    if launch_error:
        return False, [launch_error], nums
    log = open(os.path.join(folder, 'patrol_lidar.log')).read()
    for needed, why in (('TAKEOFF -> PATROL', 'never patrolled'),
                        ('PATROL -> RETURN', 'never returned from patrol'),
                        ('PRECISION_LAND -> LAND', 'no precision landing'),
                        ('LAND -> DONE', 'did not finish (land and disarm)')):
        if needed not in log:
            reasons.append(why)
    if 'GO_HOME' in log:
        reasons.append('gave up finding the robot (GO_HOME)')
    if 'HANDOFF' in log:
        reasons.append('handed control back (left offboard)')
    nums['laps'] = len(re.findall(r'Lap \d+ done', log))
    nums['dock_source'] = (re.findall(r'from (UWB|robot radio|last known[^)]*\)|setting)', log)
                           or ['?'])[-1]
    bag = os.path.join(folder, 'bag')
    try:
        pos, _, is_truth, robot = land.read(bag)
        if not is_truth or robot is None:
            reasons.append('no ground truth recorded')
        else:
            end = pos[-1]
            de, dn = end[0] - robot[0], end[1] - robot[1]
            nums['touchdown_cm'] = round(100 * math.hypot(de, dn), 1)
            if not (abs(de) <= land.PAD_HALF and abs(dn) <= land.PAD_HALF):
                reasons.append(f'not on the pad ({nums["touchdown_cm"]} cm off)')
            if end[2] > -0.4:                                   # down: on the platform ~ -0.65
                reasons.append('not resting on the robot')
        tr, _ = clear.read_positions(bag)
        tr = tr[-tr[:, 3] > 1.0]
        obs = clear.obstacles(WORLD_FILE)
        import numpy as np
        dmin = min(float(np.min(clear.surface_distance(tr[:, 1], tr[:, 2], o)))
                   for o in obs if o[0].startswith('trunk'))
        nums['closest_m'] = round(dmin, 2)
        if dmin < 0.38:
            reasons.append(f'touched a trunk ({dmin:.2f} m)')
    except (SystemExit, Exception) as exc:            # a broken recording is a failure
        reasons.append(f'could not score: {exc}')
    rob = open(os.path.join(folder, 'robot_sim.log')).read()
    m = re.search(r'Robot moved ([\d.]+) m', rob)
    nums['robot_moved_m'] = float(m.group(1)) if m else 0.0
    return not reasons, reasons, nums


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--target', type=int, default=50, help='successes in a row to stop at')
    ap.add_argument('--sorties', type=int, default=200, help='give up after this many')
    ap.add_argument('--seed', type=int, default=1, help='seed for the sortie plans')
    ap.add_argument('--return-battery', type=float, default=0.80)
    ap.add_argument('--fault', choices=sorted(FAULT_PICK),
                    help='give every sortie this fault (for checking one fallback)')
    ap.add_argument('--timeout', type=float, default=600.0, help='s per sortie')
    ap.add_argument('--out', default=os.path.join(REPO, 'recordings',
                                                  time.strftime('sorties_%Y%m%d_%H%M%S')))
    a = ap.parse_args()
    a.out = os.path.abspath(a.out)       # programs run inside each sortie folder

    # Keep the computer awake: a laptop suspending mid-sortie freezes the simulator
    # and fails the sortie (seen: suspend at 00:15, resumed 14 h later -> timeout).
    if not os.environ.get('SORTIE_RUNNER_AWAKE') and shutil.which('systemd-inhibit'):
        os.environ['SORTIE_RUNNER_AWAKE'] = '1'
        os.execvp('systemd-inhibit', ['systemd-inhibit', '--what=sleep:idle:handle-lid-switch',
                                      '--who=sortie runner', '--why=simulated sorties running',
                                      sys.executable, os.path.abspath(__file__), *sys.argv[1:],
                                      f'--out={a.out}'])   # last --out wins

    if running_leftovers():
        sys.exit('A simulator or las_offboard node is already running; stop it first:\n  '
                 + '\n  '.join(f'{pid} {cmd}' for pid, cmd in running_leftovers()))
    land = load(os.path.join(REPO, 'tools/land_eval/landing_error.py'), 'land_eval')
    clear = load(os.path.join(REPO, 'tools/avoid_eval/clearance.py'), 'avoid_eval')
    normal_sha = sha(NORMAL_PARAMS) if os.path.exists(NORMAL_PARAMS) else None
    os.makedirs(a.out, exist_ok=True)
    rng = random.Random(a.seed)
    fields = ['sortie', 'result', 'in_a_row', 'fault', 'seed', 'relocate_after_s', 'detail',
              'robot_moved_m', 'dock_source', 'touchdown_cm', 'closest_m', 'laps', 'reasons', 'minutes']
    report = os.path.join(a.out, 'report.csv')
    with open(report, 'w', newline='') as f:
        csv.writer(f).writerow(fields)
    print(f'Sortie runner: target {a.target} in a row, max {a.sorties} sorties -> {a.out}', flush=True)

    streak, best, passed = 0, 0, 0
    for i in range(1, a.sorties + 1):
        p = plan(rng, a.fault)
        folder = os.path.join(a.out, f'sortie_{i:03d}')
        os.makedirs(folder)
        t0 = time.time()
        err = fly(folder, p, a)
        left = running_leftovers()
        for pid, _ in left:                      # anything of ours that survived
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        step('scoring')
        ok, reasons, nums = score(folder, err, land, clear)
        step('scored')
        if left:
            reasons.append(f'{len(left)} process(es) left running after the sortie (killed)')
        streak = streak + 1 if ok else 0
        best, passed = max(best, streak), passed + ok
        detail = {'radio_drop': f"radio silent at {p.get('radio_silent_after_s')} s",
                  'bad_odometry': f"report {p.get('odometry_bias_m')} m off",
                  'uwb_lost': f"UWB lost at {p.get('uwb_lost_after_s')} s"}.get(p['fault'], '')
        row = [i, 'PASS' if ok else 'FAIL', streak, p['fault'], p['seed'], p['relocate_after_s'],
               detail, nums.get('robot_moved_m'), nums.get('dock_source'), nums.get('touchdown_cm'),
               nums.get('closest_m'), nums.get('laps'), '; '.join(reasons),
               round((time.time() - t0) / 60, 1)]
        with open(report, 'a', newline='') as f:
            csv.writer(f).writerow(row)
        print(f"sortie {i:3d}: {'PASS' if ok else 'FAIL'} ({p['fault']}{', ' + detail if detail else ''}) "
              f"touchdown {nums.get('touchdown_cm')} cm, closest {nums.get('closest_m')} m, "
              f"in a row {streak}{'  <- ' + '; '.join(reasons) if reasons else ''}", flush=True)
        with open(os.path.join(a.out, 'summary.md'), 'w') as f:
            f.write(f'# Sortie run\n\n{i} sorties, {passed} passed; {streak} in a row now, '
                    f'best {best}; target {a.target}.\n\nSee report.csv for each sortie.\n')
        if normal_sha and sha(NORMAL_PARAMS) != normal_sha:
            sys.exit('STOP: your normal PX4 parameters changed during the run.')
        if streak >= a.target:
            print(f'Target reached: {streak} successful sorties in a row.', flush=True)
            return
    print(f'Stopped after {a.sorties} sorties: best run {best} in a row.', flush=True)


if __name__ == '__main__':
    main()
