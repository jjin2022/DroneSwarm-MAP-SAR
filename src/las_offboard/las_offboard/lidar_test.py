"""
lidar_test.py - Read and print the 2D LIDAR on the simulated x500 drone.

WHAT THIS IS FOR
----------------
`make px4_sitl gz_x500_lidar_2d` starts PX4 + Gazebo with an x500 quadcopter
that has a spinning 2D laser scanner ("lidar_2d_v2") bolted on top. This script
listens to that laser scanner and prints what it sees, so you can learn how the
data looks before using it for mapping / obstacle avoidance.

HOW THE DATA FLOWS (important to understand)
--------------------------------------------
    Gazebo lidar sensor
        |  (Gazebo Transport topic, message type gz.msgs.LaserScan)
        +------------------------> THIS SCRIPT (reads it directly)
        |
        +--> PX4 gz_bridge --> uORB "obstacle_distance" (used internally by
                                PX4 Collision Prevention; NOT sent out to ROS 2)

So the raw scan is NOT a ROS 2 topic. Gazebo has its own message system
("Gazebo Transport"), and this script subscribes to it using Gazebo's Python
bindings (packages python3-gz-transport13 / python3-gz-msgs10, already
installed with Gazebo Harmonic). No MicroXRCEAgent or ROS bridge is needed.

HOW TO RUN
----------
Terminal 1 (PX4 + Gazebo with the lidar drone):
    cd ~/dev/PX4-Autopilot
    make px4_sitl gz_x500_lidar_2d

Terminal 2 (this script), once Gazebo is up:
    python3 ~/dev/DroneSwarm-MAP-SAR/src/las_offboard/las_offboard/lidar_test.py
  or, after `colcon build` and sourcing install/setup.bash:
    ros2 run las_offboard lidar_test

Then drop objects near the drone in the Gazebo GUI (the box / cylinder shapes
in the top toolbar) and watch the distances change. Ctrl+C to stop.

ABOUT THIS SENSOR (from PX4-Autopilot/Tools/simulation/gz/models/lidar_2d_v2)
---------------------------------------------------------------------------
- 1080 samples per scan
- Field of view: -135 deg to +135 deg (270 deg total; 90 deg blind spot behind)
- Range: 0.1 m to 30 m
- 30 scans per second
- Angles: 0 deg = straight ahead (drone's nose), positive = to the LEFT
  (counter-clockwise when viewed from above; Gazebo uses the FLU frame:
  x Forward, y Left, z Up).
"""

import math       # for inf / pi / degree conversion
import sys        # to exit with an error code
import threading  # the callback runs on a Gazebo thread; a lock keeps data safe
import time       # for sleeping in the main loop

# Gazebo Transport: the pub/sub system Gazebo uses (like ROS 2 topics, but Gazebo's own).
from gz.transport13 import Node
# The LaserScan message definition (a protobuf class) from Gazebo's msgs library.
from gz.msgs10.laserscan_pb2 import LaserScan


# ---------------------------------------------------------------------------
# Settings you can tweak
# ---------------------------------------------------------------------------

# Every Gazebo lidar topic ends with this. The full name looks like:
#   /world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan
# "default" is the world name and "x500_lidar_2d_0" is the drone (instance 0).
# We search for the suffix so the script still works with other worlds/instances.
TOPIC_SUFFIX = '/sensor/lidar_2d_v2/scan'

PRINT_PERIOD_S = 1.0       # how often to print a summary (the sensor itself runs at 30 Hz)
TOPIC_WAIT_TIMEOUT_S = 30  # how long to wait for Gazebo to start before giving up

# Named directions we summarise. Each is (label, centre angle in degrees, half width).
# Remember: + angle = left, - angle = right, 0 = front.
SECTORS = [
    ('FRONT', 0.0, 22.5),
    ('LEFT', 90.0, 22.5),
    ('RIGHT', -90.0, 22.5),
    ('BACK-LEFT', 135.0, 22.5),    # only half of these is visible (FOV ends at +/-135)
    ('BACK-RIGHT', -135.0, 22.5),
]


# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------

def find_lidar_topic(node):
    """Ask Gazebo for every topic it knows and return the lidar scan topic.

    Gazebo takes a few seconds to start, so keep retrying until the timeout.
    """
    deadline = time.time() + TOPIC_WAIT_TIMEOUT_S
    while time.time() < deadline:
        # topic_list() returns all topic names currently advertised on the network.
        for topic in node.topic_list():
            if topic.endswith(TOPIC_SUFFIX):
                return topic
        print('Waiting for the lidar topic (is `make px4_sitl gz_x500_lidar_2d` running?)...')
        time.sleep(1.0)
    return None


def angle_of(msg, i):
    """Angle (radians) of sample number i.

    The scan is a flat list of distances. Sample 0 is at angle_min, and each
    next sample is angle_step further counter-clockwise (to the left).
    """
    return msg.angle_min + i * msg.angle_step


def is_hit(msg, r):
    """True if the distance r is a real detection.

    When the laser hits nothing within range, Gazebo reports +inf (or a value
    at/above range_max). Values below range_min are too close to measure.
    """
    return math.isfinite(r) and msg.range_min <= r < msg.range_max


def sector_min(msg, centre_deg, half_width_deg):
    """Closest hit (metres) inside one angular slice, or None if it is clear."""
    closest = None
    for i, r in enumerate(msg.ranges):
        # Convert this sample's angle to degrees and measure how far it is from
        # the centre of the slice. The ((x + 180) % 360) - 180 trick wraps the
        # difference into -180..180 so e.g. 179 deg and -179 deg count as close.
        a_deg = math.degrees(angle_of(msg, i))
        diff = ((a_deg - centre_deg + 180.0) % 360.0) - 180.0
        if abs(diff) <= half_width_deg and is_hit(msg, r):
            if closest is None or r < closest:
                closest = r
    return closest


def ascii_strip(msg, width=54):
    """Draw the scan as one line of characters, left side of the drone first.

    The 1080 samples are grouped into `width` bins (each ~5 deg). Each bin
    shows how close the nearest object is:
        '#' < 1 m   '=' < 3 m   '-' < 10 m   '.' < range_max   ' ' nothing seen
    Because angles increase to the left, we draw from the last sample (left,
    +135 deg) to the first (right, -135 deg) so it reads like looking ahead.
    """
    n = len(msg.ranges)
    per_bin = max(1, n // width)
    chars = []
    for b in range(width):
        chunk = msg.ranges[b * per_bin:(b + 1) * per_bin]
        hits = [r for r in chunk if is_hit(msg, r)]
        if not hits:
            chars.append(' ')
            continue
        d = min(hits)
        chars.append('#' if d < 1 else '=' if d < 3 else '-' if d < 10 else '.')
    return ''.join(reversed(chars))  # reverse so LEFT is on the left of the screen


def fmt(d):
    """Pretty-print a distance, or 'clear' when nothing was detected."""
    return 'clear' if d is None else f'{d:5.2f} m'


# ---------------------------------------------------------------------------
# Main program
# ---------------------------------------------------------------------------

def main():
    # A Node is our connection to Gazebo Transport (like an rclpy Node for ROS 2).
    node = Node()

    topic = find_lidar_topic(node)
    if topic is None:
        print(f'No topic ending in {TOPIC_SUFFIX} found. '
              'Start the sim with: make px4_sitl gz_x500_lidar_2d')
        sys.exit(1)
    print(f'Found lidar topic:\n  {topic}\n')

    # Shared state between the Gazebo callback thread and our main loop.
    lock = threading.Lock()
    latest = {'msg': None, 'count': 0}

    def on_scan(msg):
        """Called by Gazebo ~30 times per second with a new LaserScan.

        Keep this function fast: just store the message. The heavier
        processing and printing happens in the main loop below.
        """
        with lock:
            latest['msg'] = msg
            latest['count'] += 1

    # Subscribe: "whenever a LaserScan arrives on `topic`, call on_scan".
    # Note the argument order for the Python bindings: (type, topic, callback).
    if not node.subscribe(LaserScan, topic, on_scan):
        print(f'Failed to subscribe to {topic}')
        sys.exit(1)

    first = True
    last_count = 0
    try:
        while True:
            time.sleep(PRINT_PERIOD_S)

            # Grab the newest scan (and how many arrived) safely.
            with lock:
                msg = latest['msg']
                count = latest['count']
            if msg is None:
                print('Subscribed, but no scans yet... (is the simulation paused?)')
                continue

            # On the first scan, print the sensor's own description of itself.
            if first:
                first = False
                print('Sensor info (from the message itself):')
                print(f'  samples per scan : {len(msg.ranges)}')
                print(f'  angle range      : {math.degrees(msg.angle_min):.1f} to '
                      f'{math.degrees(msg.angle_max):.1f} deg')
                print(f'  angle step       : {math.degrees(msg.angle_step):.3f} deg')
                print(f'  distance range   : {msg.range_min:.2f} to {msg.range_max:.1f} m')
                print(f'  frame            : {msg.frame}\n')

            # Scans per second actually received since the last print.
            rate_hz = (count - last_count) / PRINT_PERIOD_S
            last_count = count

            # Find the single closest object anywhere in the scan, and its angle.
            closest_r, closest_i = None, None
            hits = 0
            for i, r in enumerate(msg.ranges):
                if is_hit(msg, r):
                    hits += 1
                    if closest_r is None or r < closest_r:
                        closest_r, closest_i = r, i

            # Print a summary block.
            print(f'--- scan #{count}  ({rate_hz:.0f} Hz)  {hits}/{len(msg.ranges)} beams hit something')
            if closest_r is None:
                print('  Closest: nothing within range')
            else:
                a = math.degrees(angle_of(msg, closest_i))
                side = 'left' if a > 0 else 'right' if a < 0 else 'ahead'
                print(f'  Closest: {closest_r:.2f} m at {a:+.1f} deg ({side})')
            for label, centre, half in SECTORS:
                print(f'  {label:<10} {fmt(sector_min(msg, centre, half))}')
            print(f'  L [{ascii_strip(msg)}] R')
            print('     (# <1m  = <3m  - <10m  . <30m  blank = nothing)\n')

    except KeyboardInterrupt:
        # Ctrl+C: exit quietly.
        print('\nStopped.')


if __name__ == '__main__':
    main()
