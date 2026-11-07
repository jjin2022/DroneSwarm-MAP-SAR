"""Stand-in for the robot (mothership) side: relocate mid-sortie and radio where it went.

The real robot will walk somewhere while the drone is out and tell the drone,
over its radio link, how far it moved according to its own (drifting) odometry.
This node does the simulator version:

  - Once the drone has been armed for relocate_after_s, it teleports the
    robot_dock model to a random clear spot in the forest (>= clearance_m from
    every trunk, like random_pad.py) and leaves it standing there.
  - Once a second it "radios" the dock position relative to the drone's take-off
    point, with odometry error: odometry_error_percent of the distance moved, in a
    random direction (legged odometry: ~3%).
  - Optional radio-drop fault: radio_silent_after_s stops the reports from that
    many seconds after arming (the drone must fall back to the last known position).
  - Optional bad-odometry fault: odometry_bias_m adds a fixed error of that size to
    the report once the robot has moved (the drone's UWB fix should catch it).
  - The robot faces a random direction after moving (the UWB anchors turn with it).
  - Several pads (multi-drone, robot_dock_duo): pads:=[x0, y0, x1, y1, ...] gives
    each pad's offset in the robot's body frame (x forward, y left, m). Pad i is
    reported on its own topic, relative to where ITS drone took off (that pad's
    start position), turned with the robot's heading. Default: one pad at the
    centre, i.e. the single-drone behaviour.

  Out:  /robot/dock_position      geometry_msgs/PointStamped, frame "takeoff_ned":
                                  x north, y east (m) of the dock from where the drone
                                  took off. What the drone is allowed to use.
                                  Pad i > 0: /robot/dock_position_<i>, same frame
                                  convention (from pad i's start).
        /sim/ground_truth/robot   geometry_msgs/PoseStamped, frame "world" (ENU):
                                  the robot's true pose, for uwb_sim and test tools
                                  only (never for the drone).
  In:   fmu/out/vehicle_status_v1 (to know when the drone armed)

Assumes the drone took off from the robot (spawned on its back), so the take-off
point is the robot's start position. Needs the world file for the trunk positions.
"""
import math
import os
import random
import re
import subprocess

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleStatus


def clean_gz_env():
    """Environment for Gazebo's own `gz` CLI: ROS's gz (no services/sim) must not shadow it."""
    env = dict(os.environ)
    env['PATH'] = ':'.join(p for p in env.get('PATH', '').split(':') if not p.startswith('/opt/ros'))
    env['LD_LIBRARY_PATH'] = ':'.join(
        p for p in env.get('LD_LIBRARY_PATH', '').split(':') if not p.startswith('/opt/ros'))
    env.pop('GZ_CONFIG_PATH', None)
    return env


class RobotSim(Node):
    def __init__(self):
        super().__init__('robot_sim')
        p = self.declare_parameter
        self.world_file = p('world_file', '').value
        self.world = p('world', 'forest_pillars_robot').value
        self.model = p('model', 'robot_dock').value
        self.start = (p('start_e', 0.0).value, p('start_n', 0.0).value)   # world ENU
        self.relocate_after = p('relocate_after_s', 20.0).value          # < 0: never
        self.radio_silent_after = p('radio_silent_after_s', -1.0).value  # < 0: never
        self.odo_err = p('odometry_error_percent', 3.0).value / 100.0
        self.clearance = p('clearance_m', 1.0).value
        self.size = p('forest_size_m', 15.0).value
        self.min_move = p('min_move_m', 2.0).value
        self.bias = p('odometry_bias_m', 0.0).value
        seed = p('seed', 0).value
        pads = list(p('pads', [0.0, 0.0]).value)
        if len(pads) < 2 or len(pads) % 2:
            raise SystemExit('pads must be [x0, y0, x1, y1, ...] (body frame, m)')
        self.pads = [(pads[i], pads[i + 1]) for i in range(0, len(pads), 2)]
        self.rng = random.Random(seed or None)

        if not self.world_file:
            raise SystemExit('Set world_file:=<path to the world .sdf> (for the trunk positions)')
        text = open(os.path.expanduser(self.world_file)).read()
        self.trunks = [(float(x), float(y), float(r)) for x, y, r in re.findall(
            r'<model name="trunk_\d+">.*?<pose>(\S+) (\S+) .*?<radius>(\S+)</radius>', text, re.S)]

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(VehicleStatus, 'fmu/out/vehicle_status_v1', self.on_status, qos)
        self.report_pubs = [self.create_publisher(
            PointStamped, '/robot/dock_position' + (f'_{i}' if i else ''), 10)
            for i in range(len(self.pads))]
        self.truth_pub = self.create_publisher(PoseStamped, '/sim/ground_truth/robot', 10)

        self.pos = self.start          # true position, world ENU
        self.yaw = 0.0                 # true heading, ENU rad (starts facing east)
        self.err = (0.0, 0.0)          # odometry error (E, N)
        self.armed_at = None
        self.moved = False
        self.create_timer(0.2, self.publish_truth)    # uwb_sim needs it often
        self.create_timer(1.0, self.tick)
        self.get_logger().info(
            f'Robot at E={self.start[0]:.1f} N={self.start[1]:.1f}; relocates '
            f'{self.relocate_after:.0f} s after arming, odometry error '
            f'{self.odo_err * 100:.0f}% of distance moved'
            + (f', radio silent after {self.radio_silent_after:.0f} s'
               if self.radio_silent_after >= 0 else ''))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_status(self, msg):
        if self.armed_at is None and msg.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.armed_at = self.now()
            self.get_logger().info('Drone armed: relocation clock started')

    def pick_spot(self):
        half = self.size / 2.0 - 0.5
        for _ in range(100000):
            e, n = self.rng.uniform(-half, half), self.rng.uniform(-half, half)
            if math.hypot(e - self.pos[0], n - self.pos[1]) < self.min_move:
                continue
            if min(math.hypot(e - x, n - y) - r for x, y, r in self.trunks) >= self.clearance:
                return e, n
        raise RuntimeError('no clear spot for the robot')

    def teleport(self, e, n, yaw):
        req = (f'name: "{self.model}", position: {{x: {e:.3f}, y: {n:.3f}, z: 0.0}}, '
               f'orientation: {{z: {math.sin(yaw / 2):.6f}, w: {math.cos(yaw / 2):.6f}}}')
        out = subprocess.run(
            ['gz', 'service', '-s', f'/world/{self.world}/set_pose',
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '3000', '--req', req],
            env=clean_gz_env(), capture_output=True, text=True, timeout=10)
        return 'data: true' in out.stdout

    def publish_truth(self):
        truth = PoseStamped()
        truth.header.stamp, truth.header.frame_id = self.get_clock().now().to_msg(), 'world'
        truth.pose.position.x, truth.pose.position.y = float(self.pos[0]), float(self.pos[1])
        truth.pose.orientation.z = math.sin(self.yaw / 2)
        truth.pose.orientation.w = math.cos(self.yaw / 2)
        self.truth_pub.publish(truth)

    def tick(self):
        t = None if self.armed_at is None else self.now() - self.armed_at
        if t is not None and not self.moved and 0 <= self.relocate_after <= t:
            e, n = self.pick_spot()
            yaw = self.rng.uniform(-math.pi, math.pi)
            if self.teleport(e, n, yaw):
                dist = math.hypot(e - self.pos[0], n - self.pos[1])
                ang = self.rng.uniform(0.0, 2 * math.pi)
                err = self.odo_err * dist + self.bias
                self.err = (self.err[0] + err * math.cos(ang), self.err[1] + err * math.sin(ang))
                self.pos, self.yaw = (e, n), yaw
                self.moved = True
                self.get_logger().info(
                    f'Robot moved {dist:.1f} m to E={e:.2f} N={n:.2f}, heading '
                    f'{math.degrees(yaw):.0f} deg (radio report now {math.hypot(*self.err):.2f} m off'
                    f'{", bad-odometry fault" if self.bias else ""})')
            else:
                self.get_logger().error('set_pose failed; will retry', throttle_duration_sec=5.0)

        stamp = self.get_clock().now().to_msg()

        if t is not None and 0 <= self.radio_silent_after <= t:
            self.get_logger().warn('Radio silent (fault injected)', throttle_duration_sec=10.0)
            return
        # What the robot believes: its odometry since take-off, error included (its
        # heading is taken as known: a legged robot's IMU yaw drifts far less).
        for (px, py), pub in zip(self.pads, self.report_pubs):
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            # Pad i now, minus pad i at the start (robot at start, heading 0).
            de = self.pos[0] + c * px - s * py - (self.start[0] + px) + self.err[0]
            dn = self.pos[1] + s * px + c * py - (self.start[1] + py) + self.err[1]
            rep = PointStamped()
            rep.header.stamp, rep.header.frame_id = stamp, 'takeoff_ned'
            rep.point.x, rep.point.y = float(dn), float(de)       # NED: x north, y east
            pub.publish(rep)


def main(args=None):
    rclpy.init(args=args)
    node = RobotSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
