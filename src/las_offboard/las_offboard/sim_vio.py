"""Simulated visual odometry: lets PX4 fly without GPS in the simulator.

A real drone without GPS gets its position from visual-inertial odometry (VIO),
which drifts: the error grows with distance flown. This node stands in for it.
It reads the drone's TRUE position from Gazebo, adds that kind of drift, and
sends the result to PX4 as external vision, so PX4's estimator, and everything
built on it (patrol_lidar, pad_detector), sees a realistically wandering
position instead of perfect simulated GPS.

  In:   /sim/ground_truth/odometry   nav_msgs/Odometry from Gazebo (ENU, body FLU),
                                     bridged by lidar_bridge.launch.py vio:=true
  Out:  fmu/in/vehicle_visual_odometry   px4_msgs/VehicleOdometry (NED / FRD)
        /sim_vio/drift   geometry_msgs/PointStamped: the current error (m, NED)
                         added to the true position, for tests to log

Drift model: every metre flown adds drift_percent/100 m of error in a direction
that wanders slowly (a VIO's scale and heading errors), plus a little noise on
each reading. The velocity sent carries the matching drift rate, as a real VIO's
does; a perfect velocity would let PX4 dead-reckon the truth and hide the drift.
Yaw is left true: PX4 takes yaw from its magnetometer here.

PX4 must be told to use it and ignore GPS (EKF2_GPS_CTRL=0, EKF2_EV_CTRL=5,
EKF2_HGT_REF=0); see docs/running_sim.md, "Flying without GPS".

Only this node may read the ground truth: everything else must work from PX4's
estimate, as on a real drone.

If anything else also publishes fmu/in/vehicle_visual_odometry (e.g. a sim_vio left
running from an earlier test), this node stops sending until it's gone: PX4 would
otherwise blend two different drifts and quietly hide the drift being tested.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleOdometry

# ENU (x east, y north, z up) -> NED (x north, y east, z down)
ENU_TO_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
# Body FLU (forward, left, up) -> FRD (forward, right, down)
FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])


def quat_to_rot(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def rot_to_quat(m):
    """(w, x, y, z) from a rotation matrix."""
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((0.25 * s), (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s)
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        return ((m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s)
    if m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        return ((m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s)
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
    return ((m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s)


class SimVio(Node):
    def __init__(self):
        super().__init__('sim_vio')
        p = self.declare_parameter
        self.drift = p('drift_percent', 1.0).value / 100.0     # error per metre flown
        self.wander = p('drift_wander_deg_per_m', 10.0).value   # how fast its direction turns
        self.noise = p('noise_m', 0.01).value                   # per-reading position noise
        self.rate = p('rate_hz', 30.0).value
        self.rng = np.random.default_rng(p('seed', 0).value or None)

        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Odometry, '/sim/ground_truth/odometry', self.on_truth, 10)
        self.pub = self.create_publisher(VehicleOdometry, 'fmu/in/vehicle_visual_odometry', px4_qos)
        self.drift_pub = self.create_publisher(PointStamped, '/sim_vio/drift', 10)

        self.origin = None                  # true position at start (NED): VIO starts at 0
        self.last = None                    # last true position, for distance flown
        self.err = np.zeros(3)              # accumulated drift, NED
        angle = self.rng.uniform(0, 2 * math.pi)
        self.dir = np.array([math.cos(angle), math.sin(angle), 0.0])   # drift direction
        self.flown = 0.0
        self.truth = None
        self.blocked = False                # another publisher on the PX4 input
        self.create_timer(1.0, self.check_publishers)
        self.create_timer(1.0 / self.rate, self.publish)
        self.create_timer(10.0, self.report)
        self.get_logger().info(f'Simulated VIO: {self.drift * 100:.1f}% drift per metre flown, '
                               f'{self.noise * 100:.0f} cm noise, {self.rate:.0f} Hz')

    def on_truth(self, msg):
        self.truth = msg

    def report(self):
        if self.truth is not None:
            self.get_logger().info(f'flown {self.flown:.0f} m, drift now '
                                   f'{np.linalg.norm(self.err[:2]):.2f} m horizontal')

    def check_publishers(self):
        others = self.count_publishers(self.pub.topic_name) - 1
        if others > 0 and not self.blocked:
            self.get_logger().error(
                f'{others} other publisher(s) on {self.pub.topic_name} (another sim_vio still '
                'running?). Not sending until they stop.')
        elif others == 0 and self.blocked:
            self.get_logger().info('Only publisher again: sending odometry.')
        self.blocked = others > 0

    def publish(self):
        m = self.truth
        if m is None or self.blocked:
            return
        p_enu = np.array([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])
        p_ned = ENU_TO_NED @ p_enu
        if self.origin is None:
            self.origin = p_ned.copy()
        if self.last is not None:
            step = float(np.linalg.norm(p_ned - self.last))
            if step < 1.0:                                  # ignore jumps (resets)
                self.flown += step
                # The drift direction wanders a little with distance, then error grows.
                turn = math.radians(self.rng.normal(0.0, self.wander) * math.sqrt(step))
                c, s = math.cos(turn), math.sin(turn)
                self.dir = np.array([c * self.dir[0] - s * self.dir[1],
                                     s * self.dir[0] + c * self.dir[1], 0.0])
                self.err += self.drift * step * self.dir
                self.err[2] += self.drift * 0.3 * step * self.rng.normal()   # a little vertical
        self.last = p_ned

        q = m.pose.pose.orientation
        r_enu_flu = quat_to_rot(q.w, q.x, q.y, q.z)
        r_ned_frd = ENU_TO_NED @ r_enu_flu @ FLU_TO_FRD
        v = m.twist.twist.linear                            # body FLU
        v_ned = ENU_TO_NED @ (r_enu_flu @ np.array([v.x, v.y, v.z]))
        # Velocity must drift consistently with position (the error grows at
        # drift * speed along self.dir). A perfect velocity would let PX4 dead-reckon
        # the truth and quietly cancel most of the drift.
        v_ned = v_ned + self.drift * float(np.linalg.norm(v_ned)) * self.dir

        out = VehicleOdometry()
        now = int(self.get_clock().now().nanoseconds / 1000)
        out.timestamp = out.timestamp_sample = now
        out.pose_frame = VehicleOdometry.POSE_FRAME_NED
        pos = p_ned - self.origin + self.err + self.rng.normal(0.0, self.noise, 3)
        out.position = [float(x) for x in pos]
        out.q = [float(x) for x in rot_to_quat(r_ned_frd)]
        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        out.velocity = [float(x) for x in v_ned + self.rng.normal(0.0, 0.02, 3)]
        out.angular_velocity = [math.nan] * 3
        out.position_variance = [self.noise ** 2 + 1e-4] * 3
        out.orientation_variance = [1e-3] * 3
        out.velocity_variance = [4e-4] * 3
        self.pub.publish(out)

        d = PointStamped()
        d.header.stamp = self.get_clock().now().to_msg()
        d.header.frame_id = 'px4_local_ned'
        d.point.x, d.point.y, d.point.z = (float(x) for x in self.err)
        self.drift_pub.publish(d)


def main(args=None):
    rclpy.init(args=args)
    node = SimVio()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
