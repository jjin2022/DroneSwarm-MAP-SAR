"""Simulated UWB ranging between the drone and the robot's 4 anchors.

Simulator side only: it uses the TRUE drone pose (from Gazebo) and the TRUE robot
pose (from robot_sim) to produce what a UWB radio on the drone would measure.
The drone itself only ever sees the ranges (uwb_locator turns them into a fix).

  In:   /sim/ground_truth/odometry   drone true pose (nav_msgs/Odometry, world ENU)
        /sim/ground_truth/robot      robot true pose (geometry_msgs/PoseStamped)
  Out:  /uwb/ranges   std_msgs/Float32MultiArray, 4 ranges (m) to the anchors in
                      las_offboard/uwb_anchors.py order; NaN = no reading

Noise model (as in tools/swarm_uwb_sim.py / docs/drone-design.md):
  - Gaussian noise noise_m (default 0.10 m, DW3000 class)
  - non-line-of-sight: with probability nlos_prob a reading is 0.3-2.5 m too long
    (signal bounced round a trunk); UWB errors are almost always positive
  - nothing beyond max_range_m

Fault (for tests): lost_after_s >= 0 stops all UWB ranging that many seconds after
the drone arms (a dead UWB radio); the drone must fall back to the robot's radio
report, last known position and camera search.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import VehicleStatus

from las_offboard.uwb_anchors import anchors_world

TAG_HEIGHT_M = 0.30     # UWB antenna on top of the drone, above the model origin


class UwbSim(Node):
    def __init__(self):
        super().__init__('uwb_sim')
        p = self.declare_parameter
        self.noise = p('noise_m', 0.10).value
        self.nlos = p('nlos_prob', 0.10).value
        self.max_range = p('max_range_m', 80.0).value
        self.rng = np.random.default_rng(p('seed', 0).value or None)
        self.lost_after = p('lost_after_s', -1.0).value
        self.armed_at = None
        self.create_subscription(VehicleStatus, 'fmu/out/vehicle_status_v1', self.on_status,
                                 QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                            history=HistoryPolicy.KEEP_LAST, depth=1))
        self.drone = None
        self.robot = None
        self.create_subscription(Odometry, '/sim/ground_truth/odometry',
                                 lambda m: setattr(self, 'drone', m), 10)
        self.create_subscription(PoseStamped, '/sim/ground_truth/robot',
                                 lambda m: setattr(self, 'robot', m), 10)
        self.pub = self.create_publisher(Float32MultiArray, '/uwb/ranges', 10)
        self.create_timer(0.1, self.tick)
        self.get_logger().info(f'UWB: noise {self.noise:.2f} m, NLOS {self.nlos:.0%}, '
                               f'range {self.max_range:.0f} m, 10 Hz'
                               + (f', LOST {self.lost_after:.0f} s after arming (fault)'
                                  if self.lost_after >= 0 else ''))

    def on_status(self, msg):
        if self.armed_at is None and msg.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.armed_at = self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        if self.drone is None or self.robot is None:
            return
        if self.lost_after >= 0 and self.armed_at is not None:
            if self.get_clock().now().nanoseconds * 1e-9 - self.armed_at >= self.lost_after:
                self.get_logger().warn('UWB lost (fault injected)', throttle_duration_sec=30.0)
                return
        d = self.drone.pose.pose.position
        tag = np.array([d.x, d.y, d.z + TAG_HEIGHT_M])
        r = self.robot.pose
        yaw = 2.0 * math.atan2(r.orientation.z, r.orientation.w)
        anchors = anchors_world(r.position.x, r.position.y, yaw)
        true = np.linalg.norm(anchors - tag, axis=1)
        meas = true + self.rng.normal(0.0, self.noise, 4)
        nlos = self.rng.random(4) < self.nlos
        meas[nlos] += self.rng.uniform(0.3, 2.5, int(nlos.sum()))
        meas[true > self.max_range] = math.nan
        out = Float32MultiArray()
        out.data = [float(x) for x in meas]
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = UwbSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
