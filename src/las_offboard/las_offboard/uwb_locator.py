"""Find the robot from UWB ranges to its 4 anchors (drone side).

The anchors are only ~1 m apart, so from 5-10 m away one set of 4 ranges hardly
says which direction the robot is in. Instead this node keeps the last window_s of
(drone position, ranges) pairs while the drone flies around, and solves for the
robot's position and heading that best explain all of them (robot standing still).
The drone's own motion supplies the geometry, as in tools/swarm_uwb_sim.py.

  In:   /uwb/ranges                        4 ranges (m), NaN = none (uwb_anchors order)
        fmu/out/vehicle_local_position_v1  PX4's own position estimate
  Out:  /uwb/dock_fix   geometry_msgs/PointStamped, frame "px4_local_ned": the robot's
                        centre (x north, y east; z = platform top), published only
                        when the fix is good. It is in PX4's own (drifting) frame,
                        i.e. relative to where the drone thinks it is, which is what
                        the drone needs to fly there: its own drift cancels out.

A fix is published when (a) after dropping NLOS readings (too long: blocked by a
trunk) the fit residual is small, and (b) the position uncertainty from the fit
geometry is below max_std_m. When new readings stop fitting (the robot moved), the
window is cleared and a new fix is built.

Assumes the robot stands on the same ground level as where the drone took off
(from the robot's platform), which holds on flat ground.
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import VehicleLocalPosition

from las_offboard.uwb_anchors import ANCHORS_BODY, PLATFORM_TOP_M

BASE_LINK_M = 0.24      # x500 body (PX4 local position) above its feet
TAG_ABOVE_BODY_M = 0.06  # UWB antenna above the body


def predict(theta, ground_u, pts):
    """Predicted ranges (K, 4) from drone tag positions pts (K, 3) for robot theta=(e, n, yaw)."""
    e, n, yaw = theta
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    anchors = ANCHORS_BODY @ rot.T + np.array([e, n, ground_u])          # (4, 3)
    diff = anchors[None, :, :] - pts[:, None, :]                          # (K, 4, 3)
    return np.linalg.norm(diff, axis=2), diff, rot


def gauss_newton(theta, ground_u, pts, meas, mask, iters=12):
    """Least squares over the readings in mask; returns theta, rms, JtJ."""
    theta = np.array(theta, dtype=float)
    jtj = np.eye(3)
    for _ in range(iters):
        pred, diff, rot = predict(theta, ground_u, pts)
        r = (meas - pred)[mask]
        u = diff / np.maximum(pred[..., None], 1e-6)                      # (K, 4, 3) unit
        c, s = math.cos(theta[2]), math.sin(theta[2])
        drot = np.array([[-s, -c, 0.0], [c, -s, 0.0], [0.0, 0.0, 0.0]])
        dyaw = (u * (ANCHORS_BODY @ drot.T)[None, :, :]).sum(axis=2)       # (K, 4)
        jac = np.stack([u[..., 0], u[..., 1], dyaw], axis=2)[mask]        # (M, 3)
        jtj = jac.T @ jac + 1e-6 * np.eye(3)
        step = np.linalg.solve(jtj, jac.T @ r)
        theta += step
        if np.linalg.norm(step[:2]) < 1e-3:
            break
    pred, _, _ = predict(theta, ground_u, pts)
    r = (meas - pred)[mask]
    return theta, float(np.sqrt(np.mean(r ** 2))) if r.size else math.inf, jtj


class UwbLocator(Node):
    def __init__(self):
        super().__init__('uwb_locator')
        p = self.declare_parameter
        self.window = p('window_s', 20.0).value
        self.sigma = p('range_noise_m', 0.10).value
        self.max_std = p('max_std_m', 0.3).value
        self.max_rms = p('max_rms_m', 0.25).value
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(VehicleLocalPosition, 'fmu/out/vehicle_local_position_v1',
                                 self.on_pos, qos)
        self.create_subscription(Float32MultiArray, '/uwb/ranges', self.on_ranges, 10)
        self.pub = self.create_publisher(PointStamped, '/uwb/dock_fix', 10)
        self.pos = None
        self.ground_u = None          # robot ground level, local ENU up
        self.samples = []             # (t, tag_enu (3,), ranges (4,))
        self.theta = None             # last solution (e, n, yaw), local ENU
        self.last_report = 0.0
        self.create_timer(1.0, self.solve)
        self.get_logger().info(f'UWB locator: {self.window:.0f} s window, fix when '
                               f'position std < {self.max_std:.2f} m')

    def now(self):
        return time.monotonic()          # sample ages; immune to wall-clock steps

    def on_pos(self, msg):
        if msg.xy_valid and msg.z_valid:
            if self.pos is not None and msg.xy_reset_counter != self.pos.xy_reset_counter:
                # PX4 reset its position estimate: move the stored track (and the last
                # solution) into the new frame, or old and new readings disagree.
                de, dn = float(msg.delta_xy[1]), float(msg.delta_xy[0])
                self.samples = [(t, tag + np.array([de, dn, 0.0]), r) for t, tag, r in self.samples]
                if self.theta is not None:
                    self.theta = self.theta + np.array([de, dn, 0.0])
                self.get_logger().info(f'PX4 position reset ({math.hypot(de, dn):.2f} m): '
                                       'shifted stored track')
            self.pos = msg
            if self.ground_u is None:
                # Took off from the robot's platform: its ground is below our start.
                self.ground_u = -msg.z - BASE_LINK_M - PLATFORM_TOP_M

    def on_ranges(self, msg):
        if self.pos is None or self.ground_u is None or len(msg.data) != 4:
            return
        r = np.array(msg.data, dtype=float)
        if np.all(np.isnan(r)):
            return
        tag = np.array([self.pos.y, self.pos.x, -self.pos.z + TAG_ABOVE_BODY_M])   # ENU
        t = self.now()
        self.samples.append((t, tag, r))
        self.samples = [s for s in self.samples if t - s[0] <= self.window]

    def starts(self, pts, meas):
        """Initial guesses: the last solution, and a ring around the latest position."""
        out = [] if self.theta is None else [self.theta.copy()]
        p, r = pts[-1], np.nanmedian(meas[-1])
        dz = p[2] - (self.ground_u + 0.55)
        dh = math.sqrt(max(r * r - dz * dz, 0.25))
        for phi in np.radians(np.arange(0, 360, 30)):
            for yaw in np.radians([0, 90, 180, 270]):
                out.append(np.array([p[0] + dh * math.cos(phi), p[1] + dh * math.sin(phi), yaw]))
        return out

    def fit(self, pts, meas):
        valid = ~np.isnan(meas)
        best = None
        for th0 in self.starts(pts, meas):
            th, rms, jtj = gauss_newton(th0, self.ground_u, pts, meas, valid, iters=8)
            if best is None or rms < best[1]:
                best = (th, rms, jtj)
        th, _, _ = best
        # Drop readings far too long (NLOS: blocked by a trunk) and refit, a few
        # rounds: the first fit is pulled off by the NLOS readings, so classifying
        # once against it throws away good readings too.
        for _ in range(4):
            pred, _, _ = predict(th, self.ground_u, pts)
            res = meas - pred
            inlier = valid & (res < 3.0 * self.sigma) & (res > -4.0 * self.sigma)
            th, rms, jtj = gauss_newton(th, self.ground_u, pts, meas, inlier, iters=12)
        cov = np.linalg.inv(jtj) * self.sigma ** 2
        std = math.sqrt(cov[0, 0] + cov[1, 1])
        return th, rms, std, inlier.sum() / max(valid.sum(), 1)

    def solve(self):
        # No ranges for a while (UWB lost or out of range): publish nothing. Solving on
        # from the old window would keep reporting where the robot WAS, which is
        # wrong if it has moved since, and would outrank a good radio report.
        if len(self.samples) < 20 or self.now() - self.samples[-1][0] > 2.0:
            return
        pts = np.array([s[1] for s in self.samples])
        meas = np.array([s[2] for s in self.samples])
        th, rms, std, inliers = self.fit(pts, meas)

        # Robot moved? The newest readings no longer fit: keep only those, start again.
        if rms > 2.0 * self.max_rms and len(self.samples) > 40:
            pred, _, _ = predict(th, self.ground_u, pts[-20:])
            recent = np.nanmedian(np.abs(meas[-20:] - pred))
            if recent > 0.5:
                self.get_logger().info('UWB readings stopped fitting (robot moved?): new fix')
                self.samples = self.samples[-20:]
                self.theta = None
                return
        good = rms < self.max_rms and std < self.max_std and inliers > 0.6
        if good:
            self.theta = th
            fix = PointStamped()
            fix.header.stamp = self.get_clock().now().to_msg()
            fix.header.frame_id = 'px4_local_ned'
            fix.point.x, fix.point.y = float(th[1]), float(th[0])          # NED: north, east
            fix.point.z = float(-(self.ground_u + PLATFORM_TOP_M))
            self.pub.publish(fix)
        if self.now() - self.last_report > 5.0:
            self.last_report = self.now()
            self.get_logger().info(
                f'{"fix" if good else "no fix"}: robot at E={th[0]:.2f} N={th[1]:.2f} '
                f'heading {math.degrees((th[2] + math.pi) % (2 * math.pi) - math.pi):.0f} deg, '
                f'rms {rms:.2f} m, std {std:.2f} m, '
                f'inliers {inliers:.0%}, {len(self.samples)} readings')


def main(args=None):
    rclpy.init(args=args)
    node = UwbLocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
