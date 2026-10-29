"""Patrol around a home point, then return and land when the battery gets low.

PX4 offboard control, driven by feedback (position, status, battery) instead of timers.

States:
  WAIT_FOR_PX4 -> PREFLIGHT -> ARMING -> TAKEOFF -> PATROL -> RETURN -> LAND -> DONE
  HANDOFF: PX4 or a pilot took over (left offboard). We stop commanding and only watch.

Home is the local (NED) position where the drone sits before arming, so this works
without GPS as long as PX4 has a valid local position estimate.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                       DurabilityPolicy, HistoryPolicy)
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleStatus, VehicleLocalPosition, BatteryStatus,
                          VehicleLandDetected)

RATE_HZ = 10
NAN = float('nan')
FLYING_STATES = ('TAKEOFF', 'PATROL', 'RETURN')
STREAMING_STATES = ('PREFLIGHT', 'ARMING') + FLYING_STATES


class PatrolRtl(Node):
    def __init__(self):
        super().__init__('patrol_rtl')

        p = self.declare_parameter
        self.alt = p('altitude_m', 5.0).value
        self.size = p('patrol_size_m', 10.0).value
        self.return_batt = p('return_battery', 0.35).value   # 0..1
        self.tol = p('reach_tolerance_m', 0.5).value
        status_topic = p('status_topic', '/fmu/out/vehicle_status_v1').value
        pos_topic = p('local_position_topic', '/fmu/out/vehicle_local_position_v1').value
        batt_topic = p('battery_topic', '/fmu/out/battery_status_v1').value
        land_topic = p('land_detected_topic', '/fmu/out/vehicle_land_detected').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        self.create_subscription(VehicleStatus, status_topic, self.on_status, qos)
        self.create_subscription(VehicleLocalPosition, pos_topic, self.on_pos, qos)
        self.create_subscription(BatteryStatus, batt_topic, self.on_batt, qos)
        self.create_subscription(VehicleLandDetected, land_topic, self.on_land, qos)

        self.status = None
        self.pos = None
        self.pos_time = None
        self.batt = None
        self.batt_time = None
        self.landed = True

        self.state = 'WAIT_FOR_PX4'
        self.state_ticks = 0
        self.tries = 0
        self.hold_ticks = 0
        self.land_confirmed = False
        self.home = None        # (x, y, z, yaw) in local NED
        self.target = None      # (x, y, z) currently streamed
        self.waypoints = []
        self.wp_index = 0
        self.laps = 0
        self.done = False

        self.timer = self.create_timer(1.0 / RATE_HZ, self.on_timer)
        self.get_logger().info(
            f'patrol_rtl: alt={self.alt} m, square={self.size} m, '
            f'return at battery <= {self.return_batt:.0%}')

    # ---------- subscriptions ----------
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_status(self, msg):
        self.status = msg

    def on_pos(self, msg):
        self.pos = msg
        self.pos_time = self.now_s()

    def on_batt(self, msg):
        self.batt = msg
        self.batt_time = self.now_s()

    def on_land(self, msg):
        self.landed = msg.landed

    # ---------- helpers ----------
    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def armed(self):
        return (self.status is not None and
                self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    def nav_state(self):
        return None if self.status is None else self.status.nav_state

    def in_offboard(self):
        return self.nav_state() == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def pos_ok(self):
        return (self.pos is not None and self.pos.xy_valid and self.pos.z_valid
                and self.now_s() - self.pos_time < 0.5)

    def battery_low(self):
        if self.batt is None or self.now_s() - self.batt_time > 2.0:
            self.get_logger().warn('No recent battery data', throttle_duration_sec=5.0)
            return True
        r = self.batt.remaining
        if not self.batt.connected or r < 0.0 or math.isnan(r):
            self.get_logger().warn('Battery state unknown', throttle_duration_sec=5.0)
            return True
        return r <= self.return_batt

    def batt_str(self):
        return 'unknown' if self.batt is None else f'{self.batt.remaining:.0%}'

    def dist_to(self, target):
        dx = self.pos.x - target[0]
        dy = self.pos.y - target[1]
        dz = self.pos.z - target[2]
        return math.hypot(dx, dy), abs(dz)

    def reached(self, target):
        horiz, vert = self.dist_to(target)
        return horiz < self.tol and vert < self.tol

    def set_state(self, new_state):
        self.get_logger().info(f'{self.state} -> {new_state} (battery {self.batt_str()})')
        self.state = new_state
        self.state_ticks = 0
        self.tries = 0
        self.hold_ticks = 0

    def publish_heartbeat(self):
        msg = OffboardControlMode()
        msg.timestamp = self.now_us()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.mode_pub.publish(msg)

    def publish_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.timestamp = self.now_us()
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [NAN, NAN, NAN]
        msg.acceleration = [NAN, NAN, NAN]
        msg.yaw = float(self.home[3]) if self.home else 0.0
        self.setpoint_pub.publish(msg)

    def send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.now_us()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    def begin_land(self):
        self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.land_confirmed = False
        self.set_state('LAND')

    # ---------- main loop ----------
    def on_timer(self):
        self.state_ticks += 1

        # Safety checks that apply whenever we are flying the vehicle.
        if self.state in FLYING_STATES:
            if not self.in_offboard():
                self.get_logger().warn(
                    'Left offboard mode (PX4 failsafe or pilot). Handing control back.')
                self.set_state('HANDOFF')
                return
            if not self.pos_ok():
                self.get_logger().error('Lost local position estimate. Landing in place.')
                self.begin_land()
                return

        getattr(self, 'step_' + self.state.lower())()

        # Offboard needs a continuous stream of heartbeats and setpoints.
        if self.state in STREAMING_STATES and self.target is not None:
            self.publish_heartbeat()
            self.publish_setpoint(*self.target)

    def step_wait_for_px4(self):
        if self.status is not None and self.pos_ok():
            self.set_state('PREFLIGHT')
        else:
            self.get_logger().info('Waiting for PX4 status and local position...',
                                   throttle_duration_sec=5.0)

    def step_preflight(self):
        if self.home is None:
            hx, hy, hz = self.pos.x, self.pos.y, self.pos.z
            yaw = self.pos.heading if not math.isnan(self.pos.heading) else 0.0
            self.home = (hx, hy, hz, yaw)
            self.target = (hx, hy, hz - self.alt)
            h = self.size / 2.0
            fz = hz - self.alt
            self.waypoints = [(hx + h, hy + h, fz), (hx + h, hy - h, fz),
                              (hx - h, hy - h, fz), (hx - h, hy + h, fz)]
            # Make PX4's own home match ours so its RTL backstop goes to the same spot.
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_HOME, 1.0)
            self.get_logger().info(
                f'Home set at local x={hx:.2f} y={hy:.2f} z={hz:.2f}')

        if self.battery_low():
            self.get_logger().error(
                f'Battery {self.batt_str()} is already at or below the return '
                'threshold. Not taking off.')
            self.set_state('DONE')
            return

        # Stream setpoints for 1 s before requesting offboard.
        if self.state_ticks >= RATE_HZ:
            self.set_state('ARMING')

    def step_arming(self):
        if self.armed() and self.in_offboard():
            self.set_state('TAKEOFF')
            return
        if self.state_ticks % RATE_HZ == 1:
            if self.tries >= 5:
                self.get_logger().error(
                    'Could not arm or enter offboard. Check arming checks in QGC.')
                self.set_state('DONE')
                return
            self.tries += 1
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def step_takeoff(self):
        if self.battery_low():
            self.set_state('RETURN')
            return
        if self.reached(self.target):
            self.set_state('PATROL')

    def step_patrol(self):
        if self.battery_low():
            self.set_state('RETURN')
            return
        self.target = self.waypoints[self.wp_index]
        if self.reached(self.target):
            self.wp_index = (self.wp_index + 1) % len(self.waypoints)
            if self.wp_index == 0:
                self.laps += 1
                self.get_logger().info(
                    f'Lap {self.laps} done (battery {self.batt_str()})')

    def step_return(self):
        hx, hy, hz, _ = self.home
        self.target = (hx, hy, hz - self.alt)
        if self.reached(self.target):
            self.hold_ticks += 1
            if self.hold_ticks >= 2 * RATE_HZ:   # settle 2 s over home
                self.begin_land()
        else:
            self.hold_ticks = 0

    def step_land(self):
        if self.nav_state() == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            self.land_confirmed = True
        elif (not self.land_confirmed and self.state_ticks % RATE_HZ == 0
              and self.tries < 5):
            self.tries += 1
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

        if self.landed and not self.armed():
            if self.pos is not None and self.home is not None:
                horiz, _ = self.dist_to(self.home[:3])
                self.get_logger().info(
                    f'Landed and disarmed {horiz:.2f} m from home '
                    f'(battery {self.batt_str()}, laps {self.laps})')
            self.set_state('DONE')

    def step_handoff(self):
        self.get_logger().info('PX4 has control. Waiting for it to land and disarm.',
                               throttle_duration_sec=10.0)
        if self.landed and not self.armed():
            self.set_state('DONE')

    def step_done(self):
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = PatrolRtl()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()