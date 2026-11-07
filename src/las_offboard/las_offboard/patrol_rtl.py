"""Patrol around a home point, then return and land when the battery gets low.

PX4 offboard control, driven by feedback (position, status, battery) instead of timers.

States:
  WAIT_FOR_PX4 -> PREFLIGHT -> ARMING -> TAKEOFF -> PATROL -> RETURN -> LAND -> DONE
  HANDOFF: PX4 or a pilot took over (left offboard). We stop commanding and only watch.

Home is the local (NED) position where the drone sits before arming, so this works
without GPS as long as PX4 has a valid local position estimate.

Coordinate note: PX4 local frame is NED (North-East-Down), so z is NEGATIVE when
the drone is above home. "Climb to altitude_m" means target z = home_z - altitude_m.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                       DurabilityPolicy, HistoryPolicy)
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleStatus, VehicleLocalPosition, BatteryStatus,
                          VehicleLandDetected)

# Control loop rate. PX4 requires offboard heartbeats faster than 2 Hz or it
# drops out of offboard mode, so 10 Hz leaves plenty of margin.
RATE_HZ = 10
# NaN in a TrajectorySetpoint field tells PX4 "don't control this", so only
# the position fields we fill in are used.
NAN = float('nan')
# States where the drone is in the air under our control; the safety checks in
# on_timer() (still in offboard? position still valid?) apply to these.
FLYING_STATES = ('TAKEOFF', 'PATROL', 'RETURN')
# States where we must keep streaming heartbeat + setpoint. PREFLIGHT and
# ARMING are included because PX4 refuses to switch into offboard unless
# setpoints are already arriving.
STREAMING_STATES = ('PREFLIGHT', 'ARMING') + FLYING_STATES


class PatrolRtl(Node):
    # node_name lets subclasses (e.g. patrol_lidar) reuse this class under their own name.
    def __init__(self, node_name='patrol_rtl'):
        super().__init__(node_name)

        # ---------- parameters (override with --ros-args -p name:=value) ----------
        p = self.declare_parameter
        self.alt = p('altitude_m', 5.0).value              # patrol height above home
        self.size = p('patrol_size_m', 10.0).value         # side length of the square
        # A rectangle instead (multi-drone: one strip each): north-south side
        # (< 0: same as patrol_size_m, which is then the east-west side) and its
        # centre's offset from home (ENU, m).
        self.size_n = p('patrol_size_n_m', -1.0).value
        self.center_en = (p('patrol_center_e_m', 0.0).value, p('patrol_center_n_m', 0.0).value)
        self.return_batt = p('return_battery', 0.35).value   # 0..1
        self.tol = p('reach_tolerance_m', 0.5).value       # "arrived" radius, h and v
        # MAVLink system ID of the vehicle; 0 = take it from VehicleStatus.
        self.target_system = p('target_system', 0).value
        # Topics are relative so a namespace (e.g. -r __ns:=/px4_1) picks the vehicle.
        status_topic = p('status_topic', 'fmu/out/vehicle_status_v1').value
        pos_topic = p('local_position_topic', 'fmu/out/vehicle_local_position_v1').value
        batt_topic = p('battery_topic', 'fmu/out/battery_status_v1').value
        land_topic = p('land_detected_topic', 'fmu/out/vehicle_land_detected').value

        # QoS that matches what the PX4 uXRCE-DDS bridge uses. With mismatched QoS
        # (e.g. RELIABLE subscriber vs BEST_EFFORT publisher) no messages arrive.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---------- outputs to PX4 ----------
        # offboard_control_mode: heartbeat saying which setpoint type we control.
        self.mode_pub = self.create_publisher(
            OffboardControlMode, 'fmu/in/offboard_control_mode', qos)
        # trajectory_setpoint: the actual position target.
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, 'fmu/in/trajectory_setpoint', qos)
        # vehicle_command: one-shot commands (arm, set mode, land, set home).
        self.cmd_pub = self.create_publisher(
            VehicleCommand, 'fmu/in/vehicle_command', qos)

        # ---------- inputs from PX4 ----------
        self.create_subscription(VehicleStatus, status_topic, self.on_status, qos)
        self.create_subscription(VehicleLocalPosition, pos_topic, self.on_pos, qos)
        self.create_subscription(BatteryStatus, batt_topic, self.on_batt, qos)
        self.create_subscription(VehicleLandDetected, land_topic, self.on_land, qos)

        # Latest message from each topic, plus receive time for staleness checks.
        self.status = None
        self.pos = None
        self.pos_time = None
        self.batt = None
        self.batt_time = None
        self.landed = True      # assume on the ground until PX4 says otherwise

        # ---------- state machine bookkeeping ----------
        self.state = 'WAIT_FOR_PX4'
        self.state_ticks = 0    # timer ticks spent in the current state
        self.tries = 0          # command retries in the current state
        self.hold_ticks = 0     # consecutive ticks spent at the target (RETURN)
        self.land_confirmed = False   # PX4 has switched to AUTO_LAND
        self.home = None        # (x, y, z, yaw) in local NED
        self.target = None      # (x, y, z) currently streamed
        self.waypoints = []     # the four corners of the patrol square
        self.wp_index = 0       # which corner we're heading to
        self.laps = 0
        self.done = False       # tells main() to exit

        self.timer = self.create_timer(1.0 / RATE_HZ, self.on_timer)
        self.get_logger().info(
            f'patrol_rtl: alt={self.alt} m, square={self.size} m, '
            f'return at battery <= {self.return_batt:.0%}')

    # ---------- subscriptions ----------
    def now_s(self):
        """Monotonic seconds, for measuring how old received data is.

        Not the wall clock: a clock step (NTP correcting the time, e.g. after a laptop
        sleep) made every message look 2 s old and landed the drone in place."""
        return time.monotonic()

    def on_status(self, msg):
        self.status = msg
        # Auto-detect which vehicle to command from the first status message.
        # Matters in multi-vehicle sims where each PX4 has its own system ID.
        if self.target_system == 0:
            self.target_system = msg.system_id
            self.get_logger().info(f'Commanding MAVLink system {self.target_system}')

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
        """Timestamp in microseconds, the unit PX4 messages use."""
        return int(self.get_clock().now().nanoseconds / 1000)

    def armed(self):
        return (self.status is not None and
                self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    def nav_state(self):
        """PX4's current flight mode (OFFBOARD, AUTO_LAND, AUTO_RTL, ...)."""
        return None if self.status is None else self.status.nav_state

    def in_offboard(self):
        return self.nav_state() == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def pos_ok(self):
        """True if we have a valid local position estimate newer than 0.5 s."""
        return (self.pos is not None and self.pos.xy_valid and self.pos.z_valid
                and self.now_s() - self.pos_time < 0.5)

    def battery_low(self):
        """True if we should go home.

        Fails safe: missing, stale (>2 s), disconnected or invalid battery data
        all count as low, so we never keep patrolling blind.
        """
        if self.batt is None or self.now_s() - self.batt_time > 2.0:
            self.get_logger().warn('No recent battery data', throttle_duration_sec=5.0)
            return True
        r = self.batt.remaining   # fraction 0..1; PX4 reports -1 when unknown
        if not self.batt.connected or r < 0.0 or math.isnan(r):
            self.get_logger().warn('Battery state unknown', throttle_duration_sec=5.0)
            return True
        return r <= self.return_batt

    def batt_str(self):
        """Battery percentage for log messages."""
        return 'unknown' if self.batt is None else f'{self.batt.remaining:.0%}'

    def dist_to(self, target):
        """(horizontal, vertical) distance from the drone to an (x, y, z) point."""
        dx = self.pos.x - target[0]
        dy = self.pos.y - target[1]
        dz = self.pos.z - target[2]
        return math.hypot(dx, dy), abs(dz)

    def reached(self, target):
        """True when within reach_tolerance_m of target both horizontally and vertically."""
        horiz, vert = self.dist_to(target)
        return horiz < self.tol and vert < self.tol

    def set_state(self, new_state):
        """Switch state, log the transition, and reset the per-state counters."""
        self.get_logger().info(f'{self.state} -> {new_state} (battery {self.batt_str()})')
        self.state = new_state
        self.state_ticks = 0
        self.tries = 0
        self.hold_ticks = 0

    def publish_heartbeat(self):
        """Tell PX4 we're alive and controlling position (not velocity/attitude)."""
        msg = OffboardControlMode()
        msg.timestamp = self.now_us()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.mode_pub.publish(msg)

    def publish_setpoint(self, x, y, z):
        """Send a position target (local NED). Yaw is held at the heading from home."""
        msg = TrajectorySetpoint()
        msg.timestamp = self.now_us()
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [NAN, NAN, NAN]        # NaN = let PX4 work these out
        msg.acceleration = [NAN, NAN, NAN]
        msg.yaw = float(self.home[3]) if self.home else 0.0
        self.setpoint_pub.publish(msg)

    def send_command(self, command, param1=0.0, param2=0.0):
        """Send a MAVLink-style VehicleCommand to the target vehicle's autopilot."""
        msg = VehicleCommand()
        msg.timestamp = self.now_us()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = self.target_system
        msg.target_component = 1      # 1 = the autopilot component
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True      # mark it as coming from outside PX4
        self.cmd_pub.publish(msg)

    def begin_land(self):
        """Ask PX4 to land in place and move to LAND (which retries if needed)."""
        self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.land_confirmed = False
        self.set_state('LAND')

    # ---------- main loop ----------
    def on_timer(self):
        """Runs at RATE_HZ: safety checks, then the current state's step, then streaming."""
        self.state_ticks += 1

        # Safety checks that apply whenever we are flying the vehicle.
        if self.state in FLYING_STATES:
            # If PX4 left offboard (failsafe, RC override, mode switch in QGC),
            # respect that and stop sending setpoints.
            if not self.in_offboard():
                self.get_logger().warn(
                    'Left offboard mode (PX4 failsafe or pilot). Handing control back.')
                self.set_state('HANDOFF')
                return
            # Position setpoints are meaningless without a position estimate.
            if not self.pos_ok():
                self.get_logger().error('Lost local position estimate. Landing in place.')
                self.begin_land()
                return

        # Dispatch to the handler for the current state, e.g. 'PATROL' -> step_patrol().
        getattr(self, 'step_' + self.state.lower())()

        # Offboard needs a continuous stream of heartbeats and setpoints.
        # Keep streaming in LAND until PX4 confirms AUTO_LAND, so a dropped land
        # command doesn't leave us in offboard with no signal.
        streaming = (self.state in STREAMING_STATES or
                     (self.state == 'LAND' and not self.land_confirmed))
        if streaming and self.target is not None:
            self.publish_heartbeat()
            self.publish_setpoint(*self.target)

    def step_wait_for_px4(self):
        """Wait until PX4 is publishing status, a valid position, and battery data."""
        if self.status is not None and self.pos_ok() and self.batt is not None:
            self.set_state('PREFLIGHT')
        else:
            self.get_logger().info('Waiting for PX4 status, local position and battery...',
                                   throttle_duration_sec=5.0)

    def step_preflight(self):
        """Record home, build the patrol square, and pre-stream setpoints for 1 s."""
        if self.home is None:
            # Home = where the drone is sitting right now.
            hx, hy, hz = self.pos.x, self.pos.y, self.pos.z
            yaw = self.pos.heading if not math.isnan(self.pos.heading) else 0.0
            self.home = (hx, hy, hz, yaw)
            # First target: straight up above home (z decreases going up in NED).
            self.target = (hx, hy, hz - self.alt)
            # Square of side patrol_size_m centred on home, flown at patrol altitude
            # (or the rectangle set by patrol_size_n_m / patrol_center_*_m).
            he = self.size / 2.0
            hn = (self.size_n if self.size_n > 0 else self.size) / 2.0
            cx, cy = hx + self.center_en[1], hy + self.center_en[0]      # NED
            fz = hz - self.alt
            self.waypoints = [(cx + hn, cy + he, fz), (cx + hn, cy - he, fz),
                              (cx - hn, cy - he, fz), (cx - hn, cy + he, fz)]
            # Make PX4's own home match ours so its RTL backstop goes to the same spot.
            # param1 = 1 means "use current location".
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_HOME, 1.0)
            self.get_logger().info(
                f'Home set at local x={hx:.2f} y={hy:.2f} z={hz:.2f}')

        # Don't take off at all if we'd have to turn around immediately.
        if self.battery_low():
            self.get_logger().error(
                f'Battery {self.batt_str()} is already at or below the return '
                'threshold. Not taking off.')
            self.set_state('DONE')
            return

        # Stream setpoints for 1 s before requesting offboard, and give PX4's
        # estimate time to settle: right after the position source starts it
        # reports "horizontal position unstable" for a few seconds and arming then
        # fails (seen with two drones, where drone 0's visual odometry starts later
        # in its boot). So wait until the position has been valid for 5 s.
        # (VehicleStatus.pre_flight_checks_pass can't be used: it is for PX4's
        # current mode, which wants a global position that a GPS-free drone lacks.)
        if not self.pos_ok():
            self.pos_ok_since = None
        elif getattr(self, 'pos_ok_since', None) is None:
            self.pos_ok_since = self.now_s()
        settled = self.pos_ok_since is not None and self.now_s() - self.pos_ok_since >= 5.0
        if self.state_ticks >= RATE_HZ and settled:
            self.set_state('ARMING')

    def step_arming(self):
        """Request offboard mode and arm, retrying once per second up to 10 times."""
        if self.armed() and self.in_offboard():
            self.set_state('TAKEOFF')
            return
        # state_ticks % RATE_HZ == 1 fires on the first tick and then once per second.
        if self.state_ticks % RATE_HZ == 1:
            if self.tries >= 10:
                self.get_logger().error(
                    'Could not arm or enter offboard. Check arming checks in QGC.')
                self.set_state('DONE')
                return
            self.tries += 1
            # DO_SET_MODE: param1=1 (custom mode), param2=6 (PX4 main mode OFFBOARD).
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            # COMPONENT_ARM_DISARM: param1=1 arms.
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def step_takeoff(self):
        """Climb to the point above home; start patrolling once there."""
        if self.battery_low():
            self.set_state('RETURN')
            return
        if self.reached(self.target):
            self.set_state('PATROL')

    def step_patrol(self):
        """Fly the square corner to corner until the battery hits the return threshold."""
        if self.battery_low():
            self.set_state('RETURN')
            return
        self.target = self.waypoints[self.wp_index]
        if self.reached(self.target):
            # Advance to the next corner; wrapping back to 0 completes a lap.
            self.wp_index = (self.wp_index + 1) % len(self.waypoints)
            if self.wp_index == 0:
                self.laps += 1
                self.get_logger().info(
                    f'Lap {self.laps} done (battery {self.batt_str()})')

    def step_return(self):
        """Fly back to the point above home, hover 2 s to settle, then land."""
        hx, hy, hz, _ = self.home
        self.target = (hx, hy, hz - self.alt)
        if self.reached(self.target):
            self.hold_ticks += 1
            if self.hold_ticks >= 2 * RATE_HZ:   # settle 2 s over home
                self.begin_land()
        else:
            # Drifted out of tolerance; restart the settle timer.
            self.hold_ticks = 0

    def step_land(self):
        """Make sure PX4 is in AUTO_LAND, then wait for touchdown and disarm."""
        if self.nav_state() == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            # PX4 accepted the land command; it now flies the descent itself.
            self.land_confirmed = True
        elif (not self.land_confirmed and self.state_ticks % RATE_HZ == 0
              and self.tries < 5):
            # Land command may have been dropped (BEST_EFFORT QoS); resend once a second.
            self.tries += 1
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

        # PX4 auto-disarms shortly after landing; that's our signal we're finished.
        if self.landed and not self.armed():
            if self.pos is not None and self.home is not None:
                horiz, _ = self.dist_to(self.home[:3])
                self.get_logger().info(
                    f'Landed and disarmed {horiz:.2f} m from home '
                    f'(battery {self.batt_str()}, laps {self.laps})')
            self.set_state('DONE')

    def step_handoff(self):
        """PX4/pilot is in charge. Send nothing; just wait for landed and disarmed."""
        self.get_logger().info('PX4 has control. Waiting for it to land and disarm.',
                               throttle_duration_sec=10.0)
        if self.landed and not self.armed():
            self.set_state('DONE')

    def step_done(self):
        """Terminal state: signal main() to shut the node down."""
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = PatrolRtl()
    try:
        # Spin manually (instead of rclpy.spin) so we can exit once the mission is DONE.
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
