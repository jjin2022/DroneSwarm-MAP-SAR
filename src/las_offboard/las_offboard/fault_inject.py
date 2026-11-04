"""Inject a fault into PX4 SITL a set time after the drone arms (for tests).

    ros2 run las_offboard fault_inject --ros-args -p fault:=gps_off -p after_s:=30.0

Faults:
  gps_off   simulated GPS loses its fix (the drone must carry on with visual odometry)
  gps_on    GPS fix back

How: PX4's simulated GPS (sensor_gps_sim) reports "no fix" when SIM_GPS_USED (its
satellite count) is below 4, so this sets SIM_GPS_USED over MAVLink (0 = off,
10 = on). PX4's own failure injection (MAV_CMD_INJECT_FAILURE) does NOT work here:
only its older simulator bridge implements GPS failures, the Gazebo (gz) bridge
silently ignores them.

Run PX4 with scripts/px4_nogps.sh: the parameter change then lands in that run's
own parameter folder, never in your normal parameters. Robot-side faults (radio
drop, bad odometry) are robot_sim parameters.
"""
import threading

import rclpy
from pymavlink import mavutil
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleStatus

FAULTS = {'gps_off': ('SIM_GPS_USED', 0), 'gps_on': ('SIM_GPS_USED', 10)}


class FaultInject(Node):
    def __init__(self):
        super().__init__('fault_inject')
        self.fault = self.declare_parameter('fault', 'gps_off').value
        self.after = self.declare_parameter('after_s', 30.0).value
        self.mavlink_url = self.declare_parameter('mavlink', 'udpin:0.0.0.0:14540').value
        if self.fault not in FAULTS:
            raise SystemExit(f'Unknown fault {self.fault!r}; known: {", ".join(FAULTS)}')
        self.create_subscription(VehicleStatus, 'fmu/out/vehicle_status_v1', self.on_status,
                                 QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                            history=HistoryPolicy.KEEP_LAST, depth=1))
        self.armed_at = None
        self.done = False
        self.create_timer(0.5, self.tick)
        self.get_logger().info(f'Will inject {self.fault} {self.after:.0f} s after arming')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_status(self, msg):
        if self.armed_at is None and msg.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.armed_at = self.now()

    def tick(self):
        if self.done or self.armed_at is None or self.now() - self.armed_at < self.after:
            return
        self.done = True
        threading.Thread(target=self.set_param, daemon=True).start()   # don't block spinning

    def set_param(self):
        name, value = FAULTS[self.fault]
        try:
            link = mavutil.mavlink_connection(self.mavlink_url)
            link.wait_heartbeat(timeout=10)
            for _ in range(3):
                link.mav.param_set_send(link.target_system, 1, name.encode(), value,
                                        mavutil.mavlink.MAV_PARAM_TYPE_INT32)
                ack = link.recv_match(type='PARAM_VALUE', blocking=True, timeout=2,
                                      condition=f'PARAM_VALUE.param_id=="{name}"')
                if ack is not None and int(ack.param_value) == value:
                    self.get_logger().warn(f'Injected {self.fault} ({name}={value}) at '
                                           f'{self.now() - self.armed_at:.0f} s after arming')
                    return
            self.get_logger().error(f'PX4 did not confirm {name}={value}')
        except Exception as exc:          # report, don't crash the test
            self.get_logger().error(f'Could not inject {self.fault}: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = FaultInject()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
