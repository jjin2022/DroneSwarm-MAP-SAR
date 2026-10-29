"""Arm, take off to a fixed altitude, hover, then land (PX4 offboard)."""
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                       DurabilityPolicy, HistoryPolicy)
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand  # pyright: ignore[reportMissingImports]

RATE_HZ = 10.0
TAKEOFF_ALT_M = 5.0      # meters above start
HOVER_SECONDS = 15.0     # time in offboard before landing
LAND_WAIT_SECONDS = 25.0 # time to wait for landing before exiting
NAN = float('nan')


class TakeoffLand(Node):
    def __init__(self):
        super().__init__('takeoff_land')
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

        self.tick = 0
        self.engage_tick = int(1.0 * RATE_HZ)
        self.land_tick = self.engage_tick + int(HOVER_SECONDS * RATE_HZ)
        self.end_tick = self.land_tick + int(LAND_WAIT_SECONDS * RATE_HZ)
        self.done = False
        self.timer = self.create_timer(1.0 / RATE_HZ, self.on_timer)

    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

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
        msg.yaw = 0.0
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

    def on_timer(self):
        # Keep the heartbeat and setpoint streaming until landing starts.
        if self.tick < self.land_tick:
            self.publish_heartbeat()
            self.publish_setpoint(0.0, 0.0, -TAKEOFF_ALT_M)

        if self.tick == self.engage_tick:
            self.get_logger().info('Switching to offboard mode and arming')
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.send_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        if self.tick == self.land_tick:
            self.get_logger().info('Landing')
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

        if self.tick >= self.end_tick:
            self.get_logger().info('Sequence complete')
            self.done = True

        self.tick += 1


def main(args=None):
    rclpy.init(args=args)
    node = TakeoffLand()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
