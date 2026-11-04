"""
lidar_bridge.launch.py - Make the simulated 2D LIDAR available as a ROS 2 topic.

WHY THIS IS NEEDED
------------------
Gazebo has its own messaging system (Gazebo Transport). The lidar publishes
there, not on ROS 2. `ros_gz_bridge` is a small program that subscribes on one
side and republishes on the other, converting the message type as it goes:

    Gazebo  gz.msgs.LaserScan  --(ros_gz_bridge)-->  ROS 2  sensor_msgs/msg/LaserScan

WHAT YOU GET IN ROS 2
---------------------
    /scan   sensor_msgs/msg/LaserScan   the lidar, ~30 Hz, header.frame_id = "link"
    /clock  rosgraph_msgs/msg/Clock     simulation time (for use_sim_time:=true nodes)
  with camera:=true (drone model x500_lidar_cam_down only):
    /camera/image_raw    sensor_msgs/msg/Image       downward camera, 640x480, 15 Hz
    /camera/camera_info  sensor_msgs/msg/CameraInfo  its intrinsics

HOW TO RUN
----------
    # after the sim is up (make px4_sitl gz_x500_lidar_2d)
    ros2 launch las_offboard lidar_bridge.launch.py
    # if you started a different world, pass its name (the <world name="..."> in the .sdf):
    ros2 launch las_offboard lidar_bridge.launch.py world:=lidar_test
    # the x500 with the downward camera, for precision landing:
    ros2 launch las_offboard lidar_bridge.launch.py world:=forest_pillars \
        model:=x500_lidar_cam_down_0 camera:=true

    # check it:
    ros2 topic hz /scan
    ros2 topic echo --once /scan
    # view it: run `rviz2`, set Fixed Frame to "link", Add > By topic > /scan > LaserScan
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_bridge(context):
    """Build the bridge node once the launch arguments have real values."""
    # Read the arguments as plain strings.
    world = LaunchConfiguration('world').perform(context)
    model = LaunchConfiguration('model').perform(context)
    camera = LaunchConfiguration('camera').perform(context).lower() == 'true'

    # The full Gazebo topic of the lidar. PX4 names the spawned drone
    # "<model>_<instance>", e.g. x500_lidar_2d_0 for `make px4_sitl`.
    # You can confirm the exact name with:  gz topic -l | grep scan
    gz_scan_topic = (f'/world/{world}/model/{model}'
                     '/link/link/sensor/lidar_2d_v2/scan')

    # Each bridge argument has the form:
    #     <topic>@<ROS 2 type><direction><Gazebo type>
    # where the direction character is:
    #     [   Gazebo -> ROS 2   (what we want: we only read the sensor)
    #     ]   ROS 2 -> Gazebo
    #     @   both ways
    bridge_args = [
        f'{gz_scan_topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        # Sim clock: lets ROS nodes use simulation time instead of wall time.
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
    ]
    remappings = [(gz_scan_topic, '/scan')]
    if camera:
        # Downward camera of the x500_lidar_cam_down model (sensor "down_cam").
        gz_cam = f'/world/{world}/model/{model}/link/camera_link/sensor/down_cam'
        bridge_args += [
            f'{gz_cam}/image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'{gz_cam}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ]
        remappings += [(f'{gz_cam}/image', '/camera/image_raw'),
                       (f'{gz_cam}/camera_info', '/camera/camera_info')]

    return [Node(
        package='ros_gz_bridge',          # installed by: sudo apt install ros-jazzy-ros-gz-bridge
        executable='parameter_bridge',    # the generic "bridge these topics" program
        name='lidar_bridge',
        arguments=bridge_args,
        # The Gazebo topic name is long and changes with the world/model, so we
        # rename (remap) it to a short, stable ROS name: /scan.
        remappings=remappings,
        output='screen',                  # show its log in this terminal
    )]


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments: values you can override on the command line
        # with name:=value.
        DeclareLaunchArgument(
            'world', default_value='default',
            description='Gazebo world name (PX4_GZ_WORLD), e.g. default or lidar_test'),
        DeclareLaunchArgument(
            'model', default_value='x500_lidar_2d_0',
            description='Spawned model name: x500_lidar_2d_<PX4 instance number>'),
        DeclareLaunchArgument(
            'camera', default_value='false',
            description='Also bridge the downward camera (x500_lidar_cam_down only)'),
        # OpaqueFunction lets us run Python (make_bridge) with the argument values.
        OpaqueFunction(function=make_bridge),
    ])
