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
  with vio:=true (x500_lidar_cam_down only), for flying without GPS:
    /sim/ground_truth/odometry  nav_msgs/msg/Odometry  the drone's TRUE pose; only
                                sim_vio (and test tools) may use it

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
    vio = LaunchConfiguration('vio').perform(context).lower() == 'true'
    # Namespace for a second (third...) drone: PX4 instance i>0 publishes under
    # /px4_<i>/fmu/..., and its nodes run in that namespace, so its sensors go
    # there too: ns:=px4_1 gives /px4_1/scan, /px4_1/camera/..., and the true pose
    # on /px4_1/sim/ground_truth/odometry (its model publishes that name, see
    # scripts/px4_nogps.sh). Empty (the default) = the single-drone names below.
    ns = LaunchConfiguration('ns').perform(context).strip('/')
    pre = f'/{ns}' if ns else ''

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
    ]
    if not ns:
        # Sim clock: lets ROS nodes use simulation time instead of wall time (one
        # bridge is enough; the namespaced ones leave it out).
        bridge_args.append('/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock')
    remappings = [(gz_scan_topic, f'{pre}/scan')]
    if camera:
        # Downward camera of the x500_lidar_cam_down model (sensor "down_cam").
        gz_cam = f'/world/{world}/model/{model}/link/camera_link/sensor/down_cam'
        bridge_args += [
            f'{gz_cam}/image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'{gz_cam}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ]
        remappings += [(f'{gz_cam}/image', f'{pre}/camera/image_raw'),
                       (f'{gz_cam}/camera_info', f'{pre}/camera/camera_info')]
    if vio:
        # True pose from the model's OdometryPublisher plugin, for sim_vio. The
        # model publishes it under this fixed name (not /model/<name>/...), which
        # keeps PX4's own Gazebo bridge from using it as a perfect position.
        bridge_args.append(f'{pre}/sim/ground_truth/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry')

    return [Node(
        package='ros_gz_bridge',          # installed by: sudo apt install ros-jazzy-ros-gz-bridge
        executable='parameter_bridge',    # the generic "bridge these topics" program
        name='lidar_bridge' + (f'_{ns}' if ns else ''),
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
        DeclareLaunchArgument(
            'vio', default_value='false',
            description='Also bridge the true pose for sim_vio (x500_lidar_cam_down only)'),
        DeclareLaunchArgument(
            'ns', default_value='',
            description='Namespace of a second drone, e.g. px4_1 (empty: single drone)'),
        # OpaqueFunction lets us run Python (make_bridge) with the argument values.
        OpaqueFunction(function=make_bridge),
    ])
