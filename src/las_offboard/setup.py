from glob import glob

from setuptools import find_packages, setup

package_name = 'las_offboard'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files and Gazebo worlds so `ros2 launch` and
        # get_package_share_directory('las_offboard') can find them.
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        # Drone models (PX4 loads them from its own folder: symlink them there, see docs).
        ('share/' + package_name + '/models/x500_lidar_cam_down', glob('models/x500_lidar_cam_down/*')),
        ('share/' + package_name + '/models/robot_dock', glob('models/robot_dock/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jjin2022',
    maintainer_email='jonathanzyjin@gmail.com',
    description='LAS offboard takeoff and land demo for PX4',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'takeoff_land = las_offboard.takeoff_land:main',
            'patrol_rtl = las_offboard.patrol_rtl:main',
            'patrol_lidar = las_offboard.patrol_lidar:main',
            'lidar_test = las_offboard.lidar_test:main',
            'pad_detector = las_offboard.pad_detector:main',
            'sim_vio = las_offboard.sim_vio:main',
            'robot_sim = las_offboard.robot_sim:main',
            'uwb_sim = las_offboard.uwb_sim:main',
            'uwb_locator = las_offboard.uwb_locator:main',
        ],
    },
)
