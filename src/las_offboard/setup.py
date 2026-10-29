from setuptools import find_packages, setup

package_name = 'las_offboard'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)
