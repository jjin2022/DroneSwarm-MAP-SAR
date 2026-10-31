# Working setup

- Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic
- PX4-Autopilot v1.17.0
- px4_msgs branch release/1.17
- Micro XRCE-DDS Agent v2.4.3
- Versioned topic: /fmu/out/vehicle_status_v1
- Do not run `make distclean`; clear builds with `rm -rf build`

# Starting the simulation, terminal by terminal

Open the terminals in the order listed. Each block shows the directory to be in and
every command to type. More detail and troubleshooting: [running_sim.md](running_sim.md).

## One drone: 3 terminals

| Terminal | Runs | Directory |
|---|---|---|
| 1 | Micro XRCE-DDS agent | any |
| 2 | PX4 + Gazebo | `~/dev/PX4-Autopilot` |
| 3 | `las_offboard` node | `~/dev/DroneSwarm-MAP-SAR` |

**Terminal 1: DDS agent** (leave running)
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 2: PX4 + Gazebo** (leave running; wait for the Gazebo window and `pxh>`)
```bash
cd ~/dev/PX4-Autopilot
make px4_sitl gz_x500
```
Then at the `pxh>` prompt (optional, lets the battery drain past 50%):
```
param set SIM_BAT_MIN_PCT 10
```

**Terminal 3: build and run the node**
```bash
cd ~/dev/DroneSwarm-MAP-SAR
source /opt/ros/jazzy/setup.bash
source ~/dev/px4_ws/install/setup.bash
colcon build --packages-select las_offboard     # only needed after code changes
source install/setup.bash
ros2 run las_offboard patrol_rtl                # or: ros2 run las_offboard takeoff_land
```

To stop: Ctrl+C in terminal 3, then 2, then 1.

## Two drones: 5 terminals

Make sure no `make px4_sitl` is still running.

| Terminal | Runs | Directory |
|---|---|---|
| 1 | Micro XRCE-DDS agent | any |
| 2 | PX4 drone 1 + Gazebo | `~/dev/PX4-Autopilot` |
| 3 | PX4 drone 2 | `~/dev/PX4-Autopilot` |
| 4 | node for drone 1 | `~/dev/DroneSwarm-MAP-SAR` |
| 5 | node for drone 2 | `~/dev/DroneSwarm-MAP-SAR` |

**Terminal 1: DDS agent**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 2: drone 1** (starts Gazebo; wait for the window and `pxh>`)
```bash
cd ~/dev/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1
```
At `pxh>`: `param set SIM_BAT_MIN_PCT 10`

**Terminal 3: drone 2** (spawns 15 m away in the same world)
```bash
cd ~/dev/PX4-Autopilot
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="0,15" ./build/px4_sitl_default/bin/px4 -i 2
```
At `pxh>`: `param set SIM_BAT_MIN_PCT 10`

**Terminal 4: node for drone 1**
```bash
cd ~/dev/DroneSwarm-MAP-SAR
source /opt/ros/jazzy/setup.bash
source ~/dev/px4_ws/install/setup.bash
source install/setup.bash
ros2 run las_offboard patrol_rtl --ros-args -r __ns:=/px4_1
```

**Terminal 5: node for drone 2**
```bash
cd ~/dev/DroneSwarm-MAP-SAR
source /opt/ros/jazzy/setup.bash
source ~/dev/px4_ws/install/setup.bash
source install/setup.bash
ros2 run las_offboard patrol_rtl --ros-args -r __ns:=/px4_2 -p altitude_m:=8.0
```

For a third drone, add one PX4 terminal (`-i 3`, `PX4_GZ_MODEL_POSE="0,30"`) and one
node terminal (`__ns:=/px4_3`).

Before starting the nodes, you can check both drones are publishing (in any sourced terminal):
```bash
ros2 topic list | grep px4_
```
