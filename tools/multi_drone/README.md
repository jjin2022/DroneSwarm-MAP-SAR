# Two-drone patrol

Two drones take off from one robot, each patrols its own strip of the forest,
both find the robot again after it moves, and each lands on its own pad. It runs
the full GPS-free stack per drone. This is Phase 5 groundwork in simulation.

```bash
source /opt/ros/jazzy/setup.bash
source ~/dev/px4_ws/install/setup.bash
source ~/dev/DroneSwarm-MAP-SAR/install/setup.bash
python3 tools/multi_drone/run_multi.py             # with the Gazebo window, to watch
python3 tools/multi_drone/run_multi.py --headless  # no windows
```

It stops by itself once both drones have landed (about 4 minutes), or on Ctrl-C.
Logs, a bag and `report.txt` are written to `recordings/multi_<time>/`. QGroundControl
isn't needed, since the launcher turns off PX4's "no ground station" arming block.
One QGC can watch both drones if you want it: they show up as vehicles 1 and 2.

**One-time setup:** link `robot_dock_duo` and `forest_pillars_robot_duo` into PX4's
folders (see [docs/running_sim.md](../../docs/running_sim.md), "Two drones from one
robot"). The PX4 timer patch in `patches/` is needed, as it is for the sortie runner.

## How it's set up

| | Drone 0 | Drone 1 |
|---|---|---|
| Pad on the robot (body frame) | (0, +0.5) m, ArUco ID 0 | (0, −0.5) m, ArUco ID 1 |
| Patrol strip | east, E 0.75 to 5 m | west, E −5 to −0.75 m |
| Altitude | 2.0 m | 3.5 m |
| Takes off | first | 10 s later |
| Topics | single-drone names (`/fmu/...`, `/scan`) | under `/px4_1` |
| Robot's radio report | `/robot/dock_position` | `/robot/dock_position_1` |

- **No collisions by design.** The strips are 1.5 m apart, and the drones fly at
  different heights, so neither appears in the other's flat lidar scan.
- **Shared:** one DDS agent, Gazebo and `robot_sim`. The robot moves 35 s after
  drone 0 arms, when both drones are in the air.
- **Drone 1's model is a copy.** The Gazebo plugin that publishes a drone's true
  pose has its topic fixed in the model file, so `scripts/px4_nogps.sh` makes a copy
  for drone 1 that publishes on `/px4_1/sim/ground_truth/odometry`. Without it both
  drones' simulated visual odometry would read drone 0's pose.

## Scoring (`report.txt`)

Each drone passes if:
- it patrolled, returned, precision-landed and finished;
- it's resting on **its own** pad, inside the 0.5 m square;
- it came no closer than 0.38 m to a trunk.

The pair also has to stay at least 0.75 m apart, centre to centre, while either
drone is flying. The x500's props would touch at about 0.7 m.

First passing run (`multi_test4`): 2 laps and 1 lap; touchdowns 4.5 cm and 2.2 cm
from their pads; closest trunks 0.76 m and 0.64 m. While both were flying their
strips they stayed at least 2.1 m apart. The closest reading, 1.0 m, is with both
parked on the robot, because the pads are 1 m apart.

## Not done yet

- **No fault injection or many-run scoring yet.** It's one run at a time, with no
  radio drops or wrong robot position.
- **The drones don't coordinate.** Each flies its own plan, and they stay apart
  only because of the strips and altitudes. Nothing assigns tasks, and the drones
  can't use each other's UWB ranges.
- **Two drones only.** A third needs a third pad on `robot_dock_duo` and a third
  entry in `PADS` and `DRONES` in `run_multi.py`.
