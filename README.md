## Spinther Robotics

> Tagline: "One spark. Every search." | One-liner: "Autonomous scouts for the world’s riskiest terrain"

## Legged-Aerial Swarm

> A quadruped "mothership" that carries, docks, charges, and relays for a team of small drones, with shared SLAM for GPS-denied mapping, search and rescue (SAR), and reconnaissance. 

**Status:** Pre-alpha / Phase 1 simulation exit criterion met (50 consecutive faulted sorties, PX4 SITL + Gazebo); Phase 2 (hardware) next. See [Current State](#current-state)
**Visibility:** Private repository
**License:** Proprietary, all rights reserved (see [License](#license))

---

## Table of Contents

- [Overview](#overview)
- [Why This Pairing](#why-this-pairing)
- [Goals and Non-Goals](#goals-and-non-goals)
- [System Architecture](#system-architecture)
- [Current State](#current-state)
- [Repository Structure](#repository-structure)
- [Getting Started](#getting-started)
- [Roadmap](#roadmap)
- [Hardware](#hardware)
- [Testing](#testing)
- [Safety, Legal, and Ethics](#safety-legal-and-ethics)
- [Contributing](#contributing)
- [References](#references)
- [License](#license)

---

## Overview

A quadruped robot serves as the ground relay station, map server, charging dock, and transport for a team of drones. The robot spends its large battery on travel. The drones spend their small batteries only on the final stretch into places the robot cannot reach: over rubble, up shafts, through windows, across gaps.

The system is designed to operate **without GPS** and under **degraded or contested communications**.

**Target applications**

| Application | What the system does |
|---|---|
| Disaster mapping | Robot walks to the zone edge; drones map above and into structures; robot merges a shared 3D map |
| Search and rescue | Drones sweep with thermal and visual sensors; robot relays detections to a human operator and anchors comms inside structures |
| Defensive reconnaissance | Drones scout ahead of a ground team and report through the robot, reducing drone RF emissions and extending reach |

## Why This Pairing

| | Quadruped | Drone |
|---|---|---|
| Strengths | Long endurance, large battery, payload capacity, handles stairs and rubble, stable relay | Fast, 3D access, sees over obstacles, enters voids |
| Weaknesses | Slow, cannot cross gaps or climb vertically | 15 to 30 minute flights, short comms range |

**Design principle:** the robot is the stable backbone (map, power, comms, mission logic). Drones are expendable, short-range extensions of its senses.

## Goals and Non-Goals

**Goals**

- Operate with no GPS and with degraded or jammed communications
- Autonomous drone launch, sortie, return, landing, and recharge on the robot
- One shared, drift-corrected map built by robot and drones together
- A human operator stays in the loop for mission goals and any consequential decision

**Non-goals for v1**

- Landing on a moving robot
- Large swarms (start with one drone, scale to three or four)
- Autonomous decisions affecting people of any kind

## System Architecture

```
                 +----------------------+
                 |  Operator / Command  |
                 +----------+-----------+
                            | long-range backhaul
                 +----------v-----------+
                 |   QUADRUPED ROBOT    |
                 |  - Mission manager   |
                 |  - LiDAR-inertial    |
                 |    SLAM + map server |
                 |  - Energy manager    |
                 |  - Dock + charging   |
                 |  - Comms hub         |
                 +----+------------+----+
         short-range  |            |  short-range
         drone link   |            |  drone link
                 +----v---+    +---v----+
                 | Drone  |    | Drone  |   ...
                 | VIO,   |    | VIO,   |
                 | sortie |    | sortie |
                 | logic  |    | logic  |
                 +--------+    +--------+
```

**Layers**

1. **Mission manager:** goals, launch-point selection, mission state (behavior tree or state machine)
2. **Robot navigation:** ROS 2, Nav2 or custom planner, terrain traversability
3. **Drone agent:** flight controller (PX4 or ArduPilot) plus companion computer running VIO and sortie logic
4. **Communications:** short-range drone link, long-range backhaul, store-and-forward buffering
5. **Mapping and fusion:** robot merges drone submaps into a global map and shares pieces back
6. **Energy manager:** team-level battery budget for robot travel, charging, and sorties

**Drone lifecycle**

```
DOCKED -> CHARGING -> READY -> LAUNCH -> SORTIE -> RETURN -> APPROACH -> LAND -> DOCKED
```

Every state has a defined failsafe (low battery, lost link, lost dock, VIO failure, abort).

## Current State

Phase 1 (simulation) has met its exit criterion: **50 consecutive autonomous sorties
with injected faults**, twice (2026-11-04 and 2026-11-05). One simulated PX4 drone
(x500 with a 2D lidar and a downward camera) flies without GPS from a simulated
robot, patrols a forest, finds the robot again after it has moved, and lands on the
pad on its back.

**A sortie** (`tools/sortie_runner/`, scored against Gazebo ground truth)

1. Take off from the robot. No GPS: the drone flies on simulated visual odometry
   (1% drift); GPS, when switched on, is fused alongside and may drop mid-flight.
2. Patrol a square with lidar avoidance until the battery reaches the return level.
3. Meanwhile the robot walks to a random clear spot in the forest (its own odometry
   drifts 3%).
4. Find the robot: a priority chain of UWB fix (4 anchors on the robot) → robot
   radio report → last known position → camera search for the pad.
5. Precision-land on the ArUco pad on the robot's back.

About half the sorties get one fault: robot radio goes silent, robot reports a
position 2–4 m wrong, or UWB ranging dies. A sortie passes only if it lands on the
pad with no human, never closer than 0.38 m (prop tips) to a trunk.

**ROS 2 package `las_offboard`** (`src/las_offboard/`)

| Node | What it does |
|---|---|
| `patrol_lidar` | The drone: patrol, lidar avoidance, occupancy map (`/map`), object tracing (`/objects`), return, robot-finding chain, pad search, precision landing. Refuses to fly if a topic has extra publishers (e.g. a bag replay) |
| `pad_detector` | Finds the ArUco pad in the downward camera |
| `uwb_locator` | Turns UWB ranges into a robot position fix (multi-epoch solve, NLOS rejection) |
| `sim_vio` | Simulated visual odometry for PX4 (Gazebo truth + drift) |
| `robot_sim` | Simulated robot: relocates mid-sortie, radios its (drifting) position; radio faults |
| `uwb_sim` | Simulated UWB ranges with noise, NLOS and a dead-radio fault |
| `fault_inject` | Turns PX4's simulated GPS off/on mid-flight |
| `patrol_rtl`, `takeoff_land`, `lidar_test` | Simpler patrol, link smoke test, raw lidar check |

Also: `lidar_bridge.launch.py` (Gazebo → ROS 2), an RViz layout, the robot and drone
models (`models/`), and the worlds (`worlds/`: `forest_pillars` with 30 trunks in
15 × 15 m, a robot variant, pad variants; `gen_forest.py`, `random_pad.py`).

**Results (simulation)**

| Run | Sorties | Passed | Best in a row |
|---|---|---|---|
| 2026-11-04: radio and robot-odometry faults | 80 | 78 | 50 |
| 2026-11-05: adds UWB loss | 140 | 133 | 50 |

- Landings 0.4–5 cm from the pad centre; the robot was found after every fault type.
- Failures were trunk contacts during avoidance (3), PX4 SITL and laptop problems
  (5: fixed, see `patches/README.md`), and one run of repeated PX4 position resets.
- PX4 needs a local patch for a SITL timer race: `patches/px4-sitl-hrt-double-enqueue.patch`.

Not yet: the 8-inch drone model, 3D lidar, depth and thermal cameras, and the
funnel dock from `docs/drone-design.md`.

## Repository Structure

**Current layout**

```
.
├── README.md
├── docs/
│   ├── setup.md                    # Tested versions; terminal-by-terminal start-up
│   ├── running_sim.md              # Sim commands, node parameters, troubleshooting
│   ├── demo.md                     # How to run the one-command demo
│   ├── simulation_test_11126.md    # Lidar sim test checklist
│   ├── drone-design.md             # Scout drone hardware, docking, GPS-denied swarm localization
│   └── *.pdf                       # Requirements and scope; SAR statistics
├── scripts/
│   ├── demo_patrol_lidar.sh        # Start/stop the whole sim + patrol_lidar; saves nothing
│   └── px4_nogps.sh                # Start PX4 without GPS (own params folder)
├── patches/                        # Local PX4 fixes this project needs (see README there)
├── tools/
│   ├── tracer_eval/                # Score the object tracer on recorded flights (no sim needed)
│   ├── avoid_eval/                 # Measure real clearance to obstacles in a recorded flight
│   ├── land_eval/                  # Measure landing error against the pad in a recorded flight
│   ├── sortie_runner/              # Phase 1 exit test: unattended sorties, faults, scoring
│   ├── drone_budget.py             # Drone mass / power / endurance estimate
│   └── swarm_uwb_sim.py            # GPS-loss swarm localization check (numpy)
└── src/
    └── las_offboard/               # ROS 2 Python package (ament_python)
        ├── las_offboard/           # Nodes (see Current State)
        ├── launch/                 # lidar_bridge.launch.py
        ├── models/                 # robot_dock (B2-class + pad + UWB), x500_lidar_cam_down
        ├── rviz/                   # patrol_lidar.rviz
        └── worlds/                 # forest_pillars*, lidar_test; gen_forest.py, random_pad.py
```

`patrol_lidar` writes maps to `recordings/` (git-ignored) when run as in the docs.

**Target layout** (proposed; adjust as the project evolves)

```
.
├── README.md
├── LICENSE
├── docs/
│   ├── design-guide.md          # Full design and development guide
│   ├── architecture/            # Diagrams, interface definitions
│   ├── decisions/               # Architecture Decision Records (ADRs)
│   └── test-reports/            # Sim and field test results
├── src/
│   ├── mission_manager/         # Behavior trees, mission logic
│   ├── robot_nav/               # Launch-site selection, traversability
│   ├── drone_agent/             # Sortie logic, lifecycle state machine
│   ├── docking/                 # Fiducial landing, approach guidance
│   ├── mapping/                 # Submap exchange, merging, loop closure
│   ├── comms/                   # Mesh, store-and-forward, message signing
│   ├── energy_manager/          # Team energy budgeting
│   └── common/                  # Shared messages, utilities
├── sim/
│   ├── worlds/                  # Tunnels, rubble, buildings
│   ├── models/                  # Robot, drone, dock models
│   └── launch/                  # Simulation launch files
├── hardware/
│   ├── dock/                    # CAD, BOM for landing pad and latch
│   ├── wiring/                  # Power and data diagrams
│   └── bom.md                   # Bill of materials
├── config/                      # Parameters, topic maps, fault profiles
├── tools/                       # Scripts, log analysis, map evaluation
└── tests/
    ├── unit/
    ├── integration/
    └── fault_injection/         # Link loss, VIO failure, dock blocked, etc.
```

## Getting Started

**Prerequisites (tested)** — full list and versions in [docs/setup.md](docs/setup.md)

- Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic
- PX4-Autopilot v1.17.0 in `~/dev/PX4-Autopilot`
- `px4_msgs` (branch `release/1.17`) built in `~/dev/px4_ws`
- Micro XRCE-DDS Agent v2.4.3
- `ros-jazzy-ros-gz-bridge` (lidar → ROS 2)
- QGroundControl (PX4 won't arm without a ground station connected)

**Clone and build**

```bash
git clone git@github.com:jjin2022/DroneSwarm-MAP-SAR.git ~/dev/DroneSwarm-MAP-SAR
cd ~/dev/DroneSwarm-MAP-SAR
source /opt/ros/jazzy/setup.bash
source ~/dev/px4_ws/install/setup.bash
colcon build --packages-select las_offboard
```

**Link the worlds into PX4** (once; PX4 only loads worlds from its own folder)

```bash
for w in lidar_test forest_pillars; do
  ln -s ~/dev/DroneSwarm-MAP-SAR/src/las_offboard/worlds/$w.sdf \
        ~/dev/PX4-Autopilot/Tools/simulation/gz/worlds/$w.sdf
done
```

**Quick demo** — opens every window below in order, flies `patrol_lidar` with RViz
and QGroundControl, and saves nothing (no map, no objects CSV, no bag). Full
walkthrough: [docs/demo.md](docs/demo.md).

```bash
scripts/demo_patrol_lidar.sh              # forest_pillars; or: ... lidar_test / forest
scripts/demo_patrol_lidar.sh stop         # stop everything and close the windows
```

**Run the lidar patrol by hand** — one terminal each, in this order. Details and
troubleshooting: [docs/running_sim.md](docs/running_sim.md).

| # | Terminal | Command |
|---|---|---|
| 1 | DDS agent | `MicroXRCEAgent udp4 -p 8888` |
| 2 | Gazebo | `cd ~/dev/PX4-Autopilot && python3 Tools/simulation/gz/simulation-gazebo --world forest_pillars --model_store ~/dev/PX4-Autopilot/Tools/simulation/gz` |
| 3 | PX4 | `cd ~/dev/PX4-Autopilot && PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=forest_pillars make px4_sitl gz_x500_lidar_2d` |
| 4 | Lidar bridge | `ros2 launch las_offboard lidar_bridge.launch.py world:=forest_pillars` |
| 5 | QGroundControl | run the QGC AppImage |
| 6 | RViz | `rviz2 -d $(ros2 pkg prefix las_offboard)/share/las_offboard/rviz/patrol_lidar.rviz` |
| 7 | Patrol | `ros2 run las_offboard patrol_lidar --ros-args -p altitude_m:=2.0 -p map_dir:=$HOME/dev/DroneSwarm-MAP-SAR/recordings` |

Terminals 4, 6 and 7 need the ROS sourcing from the build step plus
`source ~/dev/DroneSwarm-MAP-SAR/install/setup.bash`. Swap `forest_pillars` for
`lidar_test` (in terminals 2–4) to use the other world.

**Gotchas**

- **Gazebo and PX4 terminals must not have ROS on `PATH`.** ROS Jazzy ships its own
  `gz` without `sim`, which gives "Timed out waiting for Gazebo world". In those two
  terminals, remove `/opt/ros` entries from `PATH`, `LD_LIBRARY_PATH` and
  `PYTHONPATH`, and `unset GZ_CONFIG_PATH`.
- **"Could not arm or enter offboard"** with no RC transmitter: PX4 wants stick
  input. Either turn on QGC's virtual joystick or, in the sim only, run
  `param set COM_RC_IN_MODE 4` at `pxh>` (saved; undo with `param set COM_RC_IN_MODE 0`).
- **Don't replay a bag while the sim runs**: `patrol_lidar` will refuse to take off
  (or land in place) if a topic has extra publishers.

## Roadmap

Each phase has an exit criterion. Do not advance until it is met.

- [ ] **Phase 0: Requirements and scoping.** Pick the primary use case and define measurable targets.
  *Exit:* one-page requirements sheet.
- [ ] **Phase 1: Simulation.** Full lifecycle with simulated robot, PX4 SITL drone, and fiducial landing.
  *Exit:* 50 consecutive autonomous simulated sorties with injected faults.
  *Progress:* exit criterion met twice (50 in a row; 2026-11-04, 2026-11-05) with a
  relocating robot, GPS-free flight, UWB/radio/camera robot finding, ArUco landing and
  radio, odometry and UWB faults. See `tools/sortie_runner/`.
- [ ] **Phase 2: Single drone, stationary robot.** Real hardware, tethered or netted, safety pilot override.
  *Exit:* 20 consecutive autonomous landings in varied lighting.
- [ ] **Phase 3: Shared SLAM.** Drone submaps merged into the robot's global map using the dock as the shared frame.
  *Exit:* merged map error within target against ground truth.
- [ ] **Phase 4: Robot mobility and rendezvous.** Relocate between sorties; handle a moved or lost robot.
  *Exit:* multi-leg mission with recovery from a forced rendezvous failure.
- [ ] **Phase 5: Multi-drone and task allocation.** Two to four drones, auction-based allocation, dock scheduling.
  *Exit:* team beats a single drone on coverage and handles a drone loss.
- [ ] **Phase 6: Degraded comms and GPS-denied hardening.** Throttle, drop, and (in controlled settings) disrupt links.
  *Exit:* mission completes or fails safe under defined degradation profiles.
- [ ] **Phase 7: Field trials.** Staged trials with domain experts.
  *Exit:* documented field test report.

## Hardware

> Verify current specs, pricing, and availability with vendors before purchasing.

The proposed scout drone (8-inch FPV-style quad with 3D lidar, depth and thermal
cameras, mesh + UWB radios, two battery packs and a docking collar), along with the plan for keeping the swarm localized
without GPS, is in [docs/drone-design.md](docs/drone-design.md).

| Component | Prototype options |
|---|---|
| Quadruped | **Chosen: Unitree B2-class** (payload for dock + 3–4 drones). Considered: Unitree Go2 (too little payload), Boston Dynamics Spot or ANYbotics ANYmal (cost) |
| Drone companion computer | ModalAI VOXL 2 (GPS-denied VIO) or NVIDIA Jetson Orin Nano / NX |
| Drone sensors | Global-shutter camera, IMU, rangefinder or optical flow, thermal camera (SAR), optional small LiDAR |
| Robot sensors and compute | LiDAR plus IMU, Jetson Orin-class computer |
| Dock | Funnel landing pad, AprilTag/ArUco fiducials, IR beacon, contact charging pads or battery swap, latch |
| Comms | Wi-Fi mesh for prototyping; frequency-hopping or spread-spectrum mesh radios for resilience; UWB for ranging |

Before buying a robot, confirm: payload capacity (dock, drones, and compute all count), usable runtime under load, ability to power an external dock, SDK and low-level data access, and ingress protection.

## Testing

- **Simulation regression suite:** every change runs the full lifecycle with fault injection
- **Hardware-in-the-loop:** real flight controller and companion computer with simulated sensors
- **Staged real testing:** tethered, then netted, then free flight
- **Metrics tracked:** launch-to-land success rate, landing accuracy, map error versus ground truth, time-to-map per square meter, energy per mission, link uptime, recovery time after a fault

**Fault scenarios to cover**

| Fault | Expected response |
|---|---|
| Drone loses the robot | Return to last known dock position, beacon search, safe-land |
| Dock blocked or damaged | Alternate landing site, queue other drones |
| Robot falls or is stuck | Self-recovery; drones hold battery margin |
| Map divergence | Versioned submaps, rollback, re-anchor at dock |
| VIO failure | Switch sensors or return on dead reckoning |
| Link jammed or spoofed | Frequency agility, authenticated messages, autonomous fallback |
| Operator link lost | Robot holds position and runs a conservative default |

## Safety, Legal, and Ethics

This project involves aircraft, lithium batteries, radio transmitters, and a legged robot operating near people. Read this section before any hardware testing.

- **Aviation rules:** comply with applicable drone regulations (for example FAA Part 107 in the US). Beyond-visual-line-of-sight, night, and over-people operations typically need waivers.
- **Radio rules:** jamming is illegal in most jurisdictions outside licensed or controlled settings. Test resilience using throttling, shielded setups, or approved test ranges, not live jamming.
- **Export controls:** autonomy, navigation, and anti-jam technology may fall under export-control regimes (such as ITAR and EAR in the US). Get compliance advice before sharing this repository outside your organization, and before any international collaboration.
- **Battery safety:** use fire-safe charging practices, thermal monitoring, and fusing around the dock.
- **Physical safety:** use e-stops, geofences, prop guards for early testing, and a cleared test area.
- **Human control:** detections are suggestions. A human operator remains responsible for any action affecting people.
- **Privacy:** define data retention and access rules for sensor data.

## Contributing

This is a private repository. Suggested workflow:

1. Create a feature branch from `main` (`feature/<short-name>`)
2. Keep changes small and focused; add or update tests
3. Run the simulation regression suite before opening a pull request
4. Record significant design choices as ADRs in `docs/decisions/`
5. Require at least one review before merging

**Do not commit:** credentials, encryption or signing keys, API tokens, or unreleased third-party licensed material. Use environment variables or a secrets manager, and consider enabling GitHub secret scanning.

## References

- DARPA Subterranean Challenge team papers (CERBERUS, CSIRO Data61)
- Multi-robot SLAM: CCM-SLAM, Kimera-Multi, DiSCO-SLAM
- LiDAR-inertial odometry: FAST-LIO2, LIO-SAM
- Visual-inertial odometry: OpenVINS, VINS-Fusion
- Flight stacks: PX4, ArduPilot
- Multi-drone framework: Aerostack2
- Multi-robot task allocation: auction-based methods, consensus-based bundle algorithm

## License

Proprietary. All rights reserved. This repository and its contents may not be copied, distributed, or used without written permission from the owner.

> If you later decide to open-source parts of the project, replace this section with the chosen license (for example Apache-2.0 or MIT) and add a `LICENSE` file.
