# Legged-Aerial Swarm

> A quadruped "mothership" that carries, docks, charges, and relays for a team of small drones, with shared SLAM for GPS-denied mapping, search and rescue (SAR), and reconnaissance.

**Status:** Pre-alpha / design phase
**Visibility:** Private repository
**License:** Proprietary, all rights reserved (see [License](#license))

---

## Table of Contents

- [Overview](#overview)
- [Why This Pairing](#why-this-pairing)
- [Goals and Non-Goals](#goals-and-non-goals)
- [System Architecture](#system-architecture)
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

## Repository Structure

> Proposed layout. Adjust as the project evolves.

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

> Placeholder. Update as the stack is chosen and the first simulation comes online.

**Prerequisites (planned)**

- Ubuntu 22.04 or 24.04
- ROS 2 (Humble or Jazzy)
- PX4-Autopilot (SITL) or ArduPilot SITL
- Gazebo or NVIDIA Isaac Sim
- Python 3.10+, CMake, colcon

**Clone**

```bash
git clone git@github.com:<your-org>/<repo-name>.git
cd <repo-name>
```

**Build (once source packages exist)**

```bash
colcon build --symlink-install
source install/setup.bash
```

**Run the simulation (once available)**

```bash
ros2 launch sim launch_sortie_demo.launch.py
```

## Roadmap

Each phase has an exit criterion. Do not advance until it is met.

- [ ] **Phase 0: Requirements and scoping.** Pick the primary use case and define measurable targets.
  *Exit:* one-page requirements sheet.
- [ ] **Phase 1: Simulation.** Full lifecycle with simulated robot, PX4 SITL drone, and fiducial landing.
  *Exit:* 50 consecutive autonomous simulated sorties with injected faults.
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

| Component | Prototype options |
|---|---|
| Quadruped | Unitree Go2 (low-cost), Unitree B2 (larger payload), Boston Dynamics Spot or ANYbotics ANYmal (established platforms) |
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
