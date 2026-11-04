# LAS Scout Drone: Hardware and Swarm Localization Design

**Status:** Proposed design, rev B (2026-11-04). Nothing here is built or weighed yet.
Decisions made so far are listed in §11.
**Scope:** The drone that rides on, launches from and docks on the quadruped, and how
a team of these drones keeps a shared position picture when GPS is gone.

> Part numbers are candidates, not a purchase list. Check current specs, weights,
> prices and export classification with vendors before buying (see the README's
> Hardware and Safety sections).

---

## 1. Summary

| | Decision |
|---|---|
| Robot | Unitree B2-class quadruped: one dock in the first build, deck rails for a second, 3–4 drones from Phase 5 |
| Form factor | 8-inch long-range FPV-style quad, ~350 mm motor-to-motor, ~550 mm across the prop tips |
| All-up mass | ~2.15 kg (budget in §4, from `tools/drone_budget.py`) |
| Endurance | ~25 min hover with a 20 % reserve on two 6S1P 21700 Li-ion packs (97 Wh each) in parallel. That is ~10 min of margin over P-2 (15 min) |
| Flight controller | PX4 on an FPV-format H7 board (same PX4 + uXRCE-DDS link as today's sim) |
| Companion | NVIDIA Jetson Orin NX 16 GB (LIO, VIO, detection, swarm estimator) |
| Sensors | 3D lidar (Livox Mid-360 class), 3 IMUs (FC, lidar, depth camera), forward stereo depth camera, 12 MP video camera, downward global-shutter dock camera, flow + ToF rangefinder, GNSS + compass, Lepton 3.5 thermal camera |
| Radios | Wi-Fi mesh (2x2 MIMO) for data, UWB for ranging and a backup data path, ELRS for the safety pilot |
| Dock interface | A cone-shaped docking collar round the belly: self-centring, works at any yaw, and has two coaxial charge rings. The steel strike ring is on the drone and the electro-permanent magnet (EPM) latch is on the dock |
| GPS-denied swarm positioning | Each drone's own lidar-inertial odometry, plus UWB ranges to 4 anchors on the robot and between drones, plus inter-drone map loop closures. All of it is fused in the **robot's map frame**, and the dock gives the starting pose |

**Scope note:** The v1 requirements doc (§7, §8) says to land only when the robot is
stationary. This design is built so that **moving-dock landing can be added later**
(the collar and funnel geometry, robot velocity feed-forward, and the relative-pose
sensors). The v1 software should use the stationary path. §6.3 is the Phase 4+ procedure.

---

## 2. Requirements this design answers

| Req | How |
|---|---|
| F-1 no GPS | Navigation never depends on GNSS. GNSS is one optional factor (§7) |
| F-2 shared map | Every drone works in the robot's map frame from takeoff. Submaps and loop closures go to the robot (§7.5) |
| F-3 autonomous launch/land/charge | Passive capture funnel, EPM latch, collar charge contacts, tag + UWB guidance (§6) |
| F-6 lost link | UWB carries a small pose/state payload when Wi-Fi drops, and a return-to-robot behaviour runs on local LIO (§7.7) |
| P-2 15 min sortie | §4 budget: ~25 min modelled, so there is margin for cold derating and pack ageing |
| P-4 0.5 m map accuracy | LIO locally (cm level). Inter-drone consistency comes from UWB plus loop closures. The sim gives ~0.3–0.6 m before loop closures (§7.8) |
| S-2 safety pilot | ELRS receiver wired directly to the FC. It overrides offboard control |
| S-3 failsafes | Degradation ladder (§7.7) |

---

## 3. Airframe and layout

### 3.1 Why 8 inch

A 360° 3D lidar (~265 g) plus an Orin-class computer (~120 g) is ~400 g of payload
before cameras and radios. A 5-inch quad can lift that, but it would fly for
5–7 minutes. A 7-inch frame gets ~16 minutes, which only just meets P-2. The B2 has
the payload to carry a bigger drone, so the design uses an **8-inch frame with two
battery packs**: ~25 minutes, still built the FPV way (stack-mounted FC/ESC, carbon
plates, standard motors). The 7-inch build stays in `drone_budget.py` as
`compact_7in` for comparison. If you want a 5-inch scout later, it would drop the 3D
lidar and Orin and work as a "camera + VIO" scout, using this same swarm
localization stack.

### 3.2 Layout

```
                 TOP VIEW (front = up)

        (M1)                         (M2)
          \\     [depth cam  ->]     //
           \\   [video+thermal->]  //
            \\  +--------------+   //
             \\ | GNSS mast    |  //        mesh antennas on the rear arms,
              \\| Orin NX + fan| //         vertical, away from the GNSS
               \|  FC / ESC    |/
                |  UWB (top)   |
               /| pack A|pack B|\           two 97 Wh packs side by side,
              //+--------------+ \\         slide in from the rear; CG set by them
             //                   \\
            //   ant         ant   \\
        (M4)                         (M3)


                 SIDE VIEW

     GNSS puck ---+         UWB antenna (top, clear view all round)
                  |           |
   ===prop====[ top plate: Orin, FC/ESC stack ]====prop===
   depth/video/thermal>[ battery tray (2 packs) ]
                   [  belly: down cam, flow/ToF ]
                         \   Mid-360    /      <- lidar mounted upside down,
                          \  (inverted) /         FOV -52..+7 deg (sees ground
                           \___________/          and the horizon)
                        docking collar (cone)  <- lowest point; lidar sits
                        with 2 coaxial charge     inside the collar's ring
                        rings + steel strike ring
```

**Lidar mounting.** The Mid-360's vertical field of view is −7° to +52°. If you mount
it upright on top, it mostly sees sky and canopy. Mounting it **upside down under
the belly** makes it −52° to +7°: the ground, trunk bases, rubble and the horizon,
which is what SAR mapping and obstacle avoidance need. The forward depth camera covers
the band above +7°. The lidar's built-in IMU is what FAST-LIO2 / Point-LIO use.

**Design check for CAD:** the docking collar must stay outside the lidar's −52°
beam cone. With a 32 mm lidar radius and a collar inner radius of 70 mm, the
beam reaches the collar radius 48 mm below the window, so the collar lip must sit
less than ~40 mm below the window bottom. Confirm this against the real window
geometry before cutting parts.

**Vibration.** Put the lidar and the depth camera on soft-mounted sub-plates. LIO and
VIO break down quickly with prop vibration at the IMUs. Plan a notch-filter tuning
pass and log raw IMU data at 1 kHz on the bench.

### 3.3 Electrical and data

```
 pack A (6S1P) ─┐
                ├─ ideal-diode OR-ing (no cross-charging; either pack alone can fly)
 pack B (6S1P) ─┘
   6S bus ──┬── 4-in-1 ESC ── motors
            ├── Mid-360 (9–27 V input, runs direct from 6S)
            ├── 12 V buck ── Orin NX carrier ── 5 V USB rail
            └── 5 V BEC ── FC, GNSS, RC rx, UWB, cameras
            (current sensor -> FC; per-pack voltage to FC; ideal diode + fuse
             on the charge input, which charges both packs through their own BMS)

 Orin NX
  ├─ Ethernet ── Mid-360 (point cloud + IMU, PTP time sync)
  ├─ Ethernet/USB ── mesh radio
  ├─ USB 3 ── stereo depth camera
  ├─ CSI ── 12 MP video camera; CSI ── downward camera
  ├─ SPI + I2C ── Lepton 3.5 thermal (or USB via a PureThermal board)
  ├─ SPI/UART ── UWB module
  └─ UART/Eth ── FC  (uXRCE-DDS: /fmu/in/vehicle_visual_odometry, offboard setpoints)

 FC: IMU (ICM-42688-P class), baro, GNSS+compass, ELRS rx, flow+ToF, ESC telemetry
```

Time sync matters for fusion. The Orin is the PTP grandmaster for the lidar, and the
FC is synced over uXRCE-DDS timesync. Across the swarm, sync time over the mesh, and
use UWB timestamps as a cross-check.

---

## 4. Mass, power and endurance

Run `python3 tools/drone_budget.py`. Estimated results (momentum theory, figure of
merit 0.62, drivetrain 80 %, sea level, landing at 20 % reserve):

| Variant | Mass | Avionics | Total power | Pack | Hover endurance |
|---|---|---|---|---|---|
| **baseline**: 8 in, two 6S1P packs, Orin NX, depth + thermal cameras | 2153 g | 30.5 W | 378 W | 2 × 97 Wh | **24.7 min** |
| compact_7in: the earlier 7 in proposal, one pack | 1609 g | 30 W | 287 W | 97 Wh | 16.3 min |
| light_7in: 7 in, VOXL 2-class SoC, no separate depth camera | 1449 g | 22 W | 241 W | 97 Wh | 19.4 min |

- **Why not P50B cells.** 5.0 Ah cells would add ~4 min, but each pack would then hold
  108 Wh, over the 100 Wh airline and shipping limit that the two-pack layout exists
  to stay under. Use 4.5 Ah cells (Molicel P45B class), or choose 5.0 Ah cells and
  accept the airline-approval paperwork (100–160 Wh) instead.
- **Two packs in parallel.** Each pack has its own BMS and connector, and they are
  joined through ideal diodes so neither charges the other. If one pack faults, the
  other carries the drone home at reduced endurance. It also allows a
  one-pack-at-a-time swap in Phase 5 if turnaround becomes the bottleneck.
- The Orin NX is ~45 % of the avionics power. Run it in a 15 W mode.
- Li-ion (21700) beats LiPo here: the hover draw is ~17.5 A total, ~9 A per pack,
  well inside the cells' continuous rating, and the energy density is ~40 % higher.
  Below ~5 °C the pack sags, so derate endurance by 15–25 % in cold weather (still
  ~19 min).
- Hover thrust per motor is ~540 g. A 2810 motor on 8-inch props gives ~3.5–4:1
  thrust-to-weight, which is enough to fight gusts and to climb hard when aborting a
  dock approach.
- **Robot payload:** a B2-class robot carries the dock, charger and 3–4 drones
  (~2.15 kg each) with room left over. A Go2 would be marginal even for one 8-inch
  drone plus dock.

---

## 5. Sensors and radios (candidate parts)

| Function | Candidate | Why | Notes |
|---|---|---|---|
| Flight controller | ARK FPV or Holybro Kakute H7 v2 (PX4) | FPV stack format, PX4 support, good IMU | Keep the offboard flow the repo already uses |
| IMU #1 | FC IMU (ICM-42688-P class) | PX4 EKF2 attitude | Soft-mounted |
| 3D lidar + IMU #2 | Livox Mid-360 | 360° x 59°, 0.1 m minimum range, ~265 g, ~6.5 W, built-in IMU, Ethernet | Unitree 4D L1 is a lighter, cheaper option with a sparser scan |
| Depth + IMU #3 | Intel RealSense D435i, or Luxonis OAK-D Pro W | Forward obstacle depth from 0.2 m. Its IR stereo pair also feeds VIO fallback | Pick global shutter for VIO |
| Video | 12 MP CSI module (IMX477 class), or IMX296 global shutter | Operator imagery and survivor detection crops | Stream low-res live; store full-res keyframes |
| Thermal (SAR) | Teledyne FLIR Lepton 3.5 (160x120, ~5 g, ~$250) | Finds warm bodies in canopy shade or partly covered, where RGB misses them | Short range: plan thermal passes at lower altitude. Boson 320 is the upgrade if range matters and budget allows |
| Dock camera | OV9281 global-shutter module, down-facing | AprilTag pose on final approach | Rear belly, clear of the lidar |
| Flow + range | ARK Flow (PAW3902 + AFBR-S50) | Low-altitude height/velocity, and a last-ditch navigation source | Feeds PX4 EKF2 directly |
| GNSS | u-blox M10 + compass | Geo-referencing when available. Spoofing check against LIO | Mast-mounted, away from radios |
| Wi-Fi mesh | Prototype: 802.11s / batman-adv on an M.2 Wi-Fi card. Field: Doodle Labs Mesh Rider (Mini-OEM) or similar | Self-healing multi-hop between drones and the robot, IP transport | Robot carries a higher-power node and the backhaul |
| UWB | Qorvo DW3000-based module (e.g. DWM3000), or Nooploop LinkTrack | 10 cm-class ranges between every pair, at up to tens of Hz. Small data payload in ranging frames | Channel 5/9. Robot has 4 anchors |
| Safety pilot | ExpressLRS receiver | Hard override, independent of the mesh | S-2 |

---

## 6. Docking on the quadruped

### 6.1 Dock (robot side)

```
            dock, top view                         section A-A

        tags   ┌─────────────────┐          mouth Ø ~500 mm
         ▣     │   ╲         ╱   │   ▣       ╲                 ╱   25–30° walls,
               │    ╲ ┌───┐ ╱    │            ╲    ┌─────┐    ╱    low-friction
               │     ╲│ ● │╱     │             ╲   │ lid │   ╱     coating
         ▣     │      └───┘      │   ▣          ╲__│ cav │__╱
               └─────────────────┘                 └─────┘ <- lidar cavity
          UWB anchors at 4 corners,              ▲ pogo charge rings (+ inner, − outer)
          on short masts, mixed heights          ▲ EPM latch ring (holds the steel ring)
                                                  spring-damped plate on the robot's back
```

- **Passive capture.** A shallow funnel (~500 mm mouth, 25–30° walls) turns a
  ±150 mm landing error into a centred, seated drone. The collar is circular, so
  **yaw does not matter**. That cuts out a whole alignment step and makes docking on a
  moving robot practical later.
- **Latch.** An electro-permanent magnet on the dock grips the steel ring in the
  collar. EPMs only draw power for the pulse that switches them on or off, so a robot
  power loss keeps drones latched. Putting the EPM on the dock keeps its weight off
  the drone. Size it to hold more than 5 g of shock load (quadruped stumble):
  2.15 kg × 5 g ≈ 105 N, so target ≥ 200 N.
- **Charging.** Two coaxial copper rings on the collar's underside meet spring pogo
  rings in the dock. Polarity is fixed by the geometry, whatever the yaw. The charger
  stays off until the drone reports *latched* and passes a handshake. The drone has
  an ideal-diode input and a fuse. The dock has a temperature sensor and a smoke
  sensor (README battery-safety rule).
- **Charge rate.** Contact charging at ~1C: ~9 A at 25.2 V, ~230 W per drone, about
  60–75 min from the return threshold to full (CC-CV). One sortie per drone per
  60-minute mission is accepted for v1. The charger and the robot's power tap are
  sized for **two docks** (~2 × 250 W). The energy manager (F-7) counts recharging
  against the robot's own range. Battery swapping is deferred to Phase 5.
- **Deck layout.** One dock in the first build. The deck plate has rails for a second
  dock front-to-back. The 3–4 drone stacked rack is designed in Phase 5.
- **Fiducials.** An AprilTag bundle round the rim (large tags for acquisition at
  2–4 m) and a small nested tag on the lid for the last 0.5 m. Add IR LEDs for
  low light.
- **UWB anchors.** 4 anchors on short masts at the dock corners, at **different
  heights** (coplanar anchors leave the drone's height ambiguous). The dock plate's
  extrinsic in the robot frame is calibrated once.
- **Suspension.** A spring-damped dock plate soaks up the gait bounce (a trot moves
  the body by a few cm at ~2 Hz). Add an active 2-axis levelling plate only if tests
  show it is needed.

### 6.2 Stationary landing (v1)

`RETURN → APPROACH (UWB, robot frame) → ALIGN at 1.5 m (tags + UWB) → DESCEND → CAPTURE (EPM on contact) → DOCKED`

### 6.3 Moving-robot landing (Phase 4+, after the stationary landing is reliable)

1. **Robot enters docking gait.** Straight line, constant speed ≤ 0.5 m/s, no
   turns, fixed body height. It broadcasts its odometry (pose, twist) at 50 Hz
   over the mesh, with UWB as backup.
2. **Trail.** The drone holds 3 m behind and 2 m above the dock *in the robot
   frame*. The robot's velocity is fed forward into the PX4 setpoints, so the
   position loop only has to correct the small relative error.
3. **Align.** At 1.5 m above the dock, the relative pose comes from tags + UWB
   anchors + the robot's odometry, fused at ≥ 30 Hz.
4. **Descend.** Descend relative to the dock at 0.3–0.5 m/s. Gate: lateral error
   < 0.1 m and relative velocity < 0.15 m/s, sustained for 0.5 s.
5. **Commit.** In the last ~0.15 m, descend fast and drop thrust on contact. The
   funnel does the final centring. The EPM fires when the contact switch closes.
6. **Abort → back to Trail.** Abort on: tag lost > 0.3 s, error over the gate, robot
   turning or stumbling (from the robot IMU), link loss, or low battery (in which case
   go to a stationary landing instead).

Downwash: keep the robot's lidar and cameras clear of the funnel area, or blank
them during docking. Expect up to ~21 N of downwash (the drone's weight) on the dock plate in the last metre.

---

## 7. Knowing where everyone is when GPS goes out

### 7.1 The key idea: never depend on GPS for the shared frame

All drones and the robot share one frame, the **robot's map frame**, which comes from
the robot's own lidar-inertial SLAM. Every drone starts **latched to the dock**, so at
takeoff its pose in that frame is known exactly from the dock extrinsic. With a shared
origin from the start, the localization problem shrinks: instead of having to *find*
each other, the drones only have to keep their drift in check. GPS, when there is
any, adds a geo-reference for the operator's map. It is not the backbone.

### 7.2 Layers (each one works if the layers above it fail)

| # | Layer | What it gives | Rate | Fails when |
|---|---|---|---|---|
| 1 | Own lidar-inertial odometry (FAST-LIO2 / Point-LIO, Mid-360 + its IMU) | Smooth local pose. Drift ~0.5–2 % of distance in forest | 10–50 Hz, IMU 200 Hz | Open fields, fog, smoke (geometric degeneracy) |
| 1b | VIO fallback (OpenVINS on the depth camera's IR stereo + IMU) | Same, from vision | 30 Hz | Darkness, motion blur, textureless scenes |
| 1c | PX4 EKF2 flow + ToF + baro | Velocity and height for a controlled return or landing | 50 Hz | High altitude, water |
| 2 | **UWB ranges to the robot's 4 anchors** | Absolute fix in the robot frame. Bounds drift no matter how long the sortie runs | 5–20 Hz per link | Beyond ~50–100 m, or blocked by large trunks or terrain |
| 3 | **UWB ranges drone ↔ drone** | Ties the drones' estimates to each other. Relays position information from a drone near the robot to one that is out of anchor range | 5–20 Hz per pair | Drones far apart |
| 4 | **Inter-drone and drone↔robot loop closures** (lidar Scan Context descriptors, camera NetVLAD/DBoW) | cm-level relative pose wherever two agents saw the same place | Event-driven | No overlap yet |
| 5 | GNSS factor (when healthy) | Geo-reference. Slowly corrects global heading and scale | 1–10 Hz | Canopy, jamming, spoofing |

### 7.3 Estimator

- **On each drone (fast loop):** LIO/VIO odometry goes to PX4 as external vision
  (`/fmu/in/vehicle_visual_odometry`). This keeps flight control smooth and local.
- **On the robot (swarm loop):** a sliding-window factor graph (GTSAM / iSAM2)
  over all agents' poses in the robot map frame. Its factors are: odometry between
  keyframes from each agent, UWB range factors (anchor↔drone, drone↔drone) with
  Huber/Cauchy robust loss for NLOS, loop-closure factors from layer 4, and GNSS
  factors with a spoofing gate. The robot sends each drone its **map→odom correction**
  (a single transform) at 1–5 Hz. The drone applies it to its setpoints and does not
  feed it back into the flight EKF, so a correction can never cause a jump in the
  flight controller.
- **If the robot is out of reach:** each drone runs a reduced copy of the graph (just
  itself and its UWB neighbours), in the style of D²SLAM. The drones carry on
  peer-to-peer and hand their factors back to the robot when the link returns.
- **Yaw.** Range-only measurements say nothing about yaw. Yaw is constrained by
  motion during ranging (the sortie patterns naturally have it), by loop closures,
  and by the dock at takeoff. LIO yaw drift is also small, because gravity fixes roll
  and pitch.

### 7.4 What each drone broadcasts (keeps the swarm organized)

`SwarmState` at 10 Hz over the mesh, ~120 bytes:
`drone_id, stamp, pose (robot frame), 6x6 covariance (compressed), velocity,
battery %, lifecycle state, task id, health flags (LIO ok, VIO ok, UWB ok, GNSS ok)`.

If the mesh drops, a 32-byte version (id, x/y/z as cm int16, σ, state, battery)
rides in the **UWB ranging frames**. Ranging keeps working without Wi-Fi, so every
drone still knows roughly where its neighbours are and what they are doing.

Over the mesh, carry ROS 2 traffic on **Zenoh** (rmw_zenoh or zenoh-bridge-ros2dds)
rather than default DDS multicast discovery, which floods lossy multi-hop links.

### 7.5 Keeping the swarm organized for SLAM

- **Tasking in the robot frame.** The robot divides the search area into sectors or
  frontier cells in its map frame and assigns them by auction (Phase 5). Because the
  sectors live in the shared frame, they stay consistent when GPS is lost.
- **Covariance-aware separation.** Collision keep-out distance = base distance +
  3σ of each drone's relative position. When uncertainty grows, the drones spread out
  automatically.
- **Uncertainty leash.** If a drone's position σ exceeds 2 m, or it has gone more
  than 30 s without an anchor range, it stops exploring and climbs to regain line of
  sight to the robot. If that does not help, it flies back along its own LIO path:
  LIO is consistent locally even when it has drifted globally, so the drone can
  retrace its route to where it last had good fixes. Its last map segments are
  flagged *low confidence* until a loop closure re-anchors them.
- **Radio leash.** Sortie range is limited to the shorter of mesh range and UWB
  range from the robot, *or* from a "relay drone" parked in a high hover partway
  out. A relay drone holds still, has good UWB geometry to the robot, and works as a
  temporary anchor for the drones beyond it.
- **Submaps.** Drones send keyframed submaps (voxel-downsampled point cloud +
  descriptor + pose). The robot merges them with Phase 3 loop closures. Drones get
  back only the map tiles around their next task.

### 7.6 GPS loss detection and spoofing

Treat GNSS as unhealthy when: fix type < 3D, HDOP > 2.5, satellites < 8, *or* the
GNSS-vs-LIO innovation fails a chi-square test for more than 2 s (a check against
both jamming and spoofing). On recovery, re-enable the GNSS factor only after 10 s
of consistent innovations. The robot-frame estimate never jumps, only the geo-reference
updates.

### 7.7 Degradation ladder (S-3 failsafes)

| Condition | Response |
|---|---|
| GNSS lost | No change to flight. Map stays in the robot frame. Operator sees "geo-ref frozen" |
| Wi-Fi mesh lost | Pose and state continue over UWB. Drone finishes or aborts its current task under F-6 rules, and buffers its map for later upload |
| Anchor ranges lost > 30 s, or σ > 2 m | Uncertainty leash (§7.5) |
| LIO degenerate | Switch to VIO. If both are degenerate, use flow + ToF to return or land |
| All navigation sources lost | Hold attitude, land slowly with flow + ToF. Safety pilot alert |
| Battery at return threshold | Return to the robot's latest broadcast position. Plan the rendezvous with the robot's planned path |

### 7.8 Simulation check

`tools/swarm_uwb_sim.py` is a small numpy model that tests the geometry and
observability claims above: a robot walks at 0.5 m/s carrying 4 anchors, 4 drones
fly sector sweeps 20–60 m out, their odometry has a drifting velocity bias, GPS is
lost at 60 s, and the sortie runs 15 min. The ranges have 10 cm noise, 10 % NLOS
outliers and an 80 m cutoff. Three trials each, mean error after GPS loss:

| Estimator | Mean abs. error | p95 | Final | Drone-to-drone rel. error |
|---|---|---|---|---|
| Odometry only | 12.7 m | 26.7 m | 28.4 m | 18.7 m |
| + UWB to robot anchors | 0.68 m | 1.01 m | 0.74 m | 0.99 m |
| + UWB drone↔drone | **0.32 m** | **0.56 m** | **0.24 m** | **0.48 m** |

Sensitivity:
- 30 % NLOS: 0.38 m. The chi-square gate handles most outliers.
- UWB range 45 m: ~1.0 m, with drone↔drone ranges adding little. Drones in
  separate sectors are also far from each other, so this is the case for a relay
  drone or tighter sectors.

The model is a point mass with no attitude, and loop closures (layer 4) are not in it.
It shows that the geometry works. It does not show flight performance. Next step:
put the same estimator in the Gazebo multi-drone sim with a UWB range plugin.

---

## 8. Software stack (on the drone)

| Layer | Choice |
|---|---|
| Flight | PX4 v1.17 (as in the sim), EKF2 with external vision + flow + baro, GNSS optional |
| Link | uXRCE-DDS (same as `las_offboard` today) |
| Odometry | FAST-LIO2 or Point-LIO (lidar), OpenVINS (fallback) |
| Swarm estimator | GTSAM sliding window: UWB + odometry + loop factors (robot-hosted, drone-local fallback) |
| Collaborative SLAM | Swarm-SLAM or Kimera-Multi-style descriptors and loop closures |
| Perception | Person / thermal detection on the Orin (TensorRT). Detections are always suggestions (S-1) |
| Comms | Mesh (802.11s/batman-adv or the radio's own mesh), Zenoh for ROS 2, store-and-forward buffer |
| Mission | Lifecycle state machine from the README (`DOCKED → … → DOCKED`) |

---

## 9. Path into the existing roadmap

| Phase | Drone-design work |
|---|---|
| 1 (now) | Replace `x500_lidar_2d` with a custom 8-inch SDF (two-pack mass and inertia from §4) carrying a Mid-360-pattern `gpu_lidar` mounted upside down plus depth and thermal cameras. Add a UWB range plugin (noise, NLOS, cutoff as in the numpy sim). Dock model with funnel + tags |
| 2 | Build one drone. Bench-measure mass and power and update `drone_budget.py`. Tethered tests, then stationary dock landings (20 in a row) |
| 3 | Robot-frame shared map: the dock as the shared origin, then UWB + loop-closure factors |
| 4 | Moving-dock landing (§6.3) behind a feature flag |
| 5 | 2–4 drones, `SwarmState`, covariance-aware separation, auction tasking |
| 6 | GNSS loss/spoof injection, mesh loss with UWB fallback, NLOS-heavy forest tests |

## 10. Risks and open questions

1. **Endurance is modelled, not measured** (~25 min, ±15 %). Bench-test thrust and
   power early and update `drone_budget.py`.
2. **Turnaround.** At 1C a drone is out of action for over an hour after each sortie.
   Fine for one drone in v1; revisit (pack swap or 2C cells) in Phase 5.
3. **8-inch near trees.** A ~550 mm drone needs wider gaps than a 7-inch. Set the
   avoidance clearance from the real prop-tip radius in the sim.
4. **Lidar under the belly.** The collar clearance and the lidar window's exposure to
   dirt need a CAD check and a guard.
5. **UWB in dense forest.** Range and NLOS rates are guesses. Do a range survey at
   the test site (trunk density matters most).
6. **Moving-dock landing is outside the v1 scope** (§11, D2). It needs a requirements
   change before Phase 4 work.
7. **Export control.** GPS-denied swarm navigation and anti-jam comms may be
   controlled technology. Follow the README's guidance before sharing.

## 11. Decisions log

| # | Decision (2026-11-04) | Alternatives considered |
|---|---|---|
| D1 | Unitree B2-class robot | Go2 (payload too marginal), Spot/ANYmal (cost) |
| D2 | Land while stationary in v1; moving landing in Phase 4+ | Moving landing in v1; stationary only, ever |
| D3 | ~$4–5k per drone: Mid-360, Orin NX, field-grade mesh radio, onboard detection | ~$2–2.5k build (L1 lidar, VOXL-class, 802.11s) |
| D4 | 8-inch frame, ~25 min hover | 7-inch (~16 min, or ~18 min with P50B cells) |
| D5 | Lepton 3.5 thermal camera in v1 | Boson 320 (over budget); RGB only |
| D6 | Two 6S1P packs (97 Wh each) in parallel | One 6S2P pack (dangerous-goods shipping, single point of failure) |
| D7 | One dock now, deck rails and charger sized for two | Two docks now; 3–4 drone rack now |
| D8 | Contact charging at ~1C; swapping deferred to Phase 5 | Swap one pack; 2C fast charge |

## References

- FAST-LIO2 (Xu et al., 2022); Point-LIO (He et al., 2023)
- D²SLAM / Omni-swarm (Xu et al., HKUST): decentralized UWB + VIO swarm state estimation
- Swarm-SLAM (Lajoie & Beltrame, 2024); Kimera-Multi (Tian et al., 2022)
- Scan Context (Kim & Kim, 2018) for lidar place recognition
- UVDAR (CTU-MRS): blinking-UV-LED relative localization (optional visual layer)
- PX4 external-vision fusion docs; Livox Mid-360 datasheet; Qorvo DW3000 datasheet
