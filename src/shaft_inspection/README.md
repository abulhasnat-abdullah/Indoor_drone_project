# shaft_inspection

Autonomous inspection of **vertical shafts** (mine shafts, ventilation shafts,
wells) with a GPS-denied PX4 multicopter. The drone descends a shaft, stays
centred without touching the walls, turns around just above the floor, climbs
back out, and builds a 3D map and a radius-vs-depth profile of the shaft — all
from a **360° 2D lidar** and a **downward ToF rangefinder/optical-flow sensor**.
No GPS, no SLAM, no telemetry link needed during the inspection.

This README explains the whole system and the simulation. For building and
flying the real vehicle, see **[docs/HARDWARE_GUIDE.md](docs/HARDWARE_GUIDE.md)**.

---

## Contents

1. [Status — what is and is not proven](#1-status--what-is-and-is-not-proven)
2. [The idea in one page](#2-the-idea-in-one-page)
3. [Hardware it is designed for](#3-hardware-it-is-designed-for)
4. [System architecture](#4-system-architecture)
5. [How each part works](#5-how-each-part-works)
6. [Running the simulation](#6-running-the-simulation)
7. [Flying a mission in the simulation](#7-flying-a-mission-in-the-simulation)
8. [The web dashboard](#8-the-web-dashboard)
9. [Outputs](#9-outputs)
10. [Parameter reference](#10-parameter-reference)
11. [Troubleshooting](#11-troubleshooting)
12. [Known limitations](#12-known-limitations)
13. [Repository map](#13-repository-map)

---

## 1. Status — what is and is not proven

| Item | Status |
|---|---|
| Fully autonomous mission in sim (`auto_launch`): takeoff → centre → descend 24.8 m → turn around 0.5 m above floor → climb → return to launch → disarm | **Flown end-to-end**, no wall contact |
| Bore-centre estimator on circular / rectangular / elliptical / D-shaped / rough bores | **Tested**: 1–5 cm consistency against the world geometry, ~3 ms per scan |
| Pilot handover mode (`pilot_handover`): fly manually, switch to Offboard, mission runs, pilot takes back | Implemented; handover gating verified; **full handover flight not yet confirmed end-to-end** |
| Real hardware | **Not flown.** Parameters and messages checked against PX4 v1.14.4 source; see the hardware guide for the staged test plan |

---

## 2. The idea in one page

```
      pilot flies over the mouth (Position mode, flow-aided)
                    │  switch to OFFBOARD
                    ▼
   ┌──────────── collar ────────────┐
   │   CENTER  : servo to bore axis │
   │   DESCEND : sink, stay centred │   lidar (360°, horizontal) sees the walls
   │     │                          │   → bore centre → horizontal position
   │     ▼                          │
   │   (brake below 2.5 m)          │
   │   turn around 0.5 m above floor│   ToF (downward) sees the floor
   │     │                          │
   │   ASCEND : climb, stay centred │
   │   EXIT   : back at handover    │
   └────────────────────────────────┘
                    │  pilot moves the sticks → Position mode
                    ▼
            pilot flies home and lands
```

**Why no SLAM.** A vertical bore gives almost no geometric information along
its own axis. 2D scan matching (rf2o, slam_toolbox) cannot see vertical motion
and misreads changes in the wall shape as sideways motion, which would push the
vehicle into the wall; 3D lidar-inertial odometry degenerates along the axis
for the same reason. Instead the problem is split:

| Problem | Solved by |
|---|---|
| Horizontal position | The **bore centre**, measured directly from every lidar scan — an absolute reference that does not drift — sent to PX4's EKF2 as external vision |
| Depth / height | EKF2 height (barometer), with the ToF used for terrain |
| Finding the floor | The ToF rangefinder |
| Not hitting walls | Centring + repulsion + velocity clamp, all from the lidar |
| 3D map | Stacking scans at known depth ("mapping with known pose") |

---

## 3. Hardware it is designed for

| Part | Model | Role |
|---|---|---|
| Flight controller | Holybro **Pixhawk 6C**, PX4 **v1.14.x** | attitude, EKF2, position control, failsafes |
| Companion computer | **Raspberry Pi 5, 8 GB** | runs all ROS 2 nodes |
| 2D lidar | Slamtec **RPLIDAR C1** — 360°, 0.05–12 m, 10 Hz, ~500 pts/scan, ±30 mm | walls, bore centre, obstacles, map |
| Downward sensor | Holybro **H-Flow** (DroneCAN) — PAA3905 flow (42° FOV, 7.4 rad/s) + Broadcom AFBR-S50 ToF (0.08–30 m typical) | floor distance, position hold near a surface |
| Airframe | x500-class quadrotor (~0.5 m tip-to-tip) | |

The simulation models exactly these sensors (see §6.2).

---

## 4. System architecture

### 4.1 Nodes

```
 RPLIDAR C1 ──/scan──► shaft_perception ──/fmu/in/vehicle_visual_odometry──► PX4 EKF2
                           │  /shaft/offset, /shaft/clearance,                  ▲
                           │  /shaft/sector_min, /shaft/repulsion, /shaft/valid │
                           ▼                                                    │
 PX4 ──status, local position, attitude──► shaft_mission ──offboard_control_mode, trajectory_setpoint,
                                               │            vehicle_command──────► PX4
                                               │  /shaft/state, /shaft/depth, /shaft/cmd_vel_ned
                                               ▼
                    shaft_mapper ──► ~/shaft_maps/*.pcd, *.csv, /shaft/cloud
                    shaft_dashboard ──► http://<host>:8080  (read-only)
```

| Node | Job |
|---|---|
| `shaft_perception` | de-tilts each scan, finds the bore centre, computes clearances and repulsion, publishes the horizontal position fix to EKF2 |
| `shaft_mission` | state machine + control + obstacle avoidance; the only node that commands PX4 |
| `shaft_mapper` | accumulates the 3D point cloud and the depth profile |
| `shaft_dashboard` | web dashboard; subscribes only, never commands |
| `shaft_preflight` | go / no-go check (CLI tool) |
| `shaft_mount_check` | measures the lidar's mounting yaw offset (CLI tool) |

### 4.2 Topics

| Topic | Type | From → to |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | lidar driver (sim: Gazebo bridge) → perception, mapper, dashboard |
| `/down_range` | `sensor_msgs/LaserScan` (1 beam) | **sim only** Gazebo bridge → mission (`range_source: laserscan`) |
| `/shaft/offset` | `geometry_msgs/Vector3Stamped` | vehicle displacement **from** the bore centre, level body frame (x fwd, y left) |
| `/shaft/clearance` | `std_msgs/Float32` | radius of the largest wall-free circle at the centre, m |
| `/shaft/valid` | `std_msgs/Bool` | bore fix usable this scan |
| `/shaft/roundness`, `/shaft/radius` | `Float32` | shape descriptors (radius only when the section is round) |
| `/shaft/sector_min` | `Float32MultiArray` | nearest return in 36 × 10° sectors, level body frame |
| `/shaft/repulsion` | `Vector3Stamped` | push-away velocity from close returns |
| `/shaft/state` | `std_msgs/String` | mission state name |
| `/shaft/depth` | `Float32` | depth below the handover / datum point, m |
| `/shaft/cmd_vel_ned` | `Vector3Stamped` | the velocity the mission is commanding (also in dry run) |
| `/shaft/cloud` | `sensor_msgs/PointCloud2` | accumulated map, frame `shaft` |
| `/fmu/in/vehicle_visual_odometry` | `px4_msgs/VehicleOdometry` | bore-centre fix → EKF2 |
| `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command` | px4_msgs | mission → PX4 |
| `/fmu/out/vehicle_status`, `vehicle_local_position`, `vehicle_attitude`, `manual_control_setpoint`, `failsafe_flags` | px4_msgs | PX4 → nodes |

PX4 newer than v1.14 appends a version to some topic names
(`/fmu/out/vehicle_status_v4`, `vehicle_local_position_v1`). All nodes resolve
the live name at startup (`px4_topics.resolve`), so the same code runs against
v1.14 (unversioned) and newer PX4.

---

## 5. How each part works

### 5.1 Perception — finding the bore centre

For every scan (`shaft_perception_node.py`, `scan_geometry.py`):

1. **Mounting correction.** `lidar_yaw_offset_deg` rotates the scan so 0° is
   the vehicle's nose; `lidar_upside_down` mirrors it. Wrong values rotate
   every centring command and the vehicle spirals into the wall.
2. **Self filter.** Returns closer than `self_filter_radius` (0.25 m) are the
   airframe itself and are dropped.
3. **De-tilt.** Each beam is rotated by the vehicle's roll and pitch into a
   level frame. When the vehicle tilts, the scan plane tilts, a round shaft
   images as an ellipse, and its apparent centre moves — without de-tilting the
   controller chases a phantom offset. (PX4's roll/pitch are FRD, the scan is
   FLU; pitch changes sign between them.)
4. **Coverage gate.** The scan is binned into 180 bearings. If fewer than 85 %
   have a return, the fix is **refused** rather than guessed; a large gap would
   bias the answer towards "you are centred".
5. **Centre = point of maximum clearance.** The scanned free space is
   rasterised (4 cm cells) and a Euclidean distance transform gives each cell's
   distance to the nearest wall. The maximum is the safest point — correct for
   round, square, D-shaped or ragged bores. Two details matter:
   * candidates must be **inside** the scan polygon (a point beyond the wall is
     also "far from the wall");
   * for elongated bores the maximum is a line, not a point (every point on a
     rectangle's long centreline ties), so near-ties are broken by the
     cross-section's **area centroid**.
6. **Outputs.** The vehicle's offset from that centre, the clearance, per-sector
   nearest returns, a repulsion vector, and a roundness score (a circle fit is
   reported only when the section is actually round).

### 5.2 Position fix into PX4

The offset from the bore centre is rotated into NED with the current heading,
added to a shaft-axis location anchored once at startup, and published as
`VehicleOdometry` with **horizontal position only** (`EKF2_EV_CTRL = 1`):
position z is set to EKF2's own height (EKF2 drops the entire sample if any
component is NaN), attitude is NaN (a symmetric bore gives no heading).
Timestamps use the **system clock**, not the ROS clock: the uXRCE-DDS client
time-syncs against the host clock, and sim-time stamps make PX4 reject every
sample while the topics look connected.

The fix is absolute (it does not drift), but only as absolute as the bore is
prismatic — where the cross-section changes shape with depth, the centre moves.
Its variance is set conservatively (0.09 m²).

### 5.3 Height, terrain and the collar edge

**The core distinction.** A downward rangefinder does not measure the vehicle's
*height*; it measures the distance to *whatever is underneath*. Those are the
same only over flat ground. EKF2 therefore keeps two separate estimates:

```
   vehicle height  h   (above a fixed reference: where EKF2 started)
   terrain height  t   (height of the ground under the vehicle, same reference)
   rangefinder reads   r = h − t
```

| Quantity | Source | Used for |
|---|---|---|
| vehicle height `h` | EKF2: IMU + **barometer** reference (`EKF2_HGT_REF=0`) | holding height, depth, "back at the top" |
| terrain height `t` | EKF2 from the ToF (`EKF2_RNG_CTRL=1`, conditional) | flow scaling, low-altitude aid, land detection |
| raw floor distance `r` | ToF | the turnaround decision only |

**What happens at the collar edge.** Hovering 1 m above the ledge, `r = 1 m`.
Fly sideways over the shaft and the next sample reads `r = 26 m`. EKF2 has to
explain the jump: either the vehicle climbed 25 m in 50 ms, or the ground
dropped away. The IMU saw no 25 m climb and the barometer saw no pressure
change, so the measurement is rejected as a height change; EKF2 **resets the
terrain estimate** (`t` drops by 25 m) and `h` stays where it was. The position
controller holds `h`, so the vehicle simply keeps flying level.

If the ToF were the height **reference** (`EKF2_HGT_REF=2`), EKF2 would force
`h` to follow `r`: the vehicle would believe it had jumped to 26 m and dive to
"get back down". That is why the rangefinder is never the height reference here.

`EKF2_RNG_CTRL=1` (conditional) lets the ToF *help* height only while the
vehicle is slow (< `EKF2_RNG_A_VMAX` 1 m/s) and low (< `EKF2_RNG_A_HMAX` 3 m) —
e.g. on takeoff and landing, where rotor wash disturbs the barometer. Over the
shaft `r` is far above 3 m, so that aid switches itself off.

**So what holds height, moment to moment?** The IMU accelerometer, integrated
by EKF2 — it is smooth and fast but drifts. The barometer corrects that drift
continuously. Neither cares what is under the vehicle.

**Is the barometer usable?**

| Situation | Barometer | Consequence |
|---|---|---|
| Hovering, still air | good, ~0.1–0.3 m noise | fine |
| Close to the ground / walls | disturbed by rotor wash (ground effect; the sign of the error depends on the airframe) | ToF conditional aid covers takeoff and landing |
| Inside a narrow shaft | wash recirculates off the walls → pressure disturbances of decimetres | the vehicle may bob; it is centred and slow, so this is tolerable |
| Descending 25 m | real, slowly changing pressure (~0.12 hPa per metre) | this is exactly what it measures well — depth |
| Draughts / ventilation in a shaft | slow pressure offsets | slow depth bias of decimetres; the ToF turnaround is unaffected |
| Temperature change underground | slow drift | same as above |

In short: the barometer is good at **slow, large** height changes (depth) and
poor at **fast, small** ones near surfaces — and the IMU plus the conditional
ToF aid fill in exactly where the barometer is weak. The one decision that must
be precise, turning around 0.5 m above the floor, uses the ToF directly, so a
barometer error of a few decimetres never puts the vehicle into the floor.

Depth is `h` relative to the handover point, so a floor beyond the ToF's range
is fine: the vehicle descends on the barometer until the floor comes into view.

**Mounting the barometer on the real vehicle:** cover it with open-cell foam and
keep it out of the propeller wash inside the frame — this matters more in a
shaft than in open air.

### 5.4 Obstacle avoidance — three layers

PX4's Collision Prevention (`CP_DIST`) only acts on **Position mode stick
input**; it does nothing for Offboard setpoints. It is enabled (protects the
pilot), and the mission adds its own layers:

| Layer | What it does | Catches |
|---|---|---|
| 1. Centring | PD servo to the bore centre (`kp_lateral`, `kd_lateral`) | keeps away from walls in the first place |
| 2. Repulsion | push away from any return within 1 m | ledges, pipes, cables the centre estimate ignores |
| 3. Velocity clamp | no commanded speed toward a sector that cannot be braked before `stop_dist` (0.65 m) | everything else; runs last |

Plus gates: descent only while centred (`center_tol` 0.12 m, with hysteresis)
and with ≥ `descent_min_clearance` (0.60 m); stale scans (> 0.5 s) stop lateral
motion and descent; a return inside `critical_dist` (0.45 m) aborts.
All distances are from the **lidar centre** — an x500's prop tips reach
~0.38 m.

### 5.5 Mission state machine

```
auto_launch (sim):
WAIT → ARM → TAKEOFF → CENTER → DESCEND → ASCEND → EXIT → RETURN → TOUCHDOWN → LAND → DONE

pilot_handover (real vehicle, and sim manual testing):
WAIT ──(pilot armed, in OFFBOARD, bore fix valid)──► CENTER → DESCEND → ASCEND → EXIT → DONE
                                                       ▲                                   │
                            any state: pilot leaves OFFBOARD (sticks / switch) ──► DONE ◄──┘
```

| State | Behaviour |
|---|---|
| `WAIT` | streams zero-velocity setpoints (lets PX4 accept Offboard); `pilot_handover` never arms |
| `CENTER` | centres; holds 1.5 s before going on; records the depth datum |
| `DESCEND` | sinks at 0.35 m/s while gated; brakes from 2.5 m above the floor; **turns around at 0.5 m** — no landing in the shaft (dust, debris, props) |
| `ASCEND` | climbs at 0.5 m/s, still centring |
| `EXIT` | climbs back to the datum height; `pilot_handover` → `DONE` (hovers for the pilot) |
| `RETURN` / `TOUCHDOWN` / `LAND` | `auto_launch` only: fly to the recorded launch spot, descend, land and disarm there. **PX4 land mode is never used above the bore** — it descends straight down |
| `ABORT` | critical clearance, timeout or max depth: climb, then exit/return |

### 5.6 Mapping

Each scan is one horizontal slice at a known depth; the mapper stacks slices
(one per 5 cm of depth). Two products: the point cloud, and a **radius profile**
(clearance, mean/max radius, roundness vs depth). Heading drifts in a symmetric
bore, smearing the cloud azimuthally; the profile is unaffected, so treat it as
the metric deliverable.

---

## 6. Running the simulation

### Installation from this repository

Tested on Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, PX4-Autopilot main
(v1.18 alpha).

```bash
# 1. ROS workspace
mkdir -p ~/px4_ros2_ws/src && cd ~/px4_ros2_ws/src
git clone https://github.com/abulhasnat-abdullah/shaft_inspection.git
git clone https://github.com/PX4/px4_msgs.git            # match your PX4 version
cd ~/px4_ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
mkdir -p run_cmds && cp src/shaft_inspection/run_cmds/*.sh run_cmds/

# 2. PX4 additions (airframe, vehicle model, world)
S=~/px4_ros2_ws/src/shaft_inspection/px4
cd ~/PX4-Autopilot
cp $S/airframes/4023_gz_x500_shaft ROMFS/px4fmu_common/init.d-posix/airframes/
sed -i 's/^\t4022_gz_x500_gps_denied$/&\n\t4023_gz_x500_shaft/; t; s/^\t4001_gz_x500$/&\n\t4023_gz_x500_shaft/' \
    ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt   # or add the line by hand
cp -r $S/models/x500_shaft Tools/simulation/gz/models/
cp $S/worlds/vshaft.sdf Tools/simulation/gz/worlds/

# 3. Only if your PX4 main contains commit 162cce35d8 (SITL crash/freeze, see 6.3)
git apply $S/patches/revert-lockstep-162cce35d8.patch

make px4_sitl_default
```

Also needed: Micro-XRCE-DDS Agent (`MicroXRCEAgent` on the PATH),
`ros-jazzy-ros-gz-bridge`, `python3-scipy`, `pip install websockets`, and
QGroundControl at `~/Downloads/QGroundControl-x86_64.AppImage`.


### 6.0 First-time build (or after changing code/airframe/world)

```bash
cd ~/PX4-Autopilot && make px4_sitl_default                 # PX4 + airframe + world/model
cd ~/px4_ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select shaft_inspection --symlink-install
```

QGroundControl is expected at `~/Downloads/QGroundControl-x86_64.AppImage`.

### 6.1 One-command start

**Everything** — Gazebo GUI, QGroundControl, RViz, dashboard, manual handover:

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash   # only needed once per terminal
GUI=1 QGC=1 RVIZ=1 MODE=pilot_handover ./run_cmds/shaft_sim.sh
```

Then open **http://localhost:8080** for the dashboard. Startup takes about a
minute; you are ready when the dashboard shows all five links green and
*Mission: WAIT*.

Other variants:

```bash
./run_cmds/shaft_sim.sh                         # headless, fully autonomous mission
GUI=1 RVIZ=1 ./run_cmds/shaft_sim.sh            # watch the autonomous mission
GUI=1 QGC=1 RVIZ=1 GDB=1 MODE=pilot_handover ./run_cmds/shaft_sim.sh   # PX4 under gdb
```

**Stopping:** press **Ctrl-C** in the terminal running the script. That stops
PX4, Gazebo, RViz, the agent and all nodes; QGroundControl is left open on
purpose (close it normally from its window so joystick settings are saved).

If a run was killed uncleanly and processes are left over, the next launch
clears them automatically. To clear them by hand, run each line separately:

```bash
pkill -9 -f "px4_sitl_default/bin/px4"
pkill -9 -f "gz sim"
pkill -9 -x MicroXRCEAgent
pkill -9 -f "lib/ros_gz_bridge/parameter_bridge"
pkill -9 -f "lib/shaft_inspection/shaft_"
pkill -9 -x rviz2
```

(Avoid one-liners like `pkill -f shaft_sim` from a shell whose own command line
contains that text — `pkill -f` matches and kills the shell itself.)

| Variable | Default | Effect |
|---|---|---|
| `MODE` | `auto_launch` | `auto_launch` or `pilot_handover` |
| `GUI` | 0 | Gazebo GUI |
| `RVIZ` | 0 | RViz with the map and live scan |
| `QGC` | 0 | start QGroundControl (reused if already running, never killed) |
| `GDB` | 0 | run PX4 under gdb; backtrace lands in the log |
| `HEADLESS` | 1 | PX4's own Gazebo rendering |

The script clears stale sim processes, starts the XRCE agent, PX4 SITL
(`gz_x500_shaft`, world `vshaft`), optional GUIs, and the ROS stack. Logs go to
`log/shaft_<timestamp>/` (`px4.log`, `stack.log`, `agent.log`). Ctrl-C stops
everything except QGroundControl.

### 6.2 What is simulated

**World `vshaft`** (`~/PX4-Autopilot/Tools/simulation/gz/worlds/vshaft.sdf`,
generated) — deliberately changes shape with depth:

| Depth | Section |
|---|---|
| +3.0 → 0.0 m | headframe chamber, R = 3.2 m; launch ledge at z = 0 |
| 0 → −5 m | circular, R = 1.50 m |
| −5 → −11 m | rectangular, 3.0 × 2.4 m |
| −11 → −17 m | rough-cut, R ≈ 1.45 ± 0.3 m, with a ledge at −14.5 m |
| −17 → −25 m | D-shaped, R = 1.38 m with a flat wall |

Plus two service pipes down the upper sections. The vehicle spawns on the ledge
2.2 m from the axis.

**Model `x500_shaft`** (`.../gz/models/x500_shaft/`):

| Sensor | Simulated as |
|---|---|
| RPLIDAR C1 | gpu_lidar, 360°, 500 samples, 10 Hz, 0.05–12 m, σ 15 mm |
| H-Flow ToF | 1-ray gpu_lidar downward, 20 Hz, 0.08–30 m, σ 20 mm |
| H-Flow flow | PX4's optical-flow camera model (42° FOV, 7.4 rad/s) |

**Airframe `4023_gz_x500_shaft`** — the sim PX4 configuration (GPS off, EV
horizontal, baro height, conditional range, flow ≤ 5 m, Collision Prevention,
confined-space speed limits, joystick input). Sim-only settings: magnetometer
arming checks off, supply check off.

### 6.3 Changes made to the local PX4 tree

| Change | Why | Undo |
|---|---|---|
| Added airframe `4023_gz_x500_shaft` + CMake entry | the sim vehicle | delete the file / entry |
| Added `x500_shaft` model and `vshaft` world | the sim | delete |
| **Reverted commit `162cce35d8`** in the lockstep scheduler (uncommitted) | PX4-main bug: wake-ups broadcast after releasing a lock onto stack-local condition variables → `Fatal glibc error: pthread_mutex_lock … __owner == 0` or a silent freeze. Not in any release; not in v1.14; NuttX (the Pixhawk) unaffected | `git -C ~/PX4-Autopilot checkout -- platforms/posix/src/px4/common/lockstep_scheduler` |

---

## 7. Flying a mission in the simulation

### 7.1 Autonomous (`MODE=auto_launch`)
Start it and watch. The node arms, takes off, flies the mission, returns to the
ledge and disarms.

### 7.2 Manual handover (`MODE=pilot_handover`)

**QGroundControl joystick setup (once):** Vehicle Setup → Joystick → enable,
select the controller, calibrate (keep "centre stick is zero throttle"), map
buttons — suggested: one button **Position**, one **Offboard**, **Arm**,
**Disarm**. Settings persist because the run script never kills QGC.

**Procedure:**

1. Select **Position** mode. *Do not arm in Hold*: Hold, Mission, Return,
   Takeoff and Land need a GPS position; PX4 refuses with a vague "resolve
   system health failures".
2. **Arm** and climb off the ledge.
3. Fly over the middle of the shaft and hover **below ~3 m above the ledge**
   (the top of the chamber). Above that the lidar sees no walls, horizontal
   position is lost and PX4 falls back to Altitude mode (drifts). Dashboard:
   *bore fix: valid*.
4. Switch to **Offboard** (button or QGC's mode menu). *In control* turns
   **MISSION**.
5. The mission centres, descends, turns around, climbs back to the handover
   height and stops in `DONE`.
6. **Move the sticks** — PX4 switches to Position mode by itself
   (`MAN_OVERRIDE_SPD`). This also aborts the mission at any time.
7. Fly back over the ledge and land. **Hold the throttle stick fully down for
   ~3 s after touchdown**: PX4 only declares "landed" while descent is being
   commanded; a centred stick means "hold height" and it will refuse to disarm.

---

## 8. The web dashboard

`http://localhost:8080` (on the vehicle: `http://<pi-address>:8080`). Started by
the launch file (`dashboard:=false` to disable). **Read-only** — it cannot
command the vehicle.

| Panel | Shows |
|---|---|
| In control | PILOT / MISSION / FAILSAFE / ON GROUND, with a warning if you try to arm in a GPS mode |
| PX4 mode, Arming, Mission | current mode (and pilot-selected mode during failsafes), arming, mission state |
| Links | PX4, lidar, perception, mission node, joystick — green with rate, red with age |
| Mission strip | state progression |
| Bore | top-down lidar returns, bore centre, clearance circle, nearest wall, commanded velocity |
| Depth & floor | depth, ToF floor distance, clearance over time |
| Estimator | EKF2 horizontal/height validity, ToF-backed floor distance, position, velocity, attitude |
| Failsafe flags | active flags |
| Pilot input | both sticks, 16 buttons, "sticks moving" takeover indicator |
| Timeline | mode changes (with "pilot took over with the sticks" / "handed to the mission"), mission transitions, warnings |

---

## 9. Outputs

Written to `~/shaft_maps/` when the mission reaches `DONE` (and on shutdown):

* `shaft_cloud_<time>.pcd` — point cloud, frame: shaft axis, z = −depth
* `shaft_profile_<time>.csv` — `depth_m, clearance_m, r_mean_m, r_max_m, roundness`

PX4 flight logs (ULog) are in `~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/`.

---

## 10. Parameter reference

Effective values from `config/shaft.yaml` (these override the code defaults).

### `shaft_perception`
| Parameter | Value | Meaning |
|---|---|---|
| `min_coverage` | 0.85 | fraction of bearings that must have a return |
| `search_window` | 6.0 | upper bound on the raster size, m |
| `min_clearance_valid` / `max_clearance_valid` | 0.35 / 8.0 | plausibility limits, m |
| `self_filter_radius` | 0.25 | drop returns off the airframe, m |
| `lidar_yaw_offset_deg` | 0 | scan zero vs vehicle nose (measure on hardware) |
| `lidar_upside_down` | false | mirrored mounting |
| `repulse_influence` / `repulse_gain` / `repulse_max` | 1.0 / 0.35 / 0.6 | repulsion field |
| `roundness_for_radius` | 0.93 | report a circle radius only above this |
| `anchor_samples` | 30 | scans used to anchor the shaft axis |
| `publish_ev` | true | send the fix to EKF2 |

### `shaft_mission`
| Parameter | Value | Meaning |
|---|---|---|
| `start_mode` | *launch argument* | `auto_launch` / `pilot_handover` — deliberately **not** in the yaml |
| `range_source` | `laserscan` | sim: `/down_range`; real: `local_position` (EKF2 `dist_bottom`, ToF-backed only) |
| `dry_run` | false | compute everything, publish nothing to PX4 |
| `kp_lateral` / `kd_lateral` | 1.2 / 0.45 | centring PD |
| `max_lateral_speed` | 0.6 m/s | |
| `center_tol` / `center_settle_speed` / `center_hold_s` | 0.12 m / 0.15 m/s / 1.5 s | "centred" definition |
| `descend_speed` / `ascend_speed` | 0.35 / 0.5 m/s | |
| `slowdown_range` / `bottom_range` / `bottom_confirm_n` | 2.5 m / 0.5 m / 3 | floor approach and turnaround |
| `stop_dist` / `critical_dist` / `descent_min_clearance` | 0.65 / 0.45 / 0.60 m | avoidance, from the lidar centre |
| `brake_delay` / `brake_decel` | 0.4 s / 1.0 m/s² | clamp model |
| `scan_timeout` | 0.5 s | stale data stops motion |
| `max_depth` / `mission_timeout` | 40 m / 900 s | abort limits |
| `auto_descend` | true | false = hover-centre test only |
| `takeoff_clearance`, `return_tol`, `touchdown_speed`, `touchdown_range` | 1.0 m, 0.12 m, 0.3 m/s, 0.30 m | `auto_launch` only |

### Launch arguments (`shaft_bringup.launch.py`)
`start_mode` (`auto_launch`), `dashboard` (`true`), `autostart` (`true`).
This launch file is for the **simulation** (it starts Gazebo bridges); the real
vehicle is started as described in the hardware guide.

---

## 11. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| QGC: *no global position estimate*, arming refused | armed in Hold (GPS mode) | switch to **Position**, then arm |
| Won't disarm after landing | land detector needs commanded descent | hold throttle **fully down ~3 s** after touchdown |
| Mission node sits in `WAIT` in handover mode | not armed, not in Offboard, or no bore fix | check the dashboard; hover lower over the shaft |
| *Offboard engaged but no bore fix — holding* | above the chamber / too far off the shaft | descend below ~3 m, move over the shaft |
| Drone drifts while hovering high in the chamber | no walls in lidar range → no horizontal position | stay low near the shaft |
| Joystick mappings lost | QGC force-killed before saving | fixed: script no longer kills QGC; close QGC normally |
| `Fatal glibc error: pthread_mutex_lock … __owner == 0`, or PX4 freezes mid-flight | PX4-main lockstep bug | reverted locally (§6.3); if you re-pull PX4, re-check |
| Sim "PX4 did not start" but PX4 was fine | log buffering | fixed (`script -f`) |
| `gazebo already running world: default` | stale Gazebo server | the run script clears it; otherwise `pkill -9 -f "gz sim"` |
| PX4 topics exist but nothing arrives in a node | versioned topic names, or DDS discovery race | nodes resolve names and wait up to 5 s; restart the node if PX4 started much later |
| EKF2 ignores the vision fix (`xy_valid` false) | NaN in the message, wrong timestamps, or `EKF2_EV_CTRL` | handled in code; on hardware check the params |
| Launch argument `start_mode` ignored | a node-specific YAML key outranks launch parameters | keep `start_mode` out of `shaft.yaml` |
| Mission aborts on the ground with a tiny clearance | self-hits on the airframe | raise `self_filter_radius` |

---

## 12. Known limitations

* **Heading underground.** A symmetric bore gives no heading; magnetometers are
  unreliable near steel and ore. Heading drifts → the cloud smears
  azimuthally. The profile CSV is unaffected.
* **Companion computer failure deep in a shaft.** Without the lidar fix PX4 has
  no horizontal position (flow is unusable at depth). Offboard loss falls back
  to Altitude mode; there is no safe autonomous recovery. Use a tether for
  early flights; run the nodes under an auto-restarting service.
* **Downwash** reflecting off shaft walls is not simulated. Keep bores
  ≥ 2 m (≈ 4× an x500 span; 1.5 m absolute minimum).
* **ToF range on dark, wet rock** will be well below the 30 m rating.
* **Sim ≠ flight firmware.** The sim runs PX4 main; the vehicle runs v1.14.
  Estimator, commander and failsafe behaviour may differ in detail.
* **Land detector** in the sim needed the force-disarm fallback after the
  autonomous landing; verify landing/disarm on hardware.
* `pilot_handover` has not been confirmed in a complete flight yet.

---

## 13. Repository map

In this repository the PX4 additions live under `px4/` (airframe, model,
world, lockstep patch) and the scripts under `run_cmds/`; installed, they end
up as shown below.

```
px4_ros2_ws/
├── run_cmds/
│   ├── shaft_sim.sh            one-command simulation
│   └── px4_shaft_gdb.sh        PX4 under gdb (GDB=1)
└── src/shaft_inspection/
    ├── shaft_inspection/
    │   ├── scan_geometry.py        de-tilt, bore centre (EDT), sectors, repulsion, clamp
    │   ├── shaft_perception_node.py
    │   ├── shaft_mission_node.py
    │   ├── shaft_mapper_node.py
    │   ├── shaft_dashboard_node.py
    │   ├── preflight_check.py      `shaft_preflight`
    │   ├── lidar_mount_check.py    `shaft_mount_check`
    │   └── px4_topics.py           QoS, versioned topic names, PX4 timestamps
    ├── launch/shaft_bringup.launch.py   simulation stack
    ├── config/shaft.yaml, shaft.rviz
    ├── web/dashboard.html
    ├── deploy/px4_v1.14_shaft.params            load in QGC (real vehicle)
    ├── deploy/px4_v1.14_shaft_params_explained.txt
    └── docs/HARDWARE_GUIDE.md

~/PX4-Autopilot/  (local additions)
├── ROMFS/px4fmu_common/init.d-posix/airframes/4023_gz_x500_shaft
├── Tools/simulation/gz/models/x500_shaft/
└── Tools/simulation/gz/worlds/vshaft.sdf
```
