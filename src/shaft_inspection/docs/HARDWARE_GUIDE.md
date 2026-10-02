# Hardware setup and implementation guide

Building, configuring, testing and flying `shaft_inspection` on the real vehicle:

**Pixhawk 6C · PX4 v1.14.x · Raspberry Pi 5 (8 GB) · RPLIDAR C1 · Holybro H-Flow · x500-class quadrotor**

Read the main [README](../README.md) first — it explains what every part does and
why. This guide is the *how*, in the order you should do it.

> **Status and responsibility.** This software has been flown end-to-end in
> simulation only. Parameters and message definitions were checked against the
> PX4 v1.14.4 source, but nothing here has flown on hardware yet. Follow the
> stages in order, do not skip the bench tests, use a tether and prop guards
> until the vehicle has proven itself, and follow local regulations and site
> safety rules for underground work. Items marked **verify** could not be
> confirmed without the physical hardware.

---

## Contents

1. [Bill of materials](#1-bill-of-materials)
2. [How the real system differs from the simulation](#2-how-the-real-system-differs-from-the-simulation)
3. [Mechanical build](#3-mechanical-build)
4. [Electrical wiring](#4-electrical-wiring)
5. [Flight controller: firmware and parameters](#5-flight-controller-firmware-and-parameters)
6. [Holybro H-Flow configuration](#6-holybro-h-flow-configuration)
7. [Raspberry Pi 5: operating system and software](#7-raspberry-pi-5-operating-system-and-software)
8. [Configuring the stack for the real vehicle](#8-configuring-the-stack-for-the-real-vehicle)
9. [Running the stack](#9-running-the-stack)
10. [Calibration and bench tests (props off)](#10-calibration-and-bench-tests-props-off)
11. [Staged flight testing](#11-staged-flight-testing)
12. [Tuning from logs](#12-tuning-from-logs)
13. [Operating procedure and checklists](#13-operating-procedure-and-checklists)
14. [Troubleshooting](#14-troubleshooting)
15. [Open risks before unattended underground use](#15-open-risks-before-unattended-underground-use)

---

## 1. Bill of materials

| Item | Notes |
|---|---|
| x500-class quadrotor frame, motors, ESCs, props | ~0.5 m tip-to-tip. Needs payload margin for Pi 5 + lidar + cooling (~250–300 g extra) |
| **Holybro Pixhawk 6C** | with its power module (PM) and GPS mast bracket |
| **Raspberry Pi 5, 8 GB** | plus the **official active cooler** (mandatory — it throttles in an enclosed frame) |
| Storage for the Pi | 64 GB+ A2 microSD, or an NVMe HAT + SSD (faster, more robust to vibration) |
| **RPLIDAR C1** | includes its USB-UART adapter board |
| **Holybro H-Flow** (DroneCAN) | with a CAN cable (JST-GH 4-pin) |
| 5 V / 5 A BEC (switching regulator) | dedicated supply for the Pi 5; input rated for your battery voltage |
| JST-GH 6-pin cable + female Dupont jumpers | Pixhawk TELEM2 → Pi GPIO UART |
| Short USB-A to micro-USB / USB-C cable | lidar adapter → Pi (match your adapter's connector) |
| **RC transmitter + receiver** | the primary pilot link on the real vehicle (see §2) |
| Prop guards or a full cage | strongly recommended in any confined space |
| Tether (tension line + reel) | for all early shaft flights |
| LED light, downward | the H-Flow's flow sensor needs > 5 lux; the ToF does not |
| Open-cell foam | over the Pixhawk barometer |
| Vibration-damping mounts | for the Pi and lidar |
| Laptop with QGroundControl | setup, parameters, logs |
| Wi-Fi access to the Pi | SSH, the dashboard; not needed in flight |

---

## 2. How the real system differs from the simulation

| Aspect | Simulation | Real vehicle |
|---|---|---|
| PX4 version | main branch (1.18 alpha) | **v1.14.x** |
| `px4_msgs` | main | **`release/1.14` branch** — must match the firmware |
| Topic names | some versioned (`vehicle_status_v4`) | unversioned (handled automatically) |
| PX4 ↔ ROS link | UDP to a local agent | **serial**: Pixhawk TELEM2 ↔ Pi UART, 921600 baud |
| Lidar data | Gazebo bridge | `sllidar_ros2` driver over USB |
| Floor distance in ROS | Gazebo bridge `/down_range` | EKF2's `dist_bottom` (`range_source: local_position`) — v1.14 does not export the rangefinder over DDS |
| Flow / ToF into PX4 | Gazebo plugin | DroneCAN from the H-Flow |
| Pilot input | Xbox pad via QGroundControl (MAVLink) | **RC transmitter** (`COM_RC_IN_MODE 0`) |
| Mission start | `start_mode:=auto_launch` or `pilot_handover` | **`pilot_handover` only** |
| Stick takeover from Offboard | always on (newer PX4) | **`COM_RC_OVERRIDE = 3` required** (v1.14 default covers auto modes only) |
| `use_sim_time` | true | **false** |
| Lockstep bug | present in PX4 main (reverted locally) | not present (v1.14, NuttX) |

**Why an RC transmitter, not a gamepad:** a gamepad reaches PX4 through
QGroundControl and a telemetry radio. That link has latency, and it dies as
soon as the vehicle is underground. An RC receiver on the vehicle gives you
direct, low-latency control for takeoff, handover, takeover and landing
whenever you are in range. Underground you will usually lose RC too — which is
why the mission is autonomous and `COM_RCL_EXCEPT` keeps Offboard running
without it.

---

## 3. Mechanical build

### 3.1 Coordinate conventions (use these for every measurement)

PX4 body frame is **FRD**: **x forward**, **y right**, **z down**, origin at the
**centre of gravity** (CoG). Measure the CoG with the full payload and battery
installed (balance the frame on two fingers along each axis).

### 3.2 RPLIDAR C1 — on top

* **Scan plane above everything.** The laser sweeps a horizontal plane at the
  lidar's optical centre; props, GPS mast, antennas and cables must all be
  **below** that plane or they appear as permanent "walls" a few centimetres
  away. Raise it on a small standoff above the GPS mast if needed.
* **Clear 360°.** No mast or bracket may block any bearing — a blocked sector
  reduces coverage and can drop the bore fix below the 85 % coverage gate.
* **As close to the vertical axis through the CoG as possible.** An offset is
  measurable and correctable (§5.4 `EKF2_EV_POS_*`), but small is better.
* **Level and rigid**, on vibration dampers. Tilt of the mount relative to the
  airframe is not compensated.
* **Cable routed downward immediately**, never across the scan plane.
* **Note the orientation** of the housing (for example "cable toward the
  rear"). The software needs the angle between the lidar's 0° and the nose;
  you will measure it in §10.3 rather than trust the marking.

### 3.3 Holybro H-Flow — underneath

* Facing straight down, with a **clear cone of at least 45°** — the flow sensor
  sees 42°, the ToF 12.4° × 6.2°. Landing gear, battery straps and wires must
  stay out of that cone.
* **Orientation:** mount it with its arrow/forward mark pointing to the nose
  (verify the exact marking and any orientation parameter in Holybro's H-Flow
  documentation for your unit).
* Mount rigidly; flow is sensitive to vibration.
* Keep it away from the landing gear's ground-contact points so debris does not
  hit the lens.

### 3.4 Raspberry Pi 5

* On dampers, with the active cooler's air path unobstructed. Expect sustained
  high CPU; without airflow it throttles within minutes.
* Keep it away from the GPS antenna and magnetometer (switching noise).

### 3.5 Barometer

The Pixhawk 6C's barometer is inside the unit. Cover the case vents with a
piece of open-cell foam and shield the flight controller from direct prop wash.
In a shaft the recirculating wash makes this more important than in open air.

### 3.6 Record the offsets

Write these down; they go into parameters in §5.4.

| Offset (FRD from CoG, metres) | x (fwd) | y (right) | z (down) |
|---|---|---|---|
| H-Flow optical centre → `EKF2_OF_POS_*` | | | |
| H-Flow ToF (same unit) → `EKF2_RNG_POS_*` | | | |
| Lidar optical centre → `EKF2_EV_POS_*` | | | |

---

## 4. Electrical wiring

### 4.1 Overview

```
 Battery ──► Pixhawk power module ──► Pixhawk 6C (POWER1)
    │                                   ├── TELEM2 ──(TX/RX/GND only)──► Pi 5 GPIO UART
    │                                   ├── CAN1   ─────────────────────► H-Flow
    │                                   ├── RC IN  ─────────────────────► RC receiver
    │                                   ├── TELEM1 ─────────────────────► telemetry radio (optional)
    │                                   └── GPS1   ─────────────────────► GPS/compass mast (optional)
    │
    └──► 5 V / 5 A BEC ──► Raspberry Pi 5 (USB-C, or 5V/GND header pins)
                               └── USB ──► RPLIDAR C1 adapter
```

### 4.2 Pixhawk TELEM2 → Raspberry Pi 5 UART

Pixhawk telemetry connectors are JST-GH 6-pin. Standard Pixhawk pinout (pin 1 is
the red/marked wire; **verify against the Holybro 6C pinout sheet** before
connecting):

| TELEM2 pin | Signal | → Pi 5 header pin |
|---|---|---|
| 1 | +5 V | **not connected** |
| 2 | TX (from Pixhawk) | **pin 10 — GPIO15 / RXD** |
| 3 | RX (to Pixhawk) | **pin 8 — GPIO14 / TXD** |
| 4 | CTS | not connected |
| 5 | RTS | not connected |
| 6 | GND | **pin 6 — GND** |

* **TX goes to RX** and vice versa.
* **Never connect the Pixhawk's 5 V to the Pi.** The Pi is powered by its own
  BEC; two supplies fighting through the header can damage either board.
* Both sides use 3.3 V logic — no level shifter needed.
* A common ground via pin 6 is required.

### 4.3 H-Flow → CAN1

Plug the H-Flow's DroneCAN cable into **CAN1**. The CAN bus powers it. If it is
the last (only) node on the bus, confirm termination as described in Holybro's
documentation (**verify**).

### 4.4 RPLIDAR C1 → Pi USB

Connect the lidar's USB-UART adapter board to a Pi USB port. The lidar draws its
power from USB. Use a short, strain-relieved cable.

### 4.5 Powering the Pi 5

The Pi 5 wants **5 V at up to 5 A**. Use a dedicated BEC from the battery:

* **Preferred:** a BEC with a USB-C PD output that negotiates 5 A.
* **Or** feed 5 V to header pins 2/4 (5 V) and 6 (GND) from a 5 A BEC. The Pi
  cannot negotiate current this way and limits USB-port current by default;
  add `usb_max_current_enable=1` to `/boot/firmware/config.txt` so the lidar
  gets enough (**verify** the setting name for your Pi OS / Ubuntu image).
* Do not power the Pi from the Pixhawk.

Check for under-voltage in flight-like load later (§10.1).

---

## 5. Flight controller: firmware and parameters

### 5.1 Firmware

1. QGroundControl → **Vehicle Setup → Firmware**, connect the Pixhawk by USB,
   choose **PX4 Pro**, and select a **v1.14.x** release (latest v1.14 patch).
2. **Airframe:** choose your frame (e.g. the Holybro X500 V2 entry, or the generic
   quadrotor X matching your build). Reboot.
3. **Sensors:** calibrate compass, gyro, accelerometer and level horizon **with
   the Pi, lidar and H-Flow installed** (they shift the CoG and add magnetic
   interference).
4. **Radio:** calibrate the RC transmitter.
5. **Flight modes** (one 3-position switch + one 2-position switch is enough):

   | Switch position | Mode |
   |---|---|
   | Mode switch 1 | **Altitude** (fallback, needs no position) |
   | Mode switch 2 | **Position** (normal manual flight) |
   | Mode switch 3 | **Offboard** (starts the inspection) |
   | Kill switch | **Kill** (motors off — last resort) |

6. **Power:** calibrate the power module (battery voltage and current).
7. **ESCs / motors:** check spin direction and order with props off.

### 5.2 Serial link to the Pi (DDS)

In **Parameters**:

| Parameter | Value |
|---|---|
| `UXRCE_DDS_CFG` | **TELEM 2** |
| `SER_TEL2_BAUD` | **921600 8N1** (appears after setting the above and rebooting) |

Reboot. If you use a telemetry radio, keep it on TELEM1 (MAVLink).

### 5.3 Load the shaft parameters

`deploy/px4_v1.14_shaft.params` contains 48 parameters with types checked
against the v1.14.4 source. **Parameters → Tools → Load from file**, then reboot.
The companion `deploy/px4_v1.14_shaft_params_explained.txt` explains each.

The ones that **must** change from v1.14 defaults (all included in the file):

| Parameter | v1.14 default | Set to | Why |
|---|---|---|---|
| `EKF2_EV_CTRL` | 15 | **1** | fuse only horizontal position from the bore fix (no yaw, no height) |
| `EKF2_HGT_REF` | 1 (GPS) | **0** | barometer height reference (see README §5.3) |
| `EKF2_GPS_CTRL` | 7 | **0** | no GPS |
| `EKF2_OF_CTRL` | 0 | **1** | use H-Flow optical flow near surfaces |
| `COM_OBL_RC_ACT` | 0 | **1** | Offboard lost → **Altitude** (Land would sink into the shaft) |
| `COM_RC_OVERRIDE` | 1 | **3** | sticks take over from **Offboard** too |
| `COM_RCL_EXCEPT` | 0 | **4** | ignore RC loss while in Offboard (no RC underground) |
| `UAVCAN_ENABLE` | 0 | **2** | DroneCAN sensors (H-Flow) |

### 5.4 Parameters you must set from your measurements

| Parameter | Source |
|---|---|
| `EKF2_OF_POS_X/Y/Z` | H-Flow optical centre offset (§3.6) |
| `EKF2_RNG_POS_X/Y/Z` | H-Flow ToF offset (§3.6) |
| `EKF2_EV_POS_X/Y/Z` | lidar optical-centre offset (§3.6) |
| `MPC_THR_HOVER` | from a hover log (§12) |
| `EKF2_EV_DELAY` | start at 50 ms, tune from logs (§12) |

### 5.5 Parameters to decide per site

| Parameter | Guidance |
|---|---|
| `SYS_HAS_GPS` | 0 without GPS; 1 if you fly a GPS mast for surface work |
| `EKF2_MAG_TYPE` | 0 (automatic) is fine at the surface. Near steel/ore the compass is wrong; see README §12 |
| `COM_RC_IN_MODE` | **0** (RC transmitter only) on the real vehicle. The sim uses 1 (joystick) |
| `SDLOG_MODE` | **1** (from boot until disarm) so the log includes everything before arming |
| `COM_DISARM_LAND` | 2 s default is fine |
| `CP_DIST` | 0.6 m, protects the pilot in Position mode |
| Battery failsafes (`BAT_*`, `COM_LOW_BAT_ACT`) | set to your pack. Remember **the climb out needs more energy than the descent** |

---

## 6. Holybro H-Flow configuration

1. With `UAVCAN_ENABLE = 2`, `UAVCAN_SUB_FLOW = 1`, `UAVCAN_SUB_RNG = 1` loaded,
   reboot the Pixhawk.
2. QGroundControl should list the H-Flow as a DroneCAN node. Follow Holybro's
   H-Flow documentation for firmware updates and any node-side settings such as
   mounting orientation (**verify** names and values on your unit).
3. **Check the rangefinder:** Analyze Tools → **MAVLink Inspector** →
   `DISTANCE_SENSOR`. Lift the vehicle by hand: `current_distance` must track
   height smoothly from ~0.1 m to at least 2 m.
4. **Check flow direction:** `OPTICAL_FLOW_RAD`. Over a textured floor, move the
   vehicle forward by hand, then right. Note the sign of the integrated flow;
   then rotate the vehicle and repeat. If axes are swapped or inverted, fix the
   orientation setting before any flight — wrong flow axes make Position mode
   run away.
5. Confirm `EKF2_MIN_RNG` (0.08 m) is below the reading with the vehicle on the
   ground.

---

## 7. Raspberry Pi 5: operating system and software

### 7.1 Operating system

Flash **Ubuntu Server 24.04 LTS (64-bit)** for Raspberry Pi with Raspberry Pi
Imager (enable SSH and Wi-Fi in the imager's settings). Boot, then:

```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y git python3-pip build-essential cmake
sudo usermod -aG dialout $USER          # serial port access; log out and back in
```

**Enable the GPIO UART** (GPIO14/15). Add to `/boot/firmware/config.txt`:

```
dtparam=uart0=on
```

Disable the serial login console on that UART (remove any `console=serial0,...`
or `console=ttyAMA0,...` from `/boot/firmware/cmdline.txt`), then reboot and
find the device:

```bash
ls -l /dev/ttyAMA* /dev/serial*
```

On a Pi 5 the header UART is normally **`/dev/ttyAMA0`** (the separate debug
connector is `ttyAMA10`). **Verify** on your image before relying on it; the
rest of this guide uses `/dev/ttyAMA0`.

**Performance:**

```bash
sudo apt install -y cpufrequtils
echo 'GOVERNOR="performance"' | sudo tee /etc/default/cpufrequtils
```

### 7.2 ROS 2 Jazzy

Install ROS 2 **Jazzy** (the distribution for Ubuntu 24.04) following the
official Debian-package instructions for Ubuntu (arm64 is supported). Install
`ros-jazzy-ros-base` (no desktop needed on the vehicle) plus:

```bash
sudo apt install -y ros-jazzy-ros-base python3-colcon-common-extensions \
                    python3-rosdep python3-numpy python3-scipy
pip3 install --user --break-system-packages websockets     # dashboard
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### 7.3 Micro-XRCE-DDS Agent

```bash
cd ~ && git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. && make -j4 && sudo make install && sudo ldconfig /usr/local/lib/
```

Test with the Pixhawk connected and powered:

```bash
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
```

Within a few seconds the agent should log sessions and topics being created.

### 7.4 ROS workspace

```bash
mkdir -p ~/shaft_ws/src && cd ~/shaft_ws/src
git clone -b release/1.14 https://github.com/PX4/px4_msgs.git      # MUST match v1.14 firmware
git clone https://github.com/Slamtec/sllidar_ros2.git              # RPLIDAR C1 driver
# copy the shaft_inspection package from the development machine:
git clone https://github.com/abulhasnat-abdullah/shaft_inspection.git
cd ~/shaft_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --parallel-workers 2      # px4_msgs takes a while on a Pi
echo "source ~/shaft_ws/install/setup.bash" >> ~/.bashrc
```

`px4_msgs` from the wrong branch compiles fine and fails silently at runtime
(fields misaligned or topics never matched). If PX4 topics appear but values
are nonsense, this is the first thing to check.

### 7.5 Stable lidar device name

The C1's adapter is a USB-serial chip; its `/dev/ttyUSB*` number can change.
Find its IDs with `udevadm info -a -n /dev/ttyUSB0 | grep -E "idVendor|idProduct|serial"`
and create `/etc/udev/rules/99-rplidar.rules`:

```
KERNEL=="ttyUSB*", ATTRS{idVendor}=="<vendor>", ATTRS{idProduct}=="<product>", MODE="0666", SYMLINK+="rplidar"
```

`sudo udevadm control --reload-rules && sudo udevadm trigger`, replug; use
`/dev/rplidar` from now on.

### 7.6 Lidar driver check

```bash
ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=/dev/rplidar
ros2 topic hz /scan        # expect ~10 Hz
```

(C1 defaults in that launch file: 460800 baud, `frame_id: laser`,
`scan_mode: Standard`, `angle_compensate: true`.)

---

## 8. Configuring the stack for the real vehicle

The simulation launch file starts Gazebo bridges and is **not** used on the
vehicle. Create `~/shaft_ws/shaft_real.yaml`:

```yaml
/**:
  ros__parameters:
    use_sim_time: false

shaft_perception:
  ros__parameters:
    scan_topic: /scan
    min_coverage: 0.85
    search_window: 6.0
    min_clearance_valid: 0.35
    max_clearance_valid: 8.0
    self_filter_radius: 0.25       # MEASURE (§10.4)
    lidar_yaw_offset_deg: 0.0      # MEASURE (§10.3)
    lidar_upside_down: false       # MEASURE (§10.3)
    repulse_influence: 1.0
    repulse_gain: 0.35
    repulse_max: 0.6
    anchor_samples: 30
    publish_ev: true

shaft_mission:
  ros__parameters:
    start_mode: pilot_handover     # never auto_launch on the real vehicle
    range_source: local_position   # EKF2 dist_bottom (ToF-backed only)
    dry_run: false                 # true for bench tests (§10.7)
    control_hz: 20.0
    kp_lateral: 1.2
    kd_lateral: 0.45
    max_lateral_speed: 0.4         # start slower than the sim
    center_tol: 0.12
    center_settle_speed: 0.15
    center_hold_s: 1.5
    descend_speed: 0.25            # start slower than the sim
    ascend_speed: 0.4
    slowdown_range: 2.5
    bottom_range: 0.8              # start higher than the sim's 0.5 m
    bottom_confirm_n: 3
    stop_dist: 0.65                # from the lidar centre; re-check for your prop reach
    critical_dist: 0.45
    descent_min_clearance: 0.60
    brake_delay: 0.4
    brake_decel: 1.0
    scan_timeout: 0.5
    use_repulsion: true
    auto_descend: true             # false for hover-centre tests
    max_depth: 10.0                # SET: known shaft depth + margin
    mission_timeout: 300.0         # SET: from battery endurance

shaft_mapper:
  ros__parameters:
    scan_topic: /scan
    self_filter_radius: 0.25
    lidar_yaw_offset_deg: 0.0      # same as perception
    lidar_upside_down: false
```

Here `start_mode` **is** in the YAML deliberately — there is no launch file on
the vehicle, so the YAML is the single source.

**Re-check the distances for your airframe.** The defaults assume prop tips
~0.38 m from the lidar's vertical axis. If your reach is larger (prop guards,
larger props), increase `critical_dist`, `stop_dist` and
`descent_min_clearance` by the difference.

---

## 9. Running the stack

### 9.1 Manually (for bench work)

One terminal each (or `tmux`):

```bash
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=/dev/rplidar
ros2 run shaft_inspection shaft_perception --ros-args --params-file ~/shaft_ws/shaft_real.yaml
ros2 run shaft_inspection shaft_mission    --ros-args --params-file ~/shaft_ws/shaft_real.yaml
ros2 run shaft_inspection shaft_mapper     --ros-args --params-file ~/shaft_ws/shaft_real.yaml
ros2 run shaft_inspection shaft_dashboard  --ros-args -p port:=8080
```

Dashboard: `http://<pi-address>:8080` from a laptop or phone on the same network.

### 9.2 Automatically at boot (for flight)

Services restart a crashed node within a second — important, because a dead
mission node deep in a shaft leaves the vehicle without horizontal position.

`/etc/systemd/system/shaft-agent.service`:

```ini
[Unit]
Description=Micro-XRCE-DDS agent (Pixhawk TELEM2)
After=network.target

[Service]
User=<your-user>
ExecStart=/usr/local/bin/MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
Restart=always
RestartSec=1

[Install]
WantedBy=multi-user.target
```

`/etc/systemd/system/shaft-node@.service` (one template for every node):

```ini
[Unit]
Description=shaft_inspection %i
After=shaft-agent.service

[Service]
User=<your-user>
Environment=ROS_DOMAIN_ID=0
ExecStart=/bin/bash -lc 'source /opt/ros/jazzy/setup.bash && source ~/shaft_ws/install/setup.bash && exec ros2 run shaft_inspection %i --ros-args --params-file ~/shaft_ws/shaft_real.yaml'
Restart=always
RestartSec=1

[Install]
WantedBy=multi-user.target
```

A matching `shaft-lidar.service` runs
`ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=/dev/rplidar`.

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now shaft-agent shaft-lidar \
     shaft-node@shaft_perception shaft-node@shaft_mission \
     shaft-node@shaft_mapper shaft-node@shaft_dashboard
journalctl -fu shaft-node@shaft_mission        # follow a node's log
```

The mission node restarting mid-flight returns it to `WAIT` — it will not
resume a mission by itself. With `COM_OBL_RC_ACT = 1`, PX4 is already in
Altitude mode by then (Offboard stream lost for > `COM_OF_LOSS_T`).

### 9.3 Onboard logging

```bash
mkdir -p ~/flights && cd ~/flights
ros2 bag record -o flight_$(date +%Y%m%d_%H%M%S) /scan /rosout \
    /shaft/offset /shaft/clearance /shaft/valid /shaft/state /shaft/depth \
    /shaft/cmd_vel_ned /shaft/sector_min \
    /fmu/out/vehicle_status /fmu/out/vehicle_local_position \
    /fmu/out/vehicle_attitude /fmu/out/failsafe_flags
```

Together with the PX4 ULog on the SD card (`SDLOG_MODE = 1`), this is your only
telemetry underground.

---

## 10. Calibration and bench tests (props off)

**Remove the propellers for all of §10.**

### 10.1 Power and links

```bash
vcgencmd get_throttled          # 0x0 = never under-voltage or throttled
ros2 run shaft_inspection shaft_preflight
```

Expected: lidar ~10 Hz, PX4 status arriving, `z_valid` true. Run a CPU load
(`stress`/the full stack) for 10 minutes and re-check `get_throttled` and
`vcgencmd measure_temp`.

### 10.2 Rangefinder via EKF2

`shaft_preflight` → "H-Flow range (dist_bottom)" must show `from_range=True`.
Lift the vehicle: the dashboard's *Floor below (ToF)* tracks the height.

### 10.3 Lidar mounting offset

1. Vehicle on the ground, open space around it.
2. Hold a flat board 0.6–1.0 m **directly in front of the nose**:
   `ros2 run shaft_inspection shaft_mount_check` → set
   `lidar_yaw_offset_deg` to the value it prints.
3. Restart perception, move the board to the vehicle's **left**: it must read
   about **+90°** after the offset. About −90° means the scan is mirrored → set
   `lidar_upside_down: true`.

### 10.4 Self filter

With nothing within 1 m of the vehicle, look at the dashboard's bore view (or
`shaft_preflight` "self-hits"). Any returns that stay put are the airframe. Set
`self_filter_radius` a few centimetres above the farthest one, but keep it well
below `critical_dist`.

### 10.5 The sign test — do not skip

Build a **mock bore** you can walk around: a ring of boxes or boards, a tarp
enclosure, or a corridor end, at least 2 m across. Place the vehicle in it
(props off) with the stack running.

1. Push the vehicle **20 cm toward the nose**. Dashboard bore view: the blue
   centre marker moves 20 cm **behind** the vehicle; *Offset* ≈ 0.20.
2. EKF2 local position (dashboard *Estimator*): north/east moves **20 cm in the
   same direction the vehicle moved** (in NED, using the heading).
3. Rotate the vehicle 90° and repeat.

If anything moves the wrong way or the wrong amount, **stop** and fix the mount
parameters. A sign error flies the vehicle into the wall at full speed.

### 10.6 EKF2 accepts the fix

In the mock bore: dashboard *horizontal position* **valid**. In QGroundControl
MAVLink Inspector (or the ULog), EKF2's external-vision position fusion flag is
active. If `xy_valid` stays false: check `EKF2_EV_CTRL = 1`, the agent link, and
that `px4_msgs` is `release/1.14`.

### 10.7 Dry run of the mission

Set `dry_run: true`, restart the mission node. It computes everything and
publishes nothing to PX4. In the mock bore, displace the vehicle by hand: the
dashboard's yellow **command arrow** must point back toward the bore centre.
Set `dry_run: false` afterwards.

### 10.8 Failsafe behaviour

With props **removed**, safely restrained:

1. Arm in Position mode, switch to Offboard (the mission node streams setpoints).
2. `sudo systemctl stop shaft-node@shaft_mission` (or Ctrl-C it).
3. Within ~1 s PX4 must switch to **Altitude** (dashboard timeline / QGC).
4. Move the sticks during Offboard: PX4 must switch to **Position**
   (`COM_RC_OVERRIDE = 3`).
5. Disarm.

### 10.9 Arming and disarming habits

* Arm only in **Position** (or Altitude). Hold, Mission, Return, Takeoff and Land
  need GPS and are refused.
* After touchdown, **hold throttle down ~3 s** until PX4 declares landed; only
  then will it disarm.

---

## 11. Staged flight testing

Do not advance until every pass criterion is met. Tether and prop guards on
throughout stages 2–5.

| Stage | Where | What | Pass criteria |
|---|---|---|---|
| **1. Bench** | workshop | all of §10 | every check passes; sign test correct in two orientations |
| **2. Manual hover** | open space, 1–2 m | Position mode on flow + ToF only (no walls) | holds position within ~0.3 m; no drift; landing and disarm work |
| **3. Terrain step** | open space | fly slowly off a table / step edge in Position mode | ULog shows a **terrain** reset, not a height jump; vehicle stays level |
| **4. Centring hover** | mock bore ≥ 2.5 m, 1–2 m high | Offboard with `auto_descend: false` | converges to centre within ±5 cm, no oscillation; takeover by sticks works |
| **5. Obstacle** | same | add a box protruding into the bore | vehicle steers around / stops; never closer than `stop_dist` |
| **6. Short vertical** | stairwell, tower, elevator shaft, 3–8 m | full mission, RC in range | clean descent, turnaround, climb, `DONE`; takeover and landing |
| **7. Real shaft, shallow** | ≤ 10–15 m deep, ≥ 2.5 m wide, tethered | full mission with site survey done | as stage 6; logs reviewed; battery margin at landing ≥ 30 % |
| **8. Real shaft, full depth** | after stage 7 is repeatable | increase `max_depth` gradually | as above |

For stage 6 onward, raise speeds and lower `bottom_range` toward the sim values
only after reviewing logs from the previous flights.

---

## 12. Tuning from logs

Open the PX4 ULog in **PX4 Flight Review** or **PlotJuggler**, and the rosbag
alongside.

| Symptom | Look at | Adjust |
|---|---|---|
| Centring oscillates | `/shaft/offset`, `vehicle_local_position.vx/vy` | lower `kp_lateral`, raise `kd_lateral` |
| Centring slow / sluggish | same | raise `kp_lateral` in small steps |
| EKF2 position lags the fix, or innovations spike with motion | `estimator_innovations` (external-vision horizontal position) vs velocity | tune `EKF2_EV_DELAY` (lidar scan + processing latency; try 30–100 ms) |
| Vision fix rejected intermittently | `estimator_innovation_test_ratios` for EV position | raise `EKF2_EVP_GATE` slightly, or `EKF2_EVP_NOISE` |
| Vehicle bobs in the shaft | `vehicle_local_position.z`, baro | barometer foam; lower `descend_speed`; accept a few decimetres |
| Descent keeps gating | dashboard timeline "descent gated (offset …, clearance …)" | cause is real (off-centre / tight section); check the profile, don't just widen gates |
| Hover throttle | `vehicle_thrust_setpoint` during a steady hover | set `MPC_THR_HOVER` |

---

## 13. Operating procedure and checklists

### 13.1 Site survey (before bringing the vehicle)

- [ ] Shaft diameter at the mouth and, if known, down its length (≥ 2 m preferred)
- [ ] Depth (sets `max_depth`)
- [ ] Obstructions: cables, ladders, platforms, pipes, water, loose material
- [ ] Airflow / ventilation direction and strength
- [ ] Lighting at the mouth (flow needs > 5 lux for manual position hold)
- [ ] Dust and water drip (lidar and ToF degrade)
- [ ] Safe takeoff/landing spot near the mouth, clear of the edge
- [ ] Tether anchor point; exclusion zone for people
- [ ] Permissions and site safety briefing

### 13.2 Pre-flight

- [ ] Battery fully charged; endurance known; `mission_timeout` set below it
- [ ] Props, guards, tether attachment secure
- [ ] Lidar lens and H-Flow window clean
- [ ] `max_depth` set for this shaft
- [ ] Power on; wait for all services
- [ ] `ros2 run shaft_inspection shaft_preflight` → **GO**
- [ ] Dashboard: all links green, no unexpected failsafe flags
- [ ] Rosbag recording started
- [ ] RC: mode switch in **Position**, kill switch tested off
- [ ] Brief: pilot, tether handler, observer; abort word agreed

### 13.3 Flight

1. Arm in **Position**; take off to ~1 m; check position hold.
2. Fly over the shaft centre, **low** (walls within lidar range, ~< 3 m above
   the collar); dashboard *bore fix: valid*, *clearance* sensible.
3. Switch to **Offboard**. Dashboard *In control: MISSION*.
4. Monitor the dashboard (if the Wi-Fi reaches) and the tether.
5. **Abort any time:** move the sticks (→ Position) or flip the mode switch to
   Altitude.
6. At `DONE` the vehicle hovers at the handover height → take over with the
   sticks / mode switch → fly to the landing spot.
7. Land; **hold throttle down ~3 s**; disarm.

### 13.4 Post-flight

- [ ] Stop the rosbag; copy the ULog (SD card) and the rosbag off the Pi
- [ ] Copy `~/shaft_maps/` (profile CSV, point cloud)
- [ ] Note battery remaining, anomalies, dashboard timeline warnings
- [ ] Inspect props, guards, lidar, H-Flow

### 13.5 Emergencies

| Situation | Action |
|---|---|
| Vehicle drifts toward a wall in Offboard | move sticks (→ Position) and fly away from the wall |
| Offboard lost / companion crashed | PX4 is in Altitude: fly up and out manually if RC reaches; otherwise tether recovery |
| RC lost underground during the mission | nothing: the mission continues (`COM_RCL_EXCEPT`). It hovers at `DONE` until RC returns |
| Low battery warning in the shaft | take over and climb out immediately |
| Loss of control near people | **kill switch** |

---

## 14. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Agent shows no session | wiring TX/RX swapped, wrong device, baud, `UXRCE_DDS_CFG` not TELEM2 | re-check §4.2, §5.2, `/dev/ttyAMA0`, serial console disabled |
| PX4 topics listed but values nonsense / never arrive in nodes | `px4_msgs` not `release/1.14` | rebuild with the right branch |
| `/scan` missing | lidar power, `/dev/rplidar` rule, baud | `ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=...` and read its errors |
| Bore fix never valid in a real shaft | coverage < 85 % (blocked sectors, dark/absorbing walls, out of 12 m range), self filter too large | check the dashboard bore view; clear the mount; lower the vehicle |
| Mission aborts on the ground | self-hits inside `critical_dist` | raise `self_filter_radius` (§10.4) |
| Centring pushes toward the wall | wrong `lidar_yaw_offset_deg` / `lidar_upside_down` | redo §10.3 and §10.5 |
| Arming refused, "no global position" | mode is Hold | switch to Position |
| Will not disarm after landing | land detector | hold throttle down ~3 s |
| Sticks do not take over from Offboard | `COM_RC_OVERRIDE` still 1 | set 3 |
| Offboard switch refused | mission node not streaming, or no valid local position | dashboard links; bore fix / flow must give horizontal position |
| Height jumps when crossing the collar | `EKF2_HGT_REF = 2` (range) | set 0 (baro) |
| Position mode runs away near the ground | flow axes wrong | §6 step 4 |
| Pi reboots or throttles in flight | power / heat | 5 A BEC, active cooler, `vcgencmd get_throttled` |

---

## 15. Open risks before unattended underground use

1. **Heading underground.** No reliable heading reference inside a steel- or
   ore-rich symmetric bore. Centring is unaffected; the 3D map smears.
2. **Companion failure at depth.** No safe autonomous recovery exists in PX4 for
   "no horizontal position deep in a shaft". Mitigations: auto-restarting
   services, a tether, conservative depths until reliability is proven.
3. **Downwash in narrow bores.** Not modelled in simulation. Characterise it in
   stage 6 before real shafts; keep bores ≥ 2 m.
4. **ToF range on dark, wet rock** may be well below 30 m — the vehicle descends
   on the barometer until the floor is seen, so `slowdown_range` and
   `bottom_range` must allow for late detection at your descent speed.
5. **Dust at the bottom** blinds both lidar and ToF; the 0.5–0.8 m turnaround
   (no landing) limits it but does not eliminate it.
6. **Firmware gap.** The simulation runs PX4 main; the vehicle runs v1.14.
   Differences in estimator and failsafe behaviour are possible — the staged
   tests are how you find them.
