# PX4 + RPLIDAR C1 Obstacle Avoidance Setup

This guide explains how to use a Raspberry Pi 5 with a Pixhawk 6C and an RPLIDAR C1 to make the drone move backward a little when an obstacle comes too close in front of it.

## What this setup does

- The Raspberry Pi reads distance data from the RPLIDAR.
- If an object enters the front safety zone, the Pi sends a small backward velocity command to PX4.
- The drone keeps moving backward only while the obstacle is too close.
- When the obstacle is far enough away again, the drone stops backing up.

## Important limitation

The RPLIDAR C1 is a 2D lidar, so it only sees the scan plane it is mounted in. It may not detect hands or obstacles above or below that plane. Test with props removed first.

## Install Python packages

On Ubuntu 24.04 on the Raspberry Pi, run:

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip
python3 -m venv ~/px4_avoidance
source ~/px4_avoidance/bin/activate
pip install --upgrade pip
pip install mavsdk rplidar pyserial
```

If the lidar or Pixhawk serial ports are blocked, add your user to the dialout group:

```bash
sudo usermod -aG dialout $USER
```

Log out and log back in after that.

## Files in this workspace

- `lidar_front_test.py` checks that the lidar is working and prints the closest front distance.
- `avoidance_px4.py` connects to PX4 and sends the backward command when an obstacle gets too close.

## Step 1: Test the lidar only

Run this first before trying flight control.

```bash
source ~/px4_avoidance/bin/activate
python3 lidar_front_test.py --port /dev/ttyUSB0
```

If the port is different on your Pi, replace `/dev/ttyUSB0` with the correct device.

You should see the front distance update as you move your hand in front of the lidar.

## Step 2: Verify PX4 connection

Connect the Pixhawk 6C to the Raspberry Pi.

Common serial addresses are:

- `serial:///dev/ttyACM0:921600` for USB
- `serial:///dev/ttyAMA0:921600` or `serial:///dev/ttyS0:921600` for UART

Start with the correct one for your wiring.

## Step 3: Run the avoidance script

```bash
source ~/px4_avoidance/bin/activate
python3 avoidance_px4.py --lidar-port /dev/ttyUSB0 --mavsdk-address serial:///dev/ttyACM0:921600
```

Replace the ports if your device names are different.

## Step 4: Flight test procedure

1. Remove the propellers for the first test.
2. Confirm the lidar test script shows changing distance values.
3. Confirm PX4 is connected to the Pi.
4. Put the drone in a safe hover setup.
5. Run the avoidance script.
6. Press Enter in the terminal only when the drone is already stable and safe.
7. Move your hand in front of the lidar.
8. The drone should command a small backward motion.
9. Remove the obstacle and confirm the drone stops backing up.

## Suggested tuning values

Start with these values and adjust slowly:

- `stop-distance`: `1.5`
- `clear-distance`: `2.0`
- `back-speed`: `0.25`

If the drone reacts too early, lower the stop distance a little. If it reacts too late, raise it a little.

## Safety notes

- Test without props first.
- Keep the backward speed low.
- Do not rely on one lidar for full 3D obstacle detection.
- Make sure you can immediately regain manual control.

## If you want the next improvement

A better later version is to add:

- front, left, right, and downward sensing
- a depth camera or multiple lidars
- a manual override switch
- a PX4 parameter-based safety layer
