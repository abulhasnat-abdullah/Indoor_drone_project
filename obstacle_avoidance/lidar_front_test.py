#!/usr/bin/env python3
import argparse
import time
from rplidar import RPLidar

def is_front(angle_deg, cone_deg):
    angle_deg = angle_deg % 360.0
    return angle_deg <= cone_deg or angle_deg >= (360.0 - cone_deg)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--cone-deg", type=float, default=25.0)
    args = parser.parse_args()

    lidar = RPLidar(args.port, baudrate=args.baudrate, timeout=3)

    try:
        lidar.start_motor()
        print(f"Connected to lidar on {args.port}")
        print("Move your hand in front of the lidar to see the distance change.")

        for scan in lidar.iter_scans():
            front_distances = []
            for _, angle, distance_mm in scan:
                if distance_mm <= 0:
                    continue
                if not is_front(angle, args.cone_deg):
                    continue

                distance_m = distance_mm / 1000.0
                if 0.05 <= distance_m <= 12.0:
                    front_distances.append(distance_m)

            if front_distances:
                print(f"Front distance: {min(front_distances):.2f} m")
            else:
                print("Front distance: none")

            time.sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            lidar.stop()
        except Exception:
            pass
        lidar.disconnect()

if __name__ == "__main__":
    main()