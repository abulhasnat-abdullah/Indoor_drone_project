#!/usr/bin/env python3
import argparse
import asyncio
import threading
import time
from rplidar import RPLidar
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

def is_front(angle_deg, cone_deg):
    angle_deg = angle_deg % 360.0
    return angle_deg <= cone_deg or angle_deg >= (360.0 - cone_deg)

class SharedState:
    def __init__(self):
        self.distance_m = None
        self.error = None
        self.running = True

def lidar_worker(state, port, baudrate, cone_deg):
    lidar = None
    try:
        lidar = RPLidar(port, baudrate=baudrate, timeout=3)
        lidar.start_motor()

        for scan in lidar.iter_scans():
            front_distances = []
            for _, angle, distance_mm in scan:
                if distance_mm <= 0:
                    continue
                if not is_front(angle, cone_deg):
                    continue

                distance_m = distance_mm / 1000.0
                if 0.05 <= distance_m <= 12.0:
                    front_distances.append(distance_m)

            if front_distances:
                state.distance_m = min(front_distances)

    except Exception as exc:
        state.error = str(exc)
    finally:
        state.running = False
        if lidar is not None:
            try:
                lidar.stop()
            except Exception:
                pass
            try:
                lidar.disconnect()
            except Exception:
                pass

async def wait_for_connection(drone):
    async for state in drone.core.connection_state():
        if state.is_connected:
            return

async def main_async(args):
    state = SharedState()

    thread = threading.Thread(
        target=lidar_worker,
        args=(state, args.lidar_port, args.lidar_baudrate, args.cone_deg),
        daemon=True,
    )
    thread.start()

    drone = System()
    await drone.connect(system_address=args.mavsdk_address)

    print("Waiting for PX4 connection...")
    await wait_for_connection(drone)
    print("PX4 connected")

    if state.error:
        raise RuntimeError(f"Lidar failed to start: {state.error}")

    print("")
    print("Take off manually and hover safely.")
    print("Then press Enter here to engage offboard avoidance.")
    await asyncio.to_thread(input)

    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

    try:
        await drone.offboard.start()
    except OffboardError as exc:
        raise RuntimeError(f"Failed to start Offboard mode: {exc}") from exc

    print("Offboard started")
    print("Front obstacle closer than stop distance will trigger a gentle backward move.")

    try:
        while True:
            if state.error:
                raise RuntimeError(f"Lidar error: {state.error}")

            distance = state.distance_m

            if distance is None:
                vx = 0.0
                mode = "WAITING"
            elif distance < args.stop_distance:
                vx = -args.back_speed
                mode = "BACKING"
            elif distance > args.clear_distance:
                vx = 0.0
                mode = "CLEAR"
            else:
                vx = 0.0
                mode = "HOLD"

            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(vx, 0.0, 0.0, 0.0)
            )

            if distance is None:
                print("LIDAR: no front reading yet")
            else:
                print(f"LIDAR front={distance:.2f} m | mode={mode} | vx={vx:.2f} m/s")

            await asyncio.sleep(1.0 / args.control_hz)

    except KeyboardInterrupt:
        print("Stopping...")

    finally:
        try:
            await drone.offboard.stop()
        except Exception:
            pass

        try:
            await drone.action.land()
        except Exception:
            pass

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--lidar-port", default="/dev/ttyUSB0")
    parser.add_argument("--lidar-baudrate", type=int, default=115200)
    parser.add_argument("--mavsdk-address", default="serial:///dev/ttyACM0:921600")
    parser.add_argument("--cone-deg", type=float, default=25.0)
    parser.add_argument("--stop-distance", type=float, default=1.5)
    parser.add_argument("--clear-distance", type=float, default=2.0)
    parser.add_argument("--back-speed", type=float, default=0.25)
    parser.add_argument("--control-hz", type=float, default=10.0)
    return parser.parse_args()

if __name__ == "__main__":
    asyncio.run(main_async(parse_args()))