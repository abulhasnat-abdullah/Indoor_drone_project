#!/usr/bin/env python3
import asyncio
import io
import json
import subprocess
import threading
import time

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, StreamingResponse
from PIL import Image
import uvicorn

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String, Float32

# --- ROS 2 Constants & Limits ---
VEL_LIMIT_XY = 3.0
YAW_RATE_LIMIT = 2.0
XY_VELOCITY_DEFAULT = 0.8
YAW_RATE_DEFAULT = 1.5
BOOST_MULTIPLIER = 1.5
XY_VELOCITY_MIN = 0.1
YAW_RATE_MIN = 0.1

# How long an analog stick input (touch joystick) stays valid without a
# refresh before it's treated as centered. Acts as a fail-safe: if a touch
# event is missed (dropped connection, browser backgrounding the tab), the
# robot stops instead of coasting on a stale command.
AXES_TIMEOUT = 0.35

# Altitude / takeoff height are clamped here as a last line of defense before
# anything reaches the offboard controller. Tune these to the space you fly in.
ALTITUDE_MIN = 0.3
ALTITUDE_MAX = 8.0
TAKEOFF_HEIGHT_MIN = 0.3
TAKEOFF_HEIGHT_MAX = 5.0
TAKEOFF_HEIGHT_DEFAULT = 1.0

# --- ROS 2 Node Setup ---
class WebTeleopNode(Node):
    def __init__(self):
        super().__init__('web_teleop_node')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.vel_pub = self.create_publisher(Twist, '/offboard_velocity_cmd', qos)
        self.arm_pub = self.create_publisher(Bool, '/arm_message', qos)
        # One-shot trigger to start/cancel the controlled landing sequence in
        # the offboard node. True = begin descent, False = cancel and hold
        # hover at current altitude. Separate from /arm_message on purpose:
        # arm/disarm is a hard motor e-stop, land is a soft, cancellable
        # descent -- they must never share a topic.
        self.land_pub = self.create_publisher(Bool, '/land_message', qos)
        # Real-time hover altitude target while flying.
        self.altitude_pub = self.create_publisher(Float32, '/offboard_altitude_setpoint', qos)
        # Height used for the next takeoff; retained (transient local) so the
        # offboard controller sees it even if it comes up after this node.
        self.takeoff_height_pub = self.create_publisher(Float32, '/takeoff_height_cmd', qos)
        self.create_subscription(String, '/px4_offboard/status', self.status_cb, 10)

        self.mutex = threading.Lock()
        self.held_keys = set()
        self.xy_speed = XY_VELOCITY_DEFAULT
        self.yaw_speed = YAW_RATE_DEFAULT
        self.latest_status = {}
        self.last_status_time = None
        self.takeoff_height = TAKEOFF_HEIGHT_DEFAULT
        self.altitude_setpoint = TAKEOFF_HEIGHT_DEFAULT

        # Analog stick input (on-screen joystick). Kept separate from
        # held_keys so touch and keyboard control never fight each other -
        # whichever was touched most recently wins, decided in _publish_twist.
        self.axes = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.axes_expiry = 0.0

        self.create_timer(1.0 / 20.0, self._publish_twist)

        # Publish the defaults once at startup so a late-joining offboard
        # node still picks up a sane takeoff height / altitude target.
        self._publish_takeoff_height()
        self._publish_altitude_setpoint()

    def status_cb(self, msg: String):
        parsed = {}
        for field in msg.data.split('|'):
            if '=' in field:
                k, v = field.split('=', 1)
                parsed[k] = v
        with self.mutex:
            self.latest_status = parsed
            self.last_status_time = time.time()

    def _publish_twist(self):
        with self.mutex:
            held = set(self.held_keys)
            xy = self.xy_speed
            yaw = self.yaw_speed
            axes = dict(self.axes)
            axes_fresh = time.time() < self.axes_expiry

        if 'boost' in held:
            xy = min(xy * BOOST_MULTIPLIER, VEL_LIMIT_XY)
            yaw = min(yaw * BOOST_MULTIPLIER, YAW_RATE_LIMIT)

        vx = vy = yaw_rate = 0.0
        if axes_fresh and (axes['x'] or axes['y'] or axes['yaw']):
            # On-screen joystick is actively being dragged - it takes
            # priority and gives proportional (not just on/off) control.
            vx = axes['y'] * xy
            vy = -axes['x'] * xy
            yaw_rate = -axes['yaw'] * yaw
        else:
            if 'up' in held: vx += xy
            if 'down' in held: vx -= xy
            if 'left' in held: vy += xy
            if 'right' in held: vy -= xy
            if 'yaw_left' in held: yaw_rate += yaw
            if 'yaw_right' in held: yaw_rate -= yaw

        t = Twist()
        t.linear.x = vx
        t.linear.y = vy
        t.angular.z = yaw_rate
        self.vel_pub.publish(t)

    def _publish_takeoff_height(self):
        msg = Float32()
        msg.data = self.takeoff_height
        self.takeoff_height_pub.publish(msg)

    def _publish_altitude_setpoint(self):
        msg = Float32()
        msg.data = self.altitude_setpoint
        self.altitude_pub.publish(msg)

    def set_keys(self, keys: list):
        with self.mutex:
            self.held_keys = set(keys)

    def set_axes(self, x, y, yaw):
        """Update the analog joystick vector. x/y/yaw are expected in
        [-1, 1]; out-of-range or malformed values are clamped/ignored
        rather than raising, since this comes straight off touch input."""
        try:
            x = max(-1.0, min(1.0, float(x)))
            y = max(-1.0, min(1.0, float(y)))
            yaw = max(-1.0, min(1.0, float(yaw)))
        except (TypeError, ValueError):
            return
        with self.mutex:
            self.axes = {'x': x, 'y': y, 'yaw': yaw}
            self.axes_expiry = time.time() + AXES_TIMEOUT

    def release_axes(self):
        with self.mutex:
            self.axes = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
            self.axes_expiry = 0.0

    def set_boost(self, active: bool):
        """Touch-friendly equivalent of holding Shift."""
        with self.mutex:
            if active:
                self.held_keys.add('boost')
            else:
                self.held_keys.discard('boost')

    def set_speed_settings(self, xy=None, yaw=None):
        with self.mutex:
            if xy is not None:
                try:
                    self.xy_speed = max(XY_VELOCITY_MIN, min(VEL_LIMIT_XY, float(xy)))
                except (TypeError, ValueError):
                    pass
            if yaw is not None:
                try:
                    self.yaw_speed = max(YAW_RATE_MIN, min(YAW_RATE_LIMIT, float(yaw)))
                except (TypeError, ValueError):
                    pass
            return self._speed_settings_locked()

    def get_speed_settings(self):
        with self.mutex:
            return self._speed_settings_locked()

    def _speed_settings_locked(self):
        """Caller must hold self.mutex."""
        return {
            'xy_speed': self.xy_speed,
            'yaw_speed': self.yaw_speed,
            'xy_min': XY_VELOCITY_MIN,
            'xy_limit': VEL_LIMIT_XY,
            'yaw_min': YAW_RATE_MIN,
            'yaw_limit': YAW_RATE_LIMIT,
            'boost_multiplier': BOOST_MULTIPLIER,
        }

    def send_arm(self, arm_state: bool):
        msg = Bool()
        msg.data = arm_state
        self.arm_pub.publish(msg)

    def send_land(self, land_state: bool):
        """land_state=True starts the controlled descent; False cancels an
        in-progress landing and returns to hover at the current altitude."""
        msg = Bool()
        msg.data = land_state
        self.land_pub.publish(msg)

    def set_takeoff_height(self, height) -> float:
        try:
            height = float(height)
        except (TypeError, ValueError):
            height = TAKEOFF_HEIGHT_DEFAULT
        height = max(TAKEOFF_HEIGHT_MIN, min(TAKEOFF_HEIGHT_MAX, height))
        with self.mutex:
            self.takeoff_height = height
        self._publish_takeoff_height()
        return height

    def set_altitude_setpoint(self, altitude) -> float:
        try:
            altitude = float(altitude)
        except (TypeError, ValueError):
            altitude = self.altitude_setpoint
        altitude = max(ALTITUDE_MIN, min(ALTITUDE_MAX, altitude))
        with self.mutex:
            self.altitude_setpoint = altitude
        self._publish_altitude_setpoint()
        return altitude

    def get_flight_settings(self):
        with self.mutex:
            return {
                'takeoff_height': self.takeoff_height,
                'altitude_setpoint': self.altitude_setpoint,
                'takeoff_height_min': TAKEOFF_HEIGHT_MIN,
                'takeoff_height_max': TAKEOFF_HEIGHT_MAX,
                'altitude_min': ALTITUDE_MIN,
                'altitude_max': ALTITUDE_MAX,
            }

    def get_status(self):
        with self.mutex:
            st = dict(self.latest_status)
            age = None if self.last_status_time is None else time.time() - self.last_status_time
        return st, age


# --- FastAPI Application ---
app = FastAPI()
ros_node: WebTeleopNode = None

# --- Camera Settings & Streaming ---
# All camera stream parameters live in one dict so a single lock protects
# them and the capture thread can snapshot them atomically before building
# the rpicam-vid command line.
camera_settings_lock = threading.Lock()
camera_settings = {
    "rotation": 180,
    "width": 640,
    "height": 480,
    "fps": 15,
    "quality": 80,     # JPEG quality for the mjpeg encoder (1-100)
    "bitrate": None,   # bits/sec; only honored by some encoders, mainly h264
}
VALID_ROTATIONS = (0, 90, 180, 270)
VALID_RESOLUTIONS = ((640, 480), (1280, 720), (1920, 1080))
FPS_MIN, FPS_MAX = 5, 30
QUALITY_MIN, QUALITY_MAX = 10, 100


def update_camera_settings(rotation=None, width=None, height=None, fps=None,
                            quality=None, bitrate=None):
    """Validate + apply any provided camera settings and report whether the
    capture pipeline needs to restart to pick them up. Rotation is applied
    per-frame in software, so it never needs a restart."""
    needs_restart = False
    with camera_settings_lock:
        if rotation is not None and int(rotation) in VALID_ROTATIONS:
            camera_settings["rotation"] = int(rotation)

        if width is not None and height is not None:
            res = (int(width), int(height))
            if res in VALID_RESOLUTIONS and res != (camera_settings["width"], camera_settings["height"]):
                camera_settings["width"], camera_settings["height"] = res
                needs_restart = True

        if fps is not None:
            fps = max(FPS_MIN, min(FPS_MAX, int(fps)))
            if fps != camera_settings["fps"]:
                camera_settings["fps"] = fps
                needs_restart = True

        if quality is not None:
            quality = max(QUALITY_MIN, min(QUALITY_MAX, int(quality)))
            if quality != camera_settings["quality"]:
                camera_settings["quality"] = quality
                needs_restart = True

        if bitrate is not None:
            # 0 / empty clears the manual bitrate and lets the encoder pick.
            bitrate = int(bitrate) if bitrate else None
            if bitrate != camera_settings["bitrate"]:
                camera_settings["bitrate"] = bitrate
                needs_restart = True

        settings_copy = dict(camera_settings)

    if needs_restart:
        camera_stream.request_restart()
    return settings_copy


class CameraStream:
    """Owns a single rpicam-vid subprocess and republishes the latest frame
    to any number of viewers. Changing resolution/fps/quality/bitrate
    restarts the subprocess in place without dropping the /video_feed
    connections already open in browsers."""

    def __init__(self):
        self._condition = threading.Condition()
        self._frame = None
        self._frame_id = 0
        self._stop = threading.Event()
        self._restart = threading.Event()
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._restart.set()

    def request_restart(self):
        self._restart.set()

    def get_frame(self, last_id=0, timeout=2.0):
        with self._condition:
            if self._frame_id == last_id:
                self._condition.wait(timeout=timeout)
            return self._frame, self._frame_id

    @staticmethod
    def _build_cmd(settings):
        cmd = [
            "rpicam-vid",
            "--width", str(settings["width"]),
            "--height", str(settings["height"]),
            "--framerate", str(settings["fps"]),
            "--codec", "mjpeg",
            "--quality", str(settings["quality"]),
            "-o", "-", "-n", "-t", "0",
        ]
        if settings.get("bitrate"):
            cmd += ["--bitrate", str(settings["bitrate"])]
        return cmd

    def _run(self):
        consecutive_empty_starts = 0
        while not self._stop.is_set():
            with camera_settings_lock:
                settings = dict(camera_settings)

            try:
                proc = subprocess.Popen(
                    self._build_cmd(settings),
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0
                )
            except FileNotFoundError:
                print("[camera] rpicam-vid not found on this machine — camera stream disabled.")
                time.sleep(2.0)
                continue
            except Exception as e:
                print(f"[camera] failed to launch rpicam-vid: {e!r}")
                time.sleep(2.0)
                continue

            self._restart.clear()
            buf = b""
            got_any_frame = False
            try:
                while not self._stop.is_set() and not self._restart.is_set():
                    chunk = proc.stdout.read(4096)
                    if not chunk:
                        break
                    got_any_frame = True
                    buf += chunk
                    start = buf.find(b"\xff\xd8")
                    end = buf.find(b"\xff\xd9")
                    if start != -1 and end != -1 and end > start:
                        jpg = buf[start:end + 2]
                        buf = buf[end + 2:]

                        with camera_settings_lock:
                            rotation = camera_settings["rotation"]
                        if rotation:
                            try:
                                img = Image.open(io.BytesIO(jpg))
                                img = img.rotate(-rotation, expand=True)
                                out = io.BytesIO()
                                img.save(out, format="JPEG", quality=85)
                                jpg = out.getvalue()
                            except Exception:
                                # Malformed/partial frame: skip rotation for
                                # this one rather than dropping the stream.
                                pass

                        with self._condition:
                            self._frame = jpg
                            self._frame_id += 1
                            self._condition.notify_all()
            finally:
                proc.terminate()
                try:
                    proc.wait(timeout=1.0)
                except Exception:
                    proc.kill()

            if not got_any_frame and not self._stop.is_set():
                consecutive_empty_starts += 1
                stderr_out = b""
                try:
                    stderr_out = proc.stderr.read() or b""
                except Exception:
                    pass
                if stderr_out.strip():
                    print(f"[camera] rpicam-vid produced no frames, stderr: {stderr_out.decode(errors='replace').strip()}")
                elif consecutive_empty_starts == 1:
                    print("[camera] rpicam-vid produced no frames (no stderr output). "
                          "Is another process already using the camera? Check with: ps aux | grep rpicam")
                # Back off so a persistently failing camera doesn't spin the CPU
                # respawning the process dozens of times a second.
                time.sleep(min(1.0 * consecutive_empty_starts, 5.0))
            else:
                consecutive_empty_starts = 0


camera_stream = CameraStream()


def generate_camera_mjpeg():
    last_id = 0
    while True:
        frame, last_id = camera_stream.get_frame(last_id)
        if frame is None:
            time.sleep(0.05)
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.get("/video_feed")
def video_feed():
    return StreamingResponse(
        generate_camera_mjpeg(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

# --- Embedded Web Dashboard ---
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>Drone Ground Control</title>
<style>
    :root {
        --bg: #12151a;
        --panel: #1a1f27;
        --panel-raised: #202631;
        --hairline: #2b323d;
        --text: #e7ebf2;
        --text-dim: #8b93a3;
        --accent: #4fd1c5;
        --accent-dim: #2c6b66;
        --amber: #f0a742;
        --amber-dim: #6b4a12;
        --green: #3fb950;
        --red: #e5484d;
        --mono: ui-monospace, "SF Mono", "Cascadia Code", "Roboto Mono", Consolas, monospace;
        --sans: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    }

    * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; }

    body {
        background: radial-gradient(circle at 50% 0%, #171c24 0%, var(--bg) 60%);
        color: var(--text);
        font-family: var(--sans);
        margin: 0;
        min-height: 100vh;
        overscroll-behavior-y: contain;
    }

    /* --- Sticky header: annunciator + tab bar --- */
    .gcs-header {
        position: sticky;
        top: 0;
        z-index: 10;
        background: #0d1014;
        border-bottom: 1px solid var(--hairline);
    }

    .annunciator {
        display: flex;
        align-items: center;
        gap: 18px;
        padding: 10px 22px;
        flex-wrap: wrap;
    }

    .annunciator h1 {
        font-size: 14px;
        font-weight: 700;
        letter-spacing: 0.14em;
        text-transform: uppercase;
        margin: 0;
        margin-right: 8px;
        color: var(--text);
        white-space: nowrap;
    }

    .lamp {
        display: flex;
        align-items: center;
        gap: 7px;
        padding: 5px 12px;
        border-radius: 3px;
        background: var(--panel);
        border: 1px solid var(--hairline);
        font-family: var(--mono);
        font-size: 11px;
        letter-spacing: 0.08em;
        text-transform: uppercase;
        color: var(--text-dim);
    }

    .lamp .dot {
        width: 8px;
        height: 8px;
        border-radius: 50%;
        background: #3a4150;
        box-shadow: none;
        transition: background 0.2s, box-shadow 0.2s;
    }

    .lamp.on .dot { background: var(--green); box-shadow: 0 0 8px var(--green); }
    .lamp.warn .dot { background: var(--amber); box-shadow: 0 0 8px var(--amber); }
    .lamp.off .dot { background: var(--red); box-shadow: 0 0 8px var(--red); }
    .lamp.on, .lamp.warn, .lamp.off { color: var(--text); }

    /* --- Tab bar --- */
    .tabbar {
        display: flex;
        gap: 2px;
        padding: 0 16px;
        overflow-x: auto;
        scrollbar-width: none;
    }
    .tabbar::-webkit-scrollbar { display: none; }

    .tab-btn {
        background: transparent;
        border: none;
        border-bottom: 2px solid transparent;
        color: var(--text-dim);
        font-family: var(--mono);
        font-size: 12px;
        letter-spacing: 0.08em;
        text-transform: uppercase;
        padding: 12px 16px;
        cursor: pointer;
        white-space: nowrap;
        transition: color 0.15s, border-color 0.15s;
    }
    .tab-btn:hover { color: var(--text); }
    .tab-btn.active { color: var(--accent); border-bottom-color: var(--accent); }

    .tab-panel { display: none; }
    .tab-panel.active { display: block; }

    /* --- Layout --- */
    .page {
        max-width: 1200px;
        margin: 0 auto;
        padding: 22px;
    }

    .grid-2 {
        display: grid;
        grid-template-columns: 1.3fr 1fr;
        gap: 18px;
    }
    @media (max-width: 880px) {
        .grid-2 { grid-template-columns: 1fr; }
    }

    .panel {
        background: var(--panel);
        border: 1px solid var(--hairline);
        border-radius: 10px;
        padding: 18px 20px;
    }

    .panel h2 {
        font-size: 11px;
        letter-spacing: 0.14em;
        text-transform: uppercase;
        color: var(--accent);
        margin: 0 0 14px 0;
        font-weight: 700;
    }
    .panel h2 .h2-sub {
        color: var(--text-dim);
        font-weight: 500;
        text-transform: none;
        letter-spacing: normal;
        font-size: 11px;
    }

    .stack { display: flex; flex-direction: column; gap: 14px; }
    .row { display: flex; gap: 10px; flex-wrap: wrap; align-items: center; }

    /* --- Camera --- */
    #camera-wrap, #camera-wrap-home {
        position: relative;
        background: #000;
        border-radius: 8px;
        overflow: hidden;
        border: 1px solid var(--hairline);
        aspect-ratio: 4 / 3;
        display: flex;
        align-items: center;
        justify-content: center;
    }
    .camera-wrap--thumb { max-width: 480px; }
    #camera-feed, #camera-feed-home {
        width: 100%;
        height: 100%;
        object-fit: contain;
        display: block;
    }
    #camera-placeholder, #camera-placeholder-home {
        position: absolute;
        inset: 0;
        display: flex;
        align-items: center;
        justify-content: center;
        font-family: var(--mono);
        font-size: 12px;
        letter-spacing: 0.06em;
        color: var(--text-dim);
        text-align: center;
        padding: 20px;
    }

    .field { display: flex; flex-direction: column; gap: 5px; flex: 1; min-width: 110px; }
    .field label {
        font-family: var(--mono);
        font-size: 10px;
        letter-spacing: 0.08em;
        text-transform: uppercase;
        color: var(--text-dim);
    }

    select, input[type="number"] {
        background: var(--panel-raised);
        border: 1px solid var(--hairline);
        color: var(--text);
        font-family: var(--mono);
        font-size: 13px;
        padding: 8px 10px;
        border-radius: 5px;
        width: 100%;
    }
    select:focus, input[type="number"]:focus, button:focus-visible, input[type="range"]:focus-visible {
        outline: 2px solid var(--accent);
        outline-offset: 1px;
    }

    label.checkbox {
        display: flex;
        align-items: center;
        gap: 6px;
        font-family: var(--mono);
        font-size: 11px;
        color: var(--text-dim);
        text-transform: uppercase;
        letter-spacing: 0.06em;
        white-space: nowrap;
    }

    button {
        padding: 9px 16px;
        font-weight: 600;
        font-size: 12px;
        letter-spacing: 0.05em;
        text-transform: uppercase;
        border-radius: 5px;
        cursor: pointer;
        border: 1px solid transparent;
        transition: filter 0.15s, transform 0.05s, background 0.15s, border-color 0.15s;
        font-family: var(--sans);
    }
    button:active { transform: translateY(1px); }
    button:hover { filter: brightness(1.12); }

    .btn-primary { background: var(--accent-dim); color: #d9fbf8; border-color: var(--accent); }
    .btn-arm { background: #1f5c30; color: #d9f7de; border-color: var(--green); }
    .btn-disarm { background: #5c1f24; color: #fbdadb; border-color: var(--red); }
    .btn-land { background: var(--amber-dim); color: #ffe8c2; border-color: var(--amber); }
    .btn-ghost { background: transparent; color: var(--text-dim); border-color: var(--hairline); }
    .btn-ghost:hover { color: var(--text); }
    .btn-toggle { background: var(--panel-raised); color: var(--text-dim); border-color: var(--hairline); }
    .btn-toggle.active { background: var(--accent-dim); color: #d9fbf8; border-color: var(--accent); }
    .btn-lg { padding: 14px 20px; font-size: 13px; }

    /* --- Telemetry readout --- */
    .tele-grid {
        display: grid;
        grid-template-columns: 1fr 1fr;
        gap: 10px;
    }
    .tele-item {
        background: var(--panel-raised);
        border: 1px solid var(--hairline);
        border-radius: 6px;
        padding: 10px 12px;
    }
    .tele-item .tele-label {
        font-family: var(--mono);
        font-size: 10px;
        color: var(--text-dim);
        text-transform: uppercase;
        letter-spacing: 0.08em;
    }
    .tele-item .tele-value {
        font-family: var(--mono);
        font-size: 18px;
        margin-top: 3px;
    }
    .tele-item.tele-compact { padding: 8px 10px; }
    .tele-item.tele-compact .tele-value { font-size: 15px; }

    /* --- Altitude tape (signature control) --- */
    .altitude-block {
        display: flex;
        gap: 16px;
        align-items: center;
        padding: 14px 4px;
    }
    .tape-scale {
        font-family: var(--mono);
        font-size: 10px;
        color: var(--text-dim);
        display: flex;
        flex-direction: column;
        justify-content: space-between;
        height: 220px;
        text-align: right;
    }
    .tape-track {
        position: relative;
        width: 34px;
        height: 220px;
        border-radius: 4px;
        background:
            repeating-linear-gradient(to bottom,
                var(--hairline) 0, var(--hairline) 1px,
                transparent 1px, transparent 21.9px),
            var(--panel-raised);
        border: 1px solid var(--hairline);
    }
    input[type="range"].tape {
        writing-mode: vertical-lr;
        direction: rtl;
        appearance: none;
        -webkit-appearance: none;
        width: 34px;
        height: 220px;
        background: transparent;
        margin: 0;
        position: absolute;
        top: 0; left: 0;
    }
    input[type="range"].tape::-webkit-slider-thumb {
        -webkit-appearance: none;
        width: 40px;
        height: 4px;
        background: var(--accent);
        box-shadow: 0 0 10px var(--accent);
        border-radius: 2px;
        cursor: grab;
        margin-left: -3px;
    }
    input[type="range"].tape::-moz-range-thumb {
        width: 40px;
        height: 4px;
        background: var(--accent);
        box-shadow: 0 0 10px var(--accent);
        border: none;
        border-radius: 2px;
        cursor: grab;
    }
    .altitude-readout {
        display: flex;
        flex-direction: column;
        gap: 10px;
        flex: 1;
    }
    .altitude-readout .big-value {
        font-family: var(--mono);
        font-size: 34px;
        color: var(--accent);
        line-height: 1;
    }
    .altitude-readout .big-value span { font-size: 15px; color: var(--text-dim); }
    .hint { font-size: 11px; color: var(--text-dim); line-height: 1.5; }

    .chip-row { display: flex; gap: 8px; flex-wrap: wrap; }
    .chip {
        font-family: var(--mono);
        font-size: 11px;
        padding: 5px 10px;
        border-radius: 999px;
        border: 1px solid var(--hairline);
        background: var(--panel-raised);
        color: var(--text-dim);
        cursor: pointer;
    }
    .chip:hover { border-color: var(--accent); color: var(--accent); }

    kbd {
        font-family: var(--mono);
        background: var(--panel-raised);
        border: 1px solid var(--hairline);
        border-bottom-width: 2px;
        border-radius: 4px;
        padding: 1px 6px;
        font-size: 11px;
    }

    /* --- Plain horizontal sliders (Settings tab) --- */
    input[type="range"]:not(.tape) {
        width: 100%;
        accent-color: var(--accent);
        background: transparent;
        height: 24px;
    }

    /* --- Joysticks (Control tab) --- */
    .joystick-row {
        display: flex;
        gap: 32px;
        justify-content: center;
        flex-wrap: wrap;
        padding: 12px 0 6px;
    }
    .joystick-block {
        display: flex;
        flex-direction: column;
        align-items: center;
        gap: 10px;
    }
    .joystick-label {
        font-family: var(--mono);
        font-size: 10px;
        letter-spacing: 0.1em;
        text-transform: uppercase;
        color: var(--text-dim);
    }
    .joystick-base {
        position: relative;
        width: 168px;
        height: 168px;
        border-radius: 50%;
        background: var(--panel-raised);
        border: 1px solid var(--hairline);
        touch-action: none;
        user-select: none;
    }
    .joystick-base::before, .joystick-base::after {
        content: '';
        position: absolute;
        background: var(--hairline);
    }
    .joystick-base::before { left: 50%; top: 8%; bottom: 8%; width: 1px; }
    .joystick-base::after { top: 50%; left: 8%; right: 8%; height: 1px; }
    .joystick-base--yaw {
        width: 168px;
        height: 72px;
        border-radius: 36px;
    }
    .joystick-base--yaw::before { display: none; }
    .joystick-knob {
        position: absolute;
        left: 50%;
        top: 50%;
        width: 56px;
        height: 56px;
        border-radius: 50%;
        background: var(--accent-dim);
        border: 1px solid var(--accent);
        transform: translate(-50%, -50%);
        box-shadow: 0 0 14px rgba(79, 209, 197, 0.35);
        touch-action: none;
        pointer-events: none;
        transition: background 0.1s;
    }
    .joystick-base.dragging .joystick-knob {
        background: var(--accent);
    }

    /* --- D-pad --- */
    .dpad-wrap { display: flex; flex-direction: column; align-items: center; gap: 8px; }
    .dpad {
        display: grid;
        grid-template-columns: repeat(3, 52px);
        grid-template-rows: repeat(3, 52px);
        gap: 6px;
    }
    .dpad-btn {
        display: flex;
        align-items: center;
        justify-content: center;
        background: var(--panel-raised);
        border: 1px solid var(--hairline);
        border-radius: 8px;
        color: var(--text);
        font-size: 18px;
        cursor: pointer;
        user-select: none;
        touch-action: none;
    }
    .dpad-btn.pressed { background: var(--accent-dim); border-color: var(--accent); }
    .dpad-empty { visibility: hidden; }
    .yaw-btn-row { display: flex; gap: 8px; }
    .yaw-btn {
        display: flex;
        align-items: center;
        justify-content: center;
        width: 78px;
        height: 44px;
        background: var(--panel-raised);
        border: 1px solid var(--hairline);
        border-radius: 8px;
        color: var(--text);
        font-family: var(--mono);
        font-size: 12px;
        cursor: pointer;
        user-select: none;
        touch-action: none;
    }
    .yaw-btn.pressed { background: var(--accent-dim); border-color: var(--accent); }

    .divider-label {
        display: flex;
        align-items: center;
        gap: 12px;
        color: var(--text-dim);
        font-family: var(--mono);
        font-size: 10px;
        letter-spacing: 0.1em;
        text-transform: uppercase;
        margin: 20px 0 16px;
    }
    .divider-label::before, .divider-label::after {
        content: '';
        flex: 1;
        height: 1px;
        background: var(--hairline);
    }

    .control-panel-grid {
        display: grid;
        grid-template-columns: 1fr 1fr;
        gap: 24px;
        align-items: start;
    }
    @media (max-width: 720px) {
        .control-panel-grid { grid-template-columns: 1fr; }
    }

    /* --- Toasts --- */
    #toast-stack {
        position: fixed;
        bottom: 18px;
        right: 18px;
        display: flex;
        flex-direction: column;
        gap: 8px;
        z-index: 100;
    }
    .toast {
        font-family: var(--mono);
        font-size: 12px;
        background: var(--panel-raised);
        border: 1px solid var(--accent-dim);
        color: var(--text);
        padding: 10px 14px;
        border-radius: 6px;
        box-shadow: 0 6px 18px rgba(0,0,0,0.4);
        animation: toast-in 0.15s ease-out;
    }
    @keyframes toast-in {
        from { opacity: 0; transform: translateY(6px); }
        to { opacity: 1; transform: translateY(0); }
    }
</style>
</head>
<body>

    <div class="gcs-header">
        <div class="annunciator">
            <h1>Drone / GCS</h1>
            <div class="lamp off" id="lamp-conn"><span class="dot"></span><span id="lamp-conn-text">Link</span></div>
            <div class="lamp off" id="lamp-armed"><span class="dot"></span><span id="lamp-armed-text">Armed</span></div>
            <div class="lamp off" id="lamp-state"><span class="dot"></span><span id="lamp-state-text">State --</span></div>
        </div>
        <nav class="tabbar">
            <button class="tab-btn active" data-tab="home">Home</button>
            <button class="tab-btn" data-tab="camera">Camera</button>
            <button class="tab-btn" data-tab="flight">Flight</button>
            <button class="tab-btn" data-tab="control">Control</button>
            <button class="tab-btn" data-tab="settings">Settings</button>
        </nav>
    </div>

    <!-- ===================== HOME ===================== -->
    <section class="tab-panel active" data-tab="home">
        <div class="page grid-2">
            <div class="stack">
                <div class="panel">
                    <h2>Camera</h2>
                    <div id="camera-wrap-home" class="camera-wrap--thumb">
                        <img id="camera-feed-home" alt="Camera stream"
                             onload="onCameraFrame('home')" onerror="onCameraError('home')">
                        <div id="camera-placeholder-home">Waiting for camera stream&hellip;</div>
                    </div>
                </div>
                <div class="panel">
                    <h2>Telemetry</h2>
                    <div class="tele-grid">
                        <div class="tele-item">
                            <div class="tele-label">Armed</div>
                            <div class="tele-value" id="tel-armed">--</div>
                        </div>
                        <div class="tele-item">
                            <div class="tele-label">Flight State</div>
                            <div class="tele-value" id="tel-state">--</div>
                        </div>
                        <div class="tele-item" style="grid-column: 1 / -1;">
                            <div class="tele-label">Local Altitude (Z)</div>
                            <div class="tele-value" id="tel-alt">--</div>
                        </div>
                    </div>
                </div>
                <div class="panel">
                    <h2>Quick Controls</h2>
                    <div class="row">
                        <button class="btn-arm" onclick="sendArm(true)">Arm / Takeoff</button>
                        <button class="btn-land" onclick="sendLand(true)">Land</button>
                        <button class="btn-ghost" onclick="sendLand(false)">Cancel Land</button>
                        <button class="btn-disarm" onclick="sendArm(false)">Emergency Disarm</button>
                    </div>
                    <div class="hint" style="margin-top:12px;">
                        <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> move &nbsp;
                        <kbd>Q</kbd><kbd>E</kbd> yaw &nbsp;
                        <kbd>Shift</kbd> boost &nbsp;
                        <kbd>Space</kbd> arm &nbsp;
                        <kbd>L</kbd> land &nbsp;
                        <kbd>Esc</kbd> emergency disarm<br>
                        Works right here on Home &mdash; click the page first so it can capture keyboard input.<br>
                        Full stick / D-pad touch controls live on the Control tab.
                    </div>
                </div>
            </div>
            <div class="stack">
                <div class="panel">
                    <h2>Altitude</h2>
                    <div class="altitude-block">
                        <div class="tape-scale">
                            <span id="tape-max-label-home">8.0m</span>
                            <span></span>
                            <span id="tape-min-label-home">0.3m</span>
                        </div>
                        <div class="tape-track">
                            <input type="range" class="tape" id="altitude-slider-home"
                                   min="0.3" max="8.0" step="0.1" value="1.0"
                                   oninput="onAltitudeInput(this.value)">
                        </div>
                        <div class="altitude-readout">
                            <div>
                                <div class="tele-label">Hover Setpoint (live)</div>
                                <div class="big-value" id="home-altitude-readout">1.0<span>m</span></div>
                            </div>
                            <div class="hint">Drag while flying to retarget hover altitude in real time.</div>
                        </div>
                    </div>

                    <hr style="border-color: var(--hairline); margin: 16px 0;">

                    <div class="field" style="max-width: 220px;">
                        <label for="in-takeoff-height-home">Takeoff Height (m)</label>
                        <input type="number" id="in-takeoff-height-home" min="0.3" max="5.0" step="0.1" value="1.0">
                    </div>
                    <div class="chip-row" style="margin-top:8px;">
                        <span class="chip" onclick="setTakeoffHeightChip(0.5, 'in-takeoff-height-home')">0.5m</span>
                        <span class="chip" onclick="setTakeoffHeightChip(1.0, 'in-takeoff-height-home')">1.0m</span>
                        <span class="chip" onclick="setTakeoffHeightChip(1.5, 'in-takeoff-height-home')">1.5m</span>
                        <span class="chip" onclick="setTakeoffHeightChip(2.0, 'in-takeoff-height-home')">2.0m</span>
                    </div>
                    <div class="row" style="margin-top:12px;">
                        <button class="btn-primary" onclick="applyTakeoffHeight('in-takeoff-height-home')">Set Takeoff Height</button>
                    </div>
                    <div class="hint" style="margin-top:8px;">Applies before the next arm. Sends now, retained for the offboard controller.</div>
                </div>
            </div>
        </div>
    </section>

    <!-- ===================== CAMERA ===================== -->
    <section class="tab-panel" data-tab="camera">
        <div class="page">
            <div class="panel">
                <h2>Camera <span class="h2-sub">&mdash; stream only runs while this tab is open</span></h2>
                <div id="camera-wrap">
                    <img id="camera-feed" alt="Camera stream"
                         onload="onCameraFrame('main')" onerror="onCameraError('main')">
                    <div id="camera-placeholder">Waiting for camera stream&hellip;<br>Check the server terminal if this doesn't clear.</div>
                </div>
                <div class="row" style="margin-top:14px;">
                    <div class="field">
                        <label for="sel-rotation">Rotation</label>
                        <select id="sel-rotation" onchange="applyRotation(this.value)">
                            <option value="0">0&deg;</option>
                            <option value="90">90&deg;</option>
                            <option value="180" selected>180&deg;</option>
                            <option value="270">270&deg;</option>
                        </select>
                    </div>
                    <div class="field">
                        <label for="sel-resolution">Resolution</label>
                        <select id="sel-resolution">
                            <option value="640x480">640 x 480</option>
                            <option value="1280x720">1280 x 720</option>
                            <option value="1920x1080">1920 x 1080</option>
                        </select>
                    </div>
                    <div class="field">
                        <label for="in-fps">FPS</label>
                        <input type="number" id="in-fps" min="5" max="30" value="15">
                    </div>
                    <div class="field">
                        <label for="in-quality">Quality</label>
                        <input type="number" id="in-quality" min="10" max="100" value="80">
                    </div>
                    <div class="field">
                        <label for="in-bitrate">Bitrate (kbps)</label>
                        <input type="number" id="in-bitrate" min="0" placeholder="auto">
                    </div>
                </div>
                <div class="row" style="margin-top:12px;">
                    <span class="chip" onclick="applyLowBandwidthPreset()">Low bandwidth preset</span>
                </div>
                <div class="row" style="margin-top:12px; justify-content: space-between;">
                    <span class="hint">Bitrate mainly affects h264 encoders; leave blank for auto on mjpeg.<br>
                        Memory tip: the video connection closes automatically whenever you leave the Home or Camera tab, and reopens when you come back.</span>
                    <button class="btn-primary" onclick="applyCameraSettings()">Apply Stream Settings</button>
                </div>
            </div>
        </div>
    </section>

    <!-- ===================== FLIGHT ===================== -->
    <section class="tab-panel" data-tab="flight">
        <div class="page grid-2">
            <div class="stack">
                <div class="panel">
                    <h2>Altitude</h2>
                    <div class="altitude-block">
                        <div class="tape-scale">
                            <span id="tape-max-label">8.0m</span>
                            <span></span>
                            <span id="tape-min-label">0.3m</span>
                        </div>
                        <div class="tape-track">
                            <input type="range" class="tape" id="altitude-slider"
                                   min="0.3" max="8.0" step="0.1" value="1.0"
                                   oninput="onAltitudeInput(this.value)">
                        </div>
                        <div class="altitude-readout">
                            <div>
                                <div class="tele-label">Hover Setpoint (live)</div>
                                <div class="big-value" id="altitude-readout">1.0<span>m</span></div>
                            </div>
                            <div class="hint">Drag while flying to retarget hover altitude in real time.</div>
                        </div>
                    </div>

                    <hr style="border-color: var(--hairline); margin: 16px 0;">

                    <div class="field" style="max-width: 220px;">
                        <label for="in-takeoff-height">Takeoff Height (m)</label>
                        <input type="number" id="in-takeoff-height" min="0.3" max="5.0" step="0.1" value="1.0">
                    </div>
                    <div class="chip-row" style="margin-top:8px;">
                        <span class="chip" onclick="setTakeoffHeightChip(0.5, 'in-takeoff-height')">0.5m</span>
                        <span class="chip" onclick="setTakeoffHeightChip(1.0, 'in-takeoff-height')">1.0m</span>
                        <span class="chip" onclick="setTakeoffHeightChip(1.5, 'in-takeoff-height')">1.5m</span>
                        <span class="chip" onclick="setTakeoffHeightChip(2.0, 'in-takeoff-height')">2.0m</span>
                    </div>
                    <div class="row" style="margin-top:12px;">
                        <button class="btn-primary" onclick="applyTakeoffHeight('in-takeoff-height')">Set Takeoff Height</button>
                    </div>
                    <div class="hint" style="margin-top:8px;">Applies before the next arm. Sends now, retained for the offboard controller.</div>
                </div>
            </div>
            <div class="stack">
                <div class="panel">
                    <h2>Arm State</h2>
                    <div class="row">
                        <button class="btn-arm" onclick="sendArm(true)">Arm / Takeoff</button>
                        <button class="btn-land" onclick="sendLand(true)">Land</button>
                        <button class="btn-ghost" onclick="sendLand(false)">Cancel Land</button>
                        <button class="btn-disarm" onclick="sendArm(false)">Emergency Disarm</button>
                    </div>
                    <div class="hint" style="margin-top:10px;">
                        "Land" is a controlled, cancellable descent to the ground followed by an automatic disarm.
                        "Emergency Disarm" cuts motors immediately, wherever the vehicle is &mdash; use only if Land isn't safe or fast enough.
                    </div>
                </div>
                <div class="panel">
                    <h2>Telemetry</h2>
                    <div class="tele-grid">
                        <div class="tele-item tele-compact">
                            <div class="tele-label">Armed</div>
                            <div class="tele-value" id="tel-armed-flight">--</div>
                        </div>
                        <div class="tele-item tele-compact">
                            <div class="tele-label">State</div>
                            <div class="tele-value" id="tel-state-flight">--</div>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    </section>

    <!-- ===================== CONTROL ===================== -->
    <section class="tab-panel" data-tab="control">
        <div class="page">
            <div class="panel">
                <h2>Sticks <span class="h2-sub">&mdash; drag with finger or mouse</span></h2>
                <div class="joystick-row">
                    <div class="joystick-block">
                        <div class="joystick-label">Move</div>
                        <div class="joystick-base" id="stick-move-base">
                            <div class="joystick-knob" id="stick-move-knob"></div>
                        </div>
                        <div class="hint">Forward / back / strafe</div>
                    </div>
                    <div class="joystick-block">
                        <div class="joystick-label">Yaw</div>
                        <div class="joystick-base joystick-base--yaw" id="stick-yaw-base">
                            <div class="joystick-knob" id="stick-yaw-knob"></div>
                        </div>
                        <div class="hint">Rotate left / right</div>
                    </div>
                </div>

                <div class="divider-label">or use buttons</div>

                <div class="control-panel-grid">
                    <div class="dpad-wrap">
                        <div class="dpad">
                            <div class="dpad-empty"></div>
                            <div class="dpad-btn" id="btn-up">&#9650;</div>
                            <div class="dpad-empty"></div>
                            <div class="dpad-btn" id="btn-left">&#9664;</div>
                            <div class="dpad-empty"></div>
                            <div class="dpad-btn" id="btn-right">&#9654;</div>
                            <div class="dpad-empty"></div>
                            <div class="dpad-btn" id="btn-down">&#9660;</div>
                            <div class="dpad-empty"></div>
                        </div>
                        <div class="hint">Hold to move</div>
                    </div>
                    <div class="dpad-wrap">
                        <div class="yaw-btn-row">
                            <div class="yaw-btn" id="btn-yaw-left">&#8634; Yaw L</div>
                            <div class="yaw-btn" id="btn-yaw-right">Yaw R &#8635;</div>
                        </div>
                        <div class="row" style="margin-top:10px;">
                            <button class="btn-toggle" id="btn-boost" onclick="toggleBoost()">Boost: Off</button>
                        </div>
                        <div class="hint">Hold yaw buttons to rotate</div>
                    </div>
                </div>

                <div class="divider-label">arm / land</div>
                <div class="row">
                    <button class="btn-arm btn-lg" onclick="sendArm(true)">Arm / Takeoff</button>
                    <button class="btn-land btn-lg" onclick="sendLand(true)">Land</button>
                    <button class="btn-ghost btn-lg" onclick="sendLand(false)">Cancel Land</button>
                    <button class="btn-disarm btn-lg" onclick="sendArm(false)">Emergency Disarm</button>
                </div>

                <div class="hint" style="margin-top:16px;">
                    Keyboard still works from any tab:
                    <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> move &nbsp;
                    <kbd>Q</kbd><kbd>E</kbd> yaw &nbsp;
                    <kbd>Shift</kbd> boost &nbsp;
                    <kbd>Space</kbd> arm &nbsp;
                    <kbd>L</kbd> land &nbsp;
                    <kbd>Esc</kbd> emergency disarm<br>
                    Click the page first so it can capture keyboard input.
                </div>
            </div>
        </div>
    </section>

    <!-- ===================== SETTINGS ===================== -->
    <section class="tab-panel" data-tab="settings">
        <div class="page grid-2">
            <div class="stack">
                <div class="panel">
                    <h2>Control Speeds</h2>
                    <div class="field">
                        <label for="in-xy-speed">Move Speed &mdash; <span id="xy-speed-readout">--</span> m/s</label>
                        <input type="range" id="in-xy-speed" min="0.1" max="3.0" step="0.1" value="0.8"
                               oninput="onSpeedInput('xy', this.value)">
                    </div>
                    <div class="field" style="margin-top:14px;">
                        <label for="in-yaw-speed">Yaw Speed &mdash; <span id="yaw-speed-readout">--</span> rad/s</label>
                        <input type="range" id="in-yaw-speed" min="0.1" max="2.0" step="0.1" value="1.5"
                               oninput="onSpeedInput('yaw', this.value)">
                    </div>
                    <div class="hint" style="margin-top:10px;">Applies to keyboard, D-pad, and joystick alike. Boost multiplies both by <span id="boost-mult-readout">--</span>x, capped at the hard limits below.</div>
                </div>
                <div class="panel">
                    <h2>Hardware Info</h2>
                    <div class="hint">Interface: FastAPI + WebSocket, served on port 8000.<br>
                        Video: MJPEG over HTTP from <code>rpicam-vid</code>, one connection per open Home/Camera tab.</div>
                </div>
            </div>
            <div class="stack">
                <div class="panel">
                    <h2>Limits (read-only)</h2>
                    <div class="tele-grid">
                        <div class="tele-item tele-compact">
                            <div class="tele-label">Move Speed Limit</div>
                            <div class="tele-value" id="lim-xy">--</div>
                        </div>
                        <div class="tele-item tele-compact">
                            <div class="tele-label">Yaw Speed Limit</div>
                            <div class="tele-value" id="lim-yaw">--</div>
                        </div>
                        <div class="tele-item tele-compact">
                            <div class="tele-label">Takeoff Height Range</div>
                            <div class="tele-value" id="lim-takeoff">--</div>
                        </div>
                        <div class="tele-item tele-compact">
                            <div class="tele-label">Altitude Range</div>
                            <div class="tele-value" id="lim-altitude">--</div>
                        </div>
                    </div>
                    <div class="hint" style="margin-top:10px;">These are hard-coded server-side clamps, not adjustable from the browser.</div>
                </div>
            </div>
        </div>
    </section>

    <div id="toast-stack"></div>

    <script>
        let ws = new WebSocket(`ws://${location.host}/ws`);
        let activeKeys = new Set();
        const keyMap = {
            'KeyW': 'up', 'ArrowUp': 'up',
            'KeyS': 'down', 'ArrowDown': 'down',
            'KeyA': 'left', 'ArrowLeft': 'left',
            'KeyD': 'right', 'ArrowRight': 'right',
            'KeyQ': 'yaw_left', 'KeyE': 'yaw_right',
            'ShiftLeft': 'boost', 'ShiftRight': 'boost'
        };

        // Home and Flight tabs each have their own copy of the altitude
        // and takeoff-height controls; these lists let one update function
        // keep every copy in sync instead of hard-coding a single id.
        const altitudeSliderIds = ['altitude-slider', 'altitude-slider-home'];
        const altitudeReadoutIds = ['altitude-readout', 'home-altitude-readout'];
        const tapeMinIds = ['tape-min-label', 'tape-min-label-home'];
        const tapeMaxIds = ['tape-max-label', 'tape-max-label-home'];
        const takeoffHeightIds = ['in-takeoff-height', 'in-takeoff-height-home'];

        // ---------- Tabs ----------
        const cameraActive = { home: false, main: false };

        function showTab(tab) {
            document.querySelectorAll('.tab-btn').forEach(b => b.classList.toggle('active', b.dataset.tab === tab));
            document.querySelectorAll('.tab-panel').forEach(p => p.classList.toggle('active', p.dataset.tab === tab));
            setCameraActive('home', tab === 'home');
            setCameraActive('main', tab === 'camera');
        }
        document.querySelectorAll('.tab-btn').forEach(b => {
            b.addEventListener('click', () => showTab(b.dataset.tab));
        });

        // ---------- Camera (memory-efficient: only stream on visible tabs) ----------
        function setCameraActive(key, active) {
            cameraActive[key] = active;
            const img = document.getElementById(key === 'home' ? 'camera-feed-home' : 'camera-feed');
            const placeholder = document.getElementById(key === 'home' ? 'camera-placeholder-home' : 'camera-placeholder');
            if (active) {
                placeholder.style.display = 'flex';
                placeholder.innerHTML = 'Waiting for camera stream&hellip;';
                img.src = '/video_feed?_=' + Date.now();
            } else {
                img.removeAttribute('src');
                placeholder.style.display = 'flex';
                placeholder.innerHTML = 'Stream paused to save bandwidth.';
            }
        }

        function onCameraFrame(key) {
            const placeholder = document.getElementById(key === 'home' ? 'camera-placeholder-home' : 'camera-placeholder');
            placeholder.style.display = 'none';
        }

        function onCameraError(key) {
            if (!cameraActive[key]) return; // paused on purpose, don't retry
            const placeholder = document.getElementById(key === 'home' ? 'camera-placeholder-home' : 'camera-placeholder');
            placeholder.style.display = 'flex';
            placeholder.innerHTML = 'Camera stream disconnected, retrying&hellip;';
            setTimeout(() => {
                if (cameraActive[key]) {
                    document.getElementById(key === 'home' ? 'camera-feed-home' : 'camera-feed').src = '/video_feed?retry=' + Date.now();
                }
            }, 2000);
        }

        function applyLowBandwidthPreset() {
            document.getElementById('sel-resolution').value = '640x480';
            document.getElementById('in-fps').value = 10;
            document.getElementById('in-quality').value = 55;
            document.getElementById('in-bitrate').value = '';
            applyCameraSettings();
        }

        // ---------- Toasts / lamps ----------
        function toast(msg) {
            const stack = document.getElementById('toast-stack');
            const el = document.createElement('div');
            el.className = 'toast';
            el.textContent = msg;
            stack.appendChild(el);
            setTimeout(() => el.remove(), 3200);
        }

        function setLamp(id, textId, level, text) {
            const lamp = document.getElementById(id);
            lamp.className = 'lamp ' + level;
            document.getElementById(textId).textContent = text;
        }

        ws.onopen = () => setLamp('lamp-conn', 'lamp-conn-text', 'on', 'Link OK');
        ws.onclose = () => setLamp('lamp-conn', 'lamp-conn-text', 'off', 'No Link');

        ws.onmessage = (event) => {
            const data = JSON.parse(event.data);
            switch (data.type) {
                case 'telemetry': {
                    const armed = data.armed || '?';
                    const state = data.state || '?';
                    const alt = data.local_z || '?';
                    ['tel-armed', 'tel-armed-flight'].forEach(id => {
                        const el = document.getElementById(id);
                        if (el) el.innerText = armed;
                    });
                    ['tel-state', 'tel-state-flight'].forEach(id => {
                        const el = document.getElementById(id);
                        if (el) el.innerText = state;
                    });
                    const altEl = document.getElementById('tel-alt');
                    if (altEl) altEl.innerText = alt;
                    const isArmed = String(armed).toLowerCase() === 'true';
                    setLamp('lamp-armed', 'lamp-armed-text', isArmed ? 'warn' : 'off', isArmed ? 'Armed' : 'Disarmed');
                    setLamp('lamp-state', 'lamp-state-text', data.state ? 'on' : 'off', 'State ' + (data.state || '--'));
                    break;
                }
                case 'flight_settings': {
                    altitudeSliderIds.forEach(id => {
                        const el = document.getElementById(id);
                        el.min = data.altitude_min;
                        el.max = data.altitude_max;
                        el.value = data.altitude_setpoint;
                    });
                    tapeMinIds.forEach(id => { document.getElementById(id).innerText = data.altitude_min.toFixed(1) + 'm'; });
                    tapeMaxIds.forEach(id => { document.getElementById(id).innerText = data.altitude_max.toFixed(1) + 'm'; });
                    altitudeReadoutIds.forEach(id => {
                        document.getElementById(id).innerHTML = data.altitude_setpoint.toFixed(1) + '<span>m</span>';
                    });
                    takeoffHeightIds.forEach(id => {
                        const el = document.getElementById(id);
                        el.value = data.takeoff_height;
                        el.min = data.takeoff_height_min;
                        el.max = data.takeoff_height_max;
                    });
                    document.getElementById('lim-takeoff').innerText = data.takeoff_height_min.toFixed(1) + ' - ' + data.takeoff_height_max.toFixed(1) + 'm';
                    document.getElementById('lim-altitude').innerText = data.altitude_min.toFixed(1) + ' - ' + data.altitude_max.toFixed(1) + 'm';
                    break;
                }
                case 'camera_settings': {
                    document.getElementById('sel-rotation').value = data.rotation;
                    document.getElementById('sel-resolution').value = data.width + 'x' + data.height;
                    document.getElementById('in-fps').value = data.fps;
                    document.getElementById('in-quality').value = data.quality;
                    document.getElementById('in-bitrate').value = data.bitrate || '';
                    break;
                }
                case 'speed_settings':
                case 'speed_settings_ack': {
                    const xySlider = document.getElementById('in-xy-speed');
                    const yawSlider = document.getElementById('in-yaw-speed');
                    xySlider.min = data.xy_min; xySlider.max = data.xy_limit; xySlider.value = data.xy_speed;
                    yawSlider.min = data.yaw_min; yawSlider.max = data.yaw_limit; yawSlider.value = data.yaw_speed;
                    document.getElementById('xy-speed-readout').innerText = data.xy_speed.toFixed(1);
                    document.getElementById('yaw-speed-readout').innerText = data.yaw_speed.toFixed(1);
                    document.getElementById('boost-mult-readout').innerText = data.boost_multiplier.toFixed(1);
                    document.getElementById('lim-xy').innerText = data.xy_limit.toFixed(1) + ' m/s';
                    document.getElementById('lim-yaw').innerText = data.yaw_limit.toFixed(1) + ' rad/s';
                    if (data.type === 'speed_settings_ack') toast('Speed settings applied');
                    break;
                }
                case 'takeoff_height_ack':
                    toast(`Takeoff height set to ${data.height.toFixed(1)}m`);
                    break;
                case 'camera_settings_ack':
                    toast('Camera settings applied');
                    break;
            }
        };

        // ---------- Keyboard (works from any tab) ----------
        document.addEventListener('keydown', (e) => {
            if (e.repeat) return;
            if (e.code === 'Space') { sendArm(true); return; }
            if (e.code === 'Escape') { sendArm(false); return; }
            if (e.code === 'KeyL') { sendLand(true); return; }
            if (keyMap[e.code]) {
                activeKeys.add(keyMap[e.code]);
                sendControlState();
            }
        });

        document.addEventListener('keyup', (e) => {
            if (keyMap[e.code]) {
                activeKeys.delete(keyMap[e.code]);
                sendControlState();
            }
        });

        function sendControlState() {
            if (ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({ type: 'keys', keys: Array.from(activeKeys) }));
            }
        }

        function sendArm(state) {
            if (ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({ type: 'arm', arm: state }));
                toast(state ? 'Arm command sent' : 'EMERGENCY DISARM sent');
            }
        }

        function sendLand(state) {
            if (ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({ type: 'land', land: state }));
                toast(state ? 'Land command sent' : 'Land cancelled');
            }
        }

        function applyRotation(degrees) {
            if (ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({ type: 'rotation', degrees: parseInt(degrees, 10) }));
            }
        }

        function applyCameraSettings() {
            const [w, h] = document.getElementById('sel-resolution').value.split('x').map(Number);
            const fps = parseInt(document.getElementById('in-fps').value, 10);
            const quality = parseInt(document.getElementById('in-quality').value, 10);
            const bitrateRaw = document.getElementById('in-bitrate').value;
            const bitrate = bitrateRaw ? parseInt(bitrateRaw, 10) * 1000 : 0;
            if (ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({
                    type: 'camera_settings', width: w, height: h, fps: fps, quality: quality, bitrate: bitrate
                }));
            }
        }

        let altitudeThrottle = null;
        function onAltitudeInput(value) {
            const label = parseFloat(value).toFixed(1) + '<span>m</span>';
            altitudeReadoutIds.forEach(id => { document.getElementById(id).innerHTML = label; });
            // Keep both sliders (Home + Flight) at the same position while dragging either one.
            altitudeSliderIds.forEach(id => { document.getElementById(id).value = value; });
            if (altitudeThrottle) clearTimeout(altitudeThrottle);
            altitudeThrottle = setTimeout(() => {
                if (ws.readyState === WebSocket.OPEN) {
                    ws.send(JSON.stringify({ type: 'altitude_setpoint', altitude: parseFloat(value) }));
                }
            }, 80);
        }

        function applyTakeoffHeight(sourceId) {
            const id = sourceId || 'in-takeoff-height';
            const h = parseFloat(document.getElementById(id).value);
            takeoffHeightIds.forEach(otherId => {
                if (otherId !== id) document.getElementById(otherId).value = h;
            });
            if (ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({ type: 'takeoff_height', height: h }));
            }
        }

        function setTakeoffHeightChip(h, targetId) {
            document.getElementById(targetId).value = h;
            applyTakeoffHeight(targetId);
        }

        let speedThrottle = null;
        function onSpeedInput(kind, value) {
            const v = parseFloat(value);
            if (kind === 'xy') document.getElementById('xy-speed-readout').innerText = v.toFixed(1);
            if (kind === 'yaw') document.getElementById('yaw-speed-readout').innerText = v.toFixed(1);
            if (speedThrottle) clearTimeout(speedThrottle);
            speedThrottle = setTimeout(() => {
                if (ws.readyState !== WebSocket.OPEN) return;
                const payload = { type: 'speed_settings' };
                if (kind === 'xy') payload.xy_speed = v;
                if (kind === 'yaw') payload.yaw_speed = v;
                ws.send(JSON.stringify(payload));
            }, 150);
        }

        // ---------- Boost toggle (touch-friendly alternative to holding Shift) ----------
        let boostOn = false;
        function toggleBoost() {
            boostOn = !boostOn;
            const btn = document.getElementById('btn-boost');
            btn.classList.toggle('active', boostOn);
            btn.textContent = boostOn ? 'Boost: On' : 'Boost: Off';
            if (ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({ type: 'boost', active: boostOn }));
            }
        }

        // ---------- On-screen joystick ----------
        function makeJoystick(baseEl, knobEl, axisLock, onMove, onStart, onEnd) {
            const radius = () => baseEl.clientWidth / 2 - knobEl.clientWidth / 2;
            let active = false;
            let pointerId = null;

            function setKnob(dx, dy) {
                knobEl.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;
            }
            function reset() { setKnob(0, 0); }
            reset();

            function handleMove(clientX, clientY) {
                const rect = baseEl.getBoundingClientRect();
                const cx = rect.left + rect.width / 2;
                const cy = rect.top + rect.height / 2;
                let dx = clientX - cx;
                let dy = clientY - cy;
                if (axisLock === 'x') dy = 0;
                if (axisLock === 'y') dx = 0;
                const r = radius();
                const dist = Math.hypot(dx, dy);
                if (dist > r) {
                    dx = (dx / dist) * r;
                    dy = (dy / dist) * r;
                }
                setKnob(dx, dy);
                onMove(dx / r, -(dy / r)); // invert y: up = positive
            }

            baseEl.addEventListener('pointerdown', (e) => {
                active = true;
                pointerId = e.pointerId;
                baseEl.classList.add('dragging');
                baseEl.setPointerCapture(pointerId);
                handleMove(e.clientX, e.clientY);
                onStart();
                e.preventDefault();
            });
            baseEl.addEventListener('pointermove', (e) => {
                if (!active || e.pointerId !== pointerId) return;
                handleMove(e.clientX, e.clientY);
                e.preventDefault();
            });
            function end(e) {
                if (!active || (pointerId !== null && e.pointerId !== pointerId)) return;
                active = false;
                pointerId = null;
                baseEl.classList.remove('dragging');
                reset();
                onEnd();
            }
            baseEl.addEventListener('pointerup', end);
            baseEl.addEventListener('pointercancel', end);
            baseEl.addEventListener('pointerleave', (e) => { if (active) end(e); });
        }

        const stickVec = { x: 0, y: 0, yaw: 0 };
        let moveStickOn = false, yawStickOn = false;
        let lastStickSend = 0;

        function sendStick(force) {
            const now = performance.now();
            if (!force && now - lastStickSend < 40) return;
            lastStickSend = now;
            if (ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({ type: 'stick', x: stickVec.x, y: stickVec.y, yaw: stickVec.yaw }));
            }
        }

        function maybeReleaseStick() {
            if (!moveStickOn && !yawStickOn) {
                stickVec.x = 0; stickVec.y = 0; stickVec.yaw = 0;
                if (ws.readyState === WebSocket.OPEN) {
                    ws.send(JSON.stringify({ type: 'stick_release' }));
                }
            }
        }

        // Heartbeat so a stick held perfectly still doesn't time out server-side.
        setInterval(() => {
            if (moveStickOn || yawStickOn) sendStick(true);
        }, 150);

        makeJoystick(
            document.getElementById('stick-move-base'),
            document.getElementById('stick-move-knob'),
            null,
            (x, y) => { stickVec.x = x; stickVec.y = y; sendStick(false); },
            () => { moveStickOn = true; },
            () => { moveStickOn = false; stickVec.x = 0; stickVec.y = 0; maybeReleaseStick(); }
        );

        makeJoystick(
            document.getElementById('stick-yaw-base'),
            document.getElementById('stick-yaw-knob'),
            'x',
            (x, y) => { stickVec.yaw = x; sendStick(false); },
            () => { yawStickOn = true; },
            () => { yawStickOn = false; stickVec.yaw = 0; maybeReleaseStick(); }
        );

        // ---------- D-pad / yaw buttons (press-and-hold, reuses keyboard's key set) ----------
        function bindHoldButton(id, key) {
            const el = document.getElementById(id);
            function press(e) {
                el.classList.add('pressed');
                activeKeys.add(key);
                sendControlState();
                e.preventDefault();
            }
            function release(e) {
                el.classList.remove('pressed');
                activeKeys.delete(key);
                sendControlState();
            }
            el.addEventListener('pointerdown', press);
            el.addEventListener('pointerup', release);
            el.addEventListener('pointercancel', release);
            el.addEventListener('pointerleave', release);
        }
        bindHoldButton('btn-up', 'up');
        bindHoldButton('btn-down', 'down');
        bindHoldButton('btn-left', 'left');
        bindHoldButton('btn-right', 'right');
        bindHoldButton('btn-yaw-left', 'yaw_left');
        bindHoldButton('btn-yaw-right', 'yaw_right');
    </script>
</body>
</html>
"""

@app.get("/", response_class=HTMLResponse)
def index():
    return HTML_TEMPLATE

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()

    # Push current server-side state so the UI reflects reality on connect
    # (or reconnect) instead of showing stale defaults.
    await websocket.send_text(json.dumps({'type': 'camera_settings', **update_camera_settings()}))
    await websocket.send_text(json.dumps({'type': 'flight_settings', **ros_node.get_flight_settings()}))
    await websocket.send_text(json.dumps({'type': 'speed_settings', **ros_node.get_speed_settings()}))

    try:
        while True:
            # Broadcast telemetry updates to client at ~10 Hz
            st, age = ros_node.get_status()
            await websocket.send_text(json.dumps({'type': 'telemetry', **st}))

            # Non-blocking check for control commands from client
            try:
                msg_raw = await asyncio.wait_for(websocket.receive_text(), timeout=0.1)
                data = json.loads(msg_raw)
                msg_type = data.get('type')

                if msg_type == 'keys':
                    ros_node.set_keys(data.get('keys', []))
                elif msg_type == 'stick':
                    ros_node.set_axes(data.get('x', 0), data.get('y', 0), data.get('yaw', 0))
                elif msg_type == 'stick_release':
                    ros_node.release_axes()
                elif msg_type == 'boost':
                    ros_node.set_boost(bool(data.get('active', False)))
                elif msg_type == 'speed_settings':
                    settings = ros_node.set_speed_settings(
                        xy=data.get('xy_speed'), yaw=data.get('yaw_speed'),
                    )
                    await websocket.send_text(json.dumps({'type': 'speed_settings_ack', **settings}))
                elif msg_type == 'arm':
                    ros_node.send_arm(data.get('arm', False))
                elif msg_type == 'land':
                    ros_node.send_land(bool(data.get('land', True)))
                elif msg_type == 'rotation':
                    update_camera_settings(rotation=data.get('degrees', 0))
                elif msg_type == 'camera_settings':
                    settings = update_camera_settings(
                        width=data.get('width'), height=data.get('height'),
                        fps=data.get('fps'), quality=data.get('quality'),
                        bitrate=data.get('bitrate'),
                    )
                    await websocket.send_text(json.dumps({'type': 'camera_settings_ack', **settings}))
                elif msg_type == 'takeoff_height':
                    h = ros_node.set_takeoff_height(data.get('height', TAKEOFF_HEIGHT_DEFAULT))
                    await websocket.send_text(json.dumps({'type': 'takeoff_height_ack', 'height': h}))
                elif msg_type == 'altitude_setpoint':
                    ros_node.set_altitude_setpoint(data.get('altitude', TAKEOFF_HEIGHT_DEFAULT))
            except asyncio.TimeoutError:
                pass
    except WebSocketDisconnect:
        ros_node.set_keys([])
        ros_node.release_axes()


# --- Main Thread Execution ---
def spin_ros():
    rclpy.spin(ros_node)

if __name__ == '__main__':
    rclpy.init()
    ros_node = WebTeleopNode()

    # Spin ROS 2 in background thread
    t = threading.Thread(target=spin_ros, daemon=True)
    t.start()

    # Start the camera capture thread (owns its own rpicam-vid subprocess)
    camera_stream.start()

    # Serve Web UI on all network interfaces on port 8000
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="warning")
