#!/usr/bin/env python3
"""
drone_gcs.py -- Ground Control Launcher for the indoor autonomous drone.

A single web page that starts, stops and monitors every piece of the stack,
so the only thing you ever type in a terminal is:

    python3 ~/drone_gcs.py            # then open http://<pi-ip>:8080

Design notes (why it is built this way):

  * This process deliberately does NOT import rclpy. It is a supervisor, not
    a ROS node. That keeps it off the ROS graph entirely -- it costs no DDS
    discovery traffic, it starts in under a second, and it survives every
    ROS node on the machine being killed and restarted underneath it.

  * Every managed process is spawned in its own session (setsid) so that
    stopping it kills the whole tree. `ros2 launch` spawns children; without
    the process group you leak orphaned nodes that keep publishing and quietly
    fight the ones you just restarted.

  * Shutdown is SIGINT first (ROS nodes need it to deregister cleanly), then
    SIGTERM, then SIGKILL.

  * The camera pipeline only runs while somebody is actually watching, and it
    uses rpicam-vid's *native* rotation instead of decoding and re-encoding
    every frame in Python. See the CameraStream docstring for why that was
    the main source of lag on the Pi.
"""

import json
import os
import shlex
import signal
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional

import psutil
import uvicorn
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse

# ---------------------------------------------------------------------------
# Configuration -- edit these, nothing below should need touching
# ---------------------------------------------------------------------------

HOME = "/home/abdazwd"
GUI_PORT = 8080

# Sourced (in order) into every managed process's shell.
ROS_SETUP = [
    "/opt/ros/jazzy/setup.bash",
    f"{HOME}/ros2_ws/install/setup.bash",
    f"{HOME}/px4_ros2_ws/install/setup.bash",
]

SLAM_PARAMS = f"{HOME}/px4_ros2_ws/src/odom_bridge/config/slam_toolbox_params.yaml"
NAV2_PARAMS = f"{HOME}/nav2_params.yaml"
RVIZ_CONFIG = f"{HOME}/px4_ros2_ws/run_cmds/drone3d.rviz"

# shaft_inspection runs off its own parameter file. The package also ships
# config/shaft.yaml, but that one is the SIMULATION config (use_sim_time,
# Gazebo laserscan for the floor range, auto_launch takeoff) and must never
# be pointed at the real vehicle.
SHAFT_PARAMS = f"{HOME}/px4_ros2_ws/shaft_real.yaml"
SHAFT_DASH_PORT = 8081        # the shaft dashboard's own server; embedded below

LOG_LINES_KEPT = 600          # per process ring buffer
STATUS_HZ = 1.0               # how often the browser gets a status frame


@dataclass
class Spec:
    """One managed unit: either a child process we own, or a systemd service."""
    key: str
    label: str
    group: str
    desc: str = ""
    cmd: Optional[str] = None            # for kind == "proc"
    service: Optional[str] = None        # for kind == "service"
    kind: str = "proc"                   # "proc" | "service"
    startup: float = 3.0                 # settle time before the next unit in a sequence
    needs_display: bool = False


SPECS: List[Spec] = [
    # --- system --------------------------------------------------------
    Spec("xrce", "Micro-XRCE Agent", "system", kind="service",
         service="micro-xrce-agent.service",
         desc="PX4 <-> ROS 2 uORB bridge on /dev/ttyAMA0. Everything /fmu/* depends on this."),
    Spec("lidar", "RPLIDAR C1", "system", kind="service",
         service="sllidar.service",
         desc="ros2 launch sllidar_ros2 sllidar_c1_launch.py -- publishes /scan.",
         startup=4.0),

    # --- SLAM chain (order matters) -------------------------------------
    Spec("odom_bridge", "Odom Bridge", "slam",
         cmd="ros2 run odom_bridge odom_bridge_node",
         desc="PX4 vehicle_odometry (NED/FRD) -> /odom (ENU/FLU) + static base_link->laser TF.",
         startup=3.0),
    Spec("rf2o", "RF2O Laser Odometry", "slam",
         cmd="ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py",
         desc="Scan-matching odometry. Owns the odom->base_link TF.",
         startup=8.0),
    Spec("slam_toolbox", "SLAM Toolbox", "slam",
         cmd=f"ros2 launch slam_toolbox online_async_launch.py slam_params_file:={SLAM_PARAMS}",
         desc="2D mapping. Publishes /map and the map->odom correction.",
         startup=10.0),
    Spec("vision_odom", "Vision Odom -> PX4", "slam",
         cmd="ros2 run odom_bridge vision_odom_bridge",
         desc="map->base_link TF -> /fmu/in/vehicle_visual_odometry for EKF2.",
         startup=2.0),

    # --- navigation ------------------------------------------------------
    Spec("nav2", "Nav2 Bringup", "nav",
         cmd=(f"ros2 launch nav2_bringup navigation_launch.py "
              f"params_file:={NAV2_PARAMS} use_sim_time:=false autostart:=true"),
         desc="Costmaps, planner, controller, behaviour tree.",
         startup=12.0),
    Spec("explore", "Frontier Explorer", "nav",
         cmd="ros2 run odom_bridge frontier_explorer",
         desc="Autonomous frontier-based exploration driving Nav2's NavigateToPose.",
         startup=2.0),

    # --- flight ----------------------------------------------------------
    Spec("avoidance", "Reactive Avoidance", "flight",
         cmd="ros2 run odom_bridge reactive_avoidance",
         desc="Filters /cmd_vel + /offboard_velocity_cmd against /scan. Brakes and steers.",
         startup=3.0),
    Spec("obstacle_dist", "PX4 Obstacle Distance", "flight",
         cmd=f"python3 {HOME}/laserscan_to_obstacle_distance.py",
         desc="/scan -> PX4. NOTE: PX4 only acts on this in Position mode, not Offboard.",
         startup=2.0),
    # The remaps are what put Reactive Avoidance in the command path. Without
    # them this node reads the raw topics and the filter is bypassed entirely.
    Spec("offboard", "PX4 Offboard Control", "flight",
         cmd=(f"python3 {HOME}/cmd_vel_teleop_v2.py --ros-args "
              f"-r /cmd_vel:=/cmd_vel_safe "
              f"-r /offboard_velocity_cmd:=/offboard_velocity_cmd_safe"),
         desc="Arm / takeoff / hover / land. Reads the AVOIDANCE-FILTERED topics.",
         startup=3.0),
    Spec("web_teleop", "Web Teleop (:8000)", "flight",
         cmd=f"python3 {HOME}/web_teleop.py",
         desc="Your existing manual flight UI. Takes over the camera while it runs.",
         startup=3.0),

    # --- 3D mapping / viz --------------------------------------------------
    Spec("pose_3d", "3D Pose Publisher", "mapping3d",
         cmd="ros2 run odom_bridge pose_3d_node",
         desc="Fuses SLAM x/y/yaw with PX4 altitude+attitude -> map->base_link_3d TF.",
         startup=2.0),
    Spec("scan_3d", "3D Scan Mapper", "mapping3d",
         cmd="ros2 run odom_bridge scan_3d_mapper",
         desc="Accumulates /scan at varying altitude into a voxel cloud on /map_3d.",
         startup=2.0),
    # --- shaft inspection (order matters) ----------------------------------
    # These four replace the SLAM/Nav stack for a shaft run; they do not
    # complement it. See CONFLICTS below for the two that actively fight.
    Spec("shaft_perception", "Shaft Perception", "shaft",
         cmd=f"ros2 run shaft_inspection shaft_perception --ros-args --params-file {SHAFT_PARAMS}",
         desc="/scan -> bore centre, clearance, radius. Feeds EKF2 as external vision.",
         startup=5.0),
    Spec("shaft_mapper", "Shaft Mapper", "shaft",
         cmd=f"ros2 run shaft_inspection shaft_mapper --ros-args --params-file {SHAFT_PARAMS}",
         desc="Accumulates /scan by depth into a 3D cloud + radius profile in ~/shaft_maps.",
         startup=2.0),
    Spec("shaft_dashboard", f"Shaft Dashboard (:{SHAFT_DASH_PORT})", "shaft",
         cmd=f"ros2 run shaft_inspection shaft_dashboard --ros-args --params-file {SHAFT_PARAMS}",
         desc="Read-only mission dashboard. Embedded in the Shaft panel on this page.",
         startup=3.0),
    # Started last, and deliberately: with start_mode=pilot_handover it sits in
    # WAIT and commands nothing until PX4 reports Offboard, so the pilot owns
    # the aircraft right up to the moment they flip the switch.
    Spec("shaft_mission", "Shaft Mission", "shaft",
         cmd=f"ros2 run shaft_inspection shaft_mission --ros-args --params-file {SHAFT_PARAMS}",
         desc="Centre -> descend -> turn around -> climb out. Waits for Offboard (pilot handover).",
         startup=3.0),

    # --- shaft bench tools: one-shot, run them on the ground, read the log ---
    Spec("shaft_preflight", "Shaft Preflight Check", "shaft",
         cmd="ros2 run shaft_inspection shaft_preflight",
         desc="One-shot GO/NO-GO: lidar rate, self-hits, PX4 link, EKF2 height, ToF. Read its log.",
         startup=1.0),
    Spec("shaft_mount_check", "Lidar Mount Check", "shaft",
         cmd="ros2 run shaft_inspection shaft_mount_check",
         desc="One-shot: measures lidar_yaw_offset_deg / lidar_upside_down for shaft_real.yaml.",
         startup=1.0),

    Spec("rviz", "RViz2", "mapping3d",
         cmd=f"rviz2 -d {RVIZ_CONFIG}",
         desc="Heavy. Run it on a laptop over the network if you can.",
         startup=2.0, needs_display=True),
]

SPEC_BY_KEY: Dict[str, Spec] = {s.key: s for s in SPECS}

# Ordered start-up sequences. Each entry is started, then we wait its
# `startup` seconds before moving to the next -- same idea as the sleeps in
# run_cmds/slam.sh, but visible and cancellable from the UI.
SEQUENCES: Dict[str, List[str]] = {
    "slam": ["odom_bridge", "rf2o", "slam_toolbox", "vision_odom"],
    "nav": ["nav2"],
    "explore": ["explore"],
    "flight": ["avoidance", "obstacle_dist", "offboard", "web_teleop"],
    "mapping3d": ["pose_3d", "scan_3d"],
    # Perception first and with the longest settle: the mission node's first
    # reading should already be a valid bore fix, not a "no fix yet" warning.
    "shaft": ["shaft_perception", "shaft_mapper", "shaft_dashboard", "shaft_mission"],
}

# Units that cannot coexist, because they publish to the SAME PX4 input topic.
# Two publishers on one /fmu/in/ topic do not merge -- PX4 acts on whichever
# sample arrived last, so the vehicle alternates between two controllers at
# 20 Hz. Starting either side of a pair stops the other first, and says so in
# the log rather than failing silently.
#
#   /fmu/in/vehicle_visual_odometry   shaft_perception  vs  vision_odom
#   /fmu/in/trajectory_setpoint       shaft_mission     vs  offboard
CONFLICTS: Dict[str, List[str]] = {
    "shaft_perception": ["vision_odom"],
    "vision_odom":      ["shaft_perception"],
    "shaft_mission":    ["offboard"],
    "offboard":         ["shaft_mission"],
}

GROUP_LABELS = {
    "system": "System",
    "slam": "SLAM",
    "nav": "Navigation",
    "flight": "Flight",
    "mapping3d": "3D Mapping",
    "shaft": "Shaft Inspection",
}


# ---------------------------------------------------------------------------
# Log buffer
# ---------------------------------------------------------------------------

class LogBuffer:
    """Ring buffer with a monotonically increasing sequence number.

    The browser asks for "everything after seq N" so a client that has been
    watching for an hour still only pulls the handful of new lines, and a
    client that just switched tabs pulls the whole buffer once.
    """

    def __init__(self, maxlen: int = LOG_LINES_KEPT):
        self._lines = deque(maxlen=maxlen)
        self._seq = 0
        self._lock = threading.Lock()

    def append(self, line: str):
        line = line.rstrip("\n")
        with self._lock:
            self._seq += 1
            self._lines.append((self._seq, line))

    def since(self, seq: int, limit: int = 400):
        with self._lock:
            out = [(s, t) for s, t in self._lines if s > seq]
            head_seq = self._lines[0][0] if self._lines else self._seq
        if seq and seq < head_seq - 1:
            out.insert(0, (0, "--- (older lines dropped) ---"))
        return out[-limit:], self._seq

    def clear(self):
        with self._lock:
            self._lines.clear()


# ---------------------------------------------------------------------------
# Managed unit
# ---------------------------------------------------------------------------

class Unit:
    def __init__(self, spec: Spec):
        self.spec = spec
        self.log = LogBuffer()
        self.proc: Optional[subprocess.Popen] = None
        self.started_at: Optional[float] = None
        self.last_exit: Optional[int] = None
        self.stopping = False
        self._lock = threading.Lock()
        self._psutil_cache: Dict[int, psutil.Process] = {}
        self._journal: Optional[subprocess.Popen] = None

    # -- shell helpers ---------------------------------------------------
    @staticmethod
    def _wrap(cmd: str, needs_display: bool = False) -> List[str]:
        srcs = " ".join(f"source {p} >/dev/null 2>&1;" for p in ROS_SETUP)
        prefix = "export DISPLAY=${DISPLAY:-:0}; " if needs_display else ""
        # `exec` matters: without it bash stays as the process-group leader's
        # parent and signals land on the wrong PID.
        return ["/bin/bash", "-c", f"{srcs} {prefix}exec {cmd}"]

    # -- state -----------------------------------------------------------
    def is_running(self) -> bool:
        if self.spec.kind == "service":
            return self._service_active()
        with self._lock:
            return self.proc is not None and self.proc.poll() is None

    def _service_active(self) -> bool:
        try:
            r = subprocess.run(["systemctl", "is-active", self.spec.service],
                               capture_output=True, text=True, timeout=4)
            return r.stdout.strip() == "active"
        except Exception:
            return False

    def _service_pid(self) -> Optional[int]:
        try:
            r = subprocess.run(["systemctl", "show", "-p", "MainPID", "--value",
                                self.spec.service],
                               capture_output=True, text=True, timeout=4)
            pid = int(r.stdout.strip() or 0)
            return pid or None
        except Exception:
            return None

    def pid(self) -> Optional[int]:
        if self.spec.kind == "service":
            return self._service_pid()
        with self._lock:
            if self.proc and self.proc.poll() is None:
                return self.proc.pid
        return None

    # -- resource usage --------------------------------------------------
    def usage(self):
        """CPU% and RSS for the whole process tree.

        psutil's cpu_percent() is relative to the previous call *on the same
        object*, which is why the Process objects are cached per PID rather
        than recreated each poll -- recreating them would make every reading
        come back 0.0.
        """
        pid = self.pid()
        if not pid:
            self._psutil_cache.clear()
            return None
        try:
            root = psutil.Process(pid)
            procs = [root] + root.children(recursive=True)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            return None

        cpu = 0.0
        rss = 0
        live = {}
        for p in procs:
            try:
                cached = self._psutil_cache.get(p.pid)
                if cached is None or not cached.is_running():
                    cached = p
                    cached.cpu_percent(None)   # prime; first read is always 0
                live[p.pid] = cached
                cpu += cached.cpu_percent(None)
                rss += cached.memory_info().rss
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        self._psutil_cache = live
        return {"pid": pid, "cpu": round(cpu, 1), "mem_mb": round(rss / 1e6, 1),
                "nproc": len(procs)}

    # -- start / stop ----------------------------------------------------
    def start(self) -> str:
        if self.is_running():
            return "already running"

        # Evict anything that would publish to the same PX4 input topic. Doing
        # this here rather than in the UI means it also covers sequence starts
        # and a direct POST to /api/unit/<key>/start.
        evicted = []
        for other in CONFLICTS.get(self.spec.key, []):
            peer = UNITS.get(other)
            if peer is not None and peer.is_running():
                peer.log.append(f"[gcs] stopped: conflicts with {self.spec.label}")
                peer.stop()
                evicted.append(peer.spec.label)
        if evicted:
            self.log.append(f"[gcs] stopped first (topic conflict): {', '.join(evicted)}")

        if self.spec.kind == "service":
            self._run_systemctl("start")
            self._start_journal()
            return "service start requested"

        with self._lock:
            self.stopping = False
            self.last_exit = None
            try:
                self.proc = subprocess.Popen(
                    self._wrap(self.spec.cmd, self.spec.needs_display),
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    bufsize=1, text=True, errors="replace",
                    cwd=HOME, start_new_session=True,
                )
            except Exception as e:
                self.log.append(f"[gcs] FAILED to start: {e!r}")
                return f"failed: {e}"
            self.started_at = time.time()
            proc = self.proc

        self.log.append(f"[gcs] started pid={proc.pid}: {self.spec.cmd}")
        threading.Thread(target=self._pump, args=(proc,), daemon=True).start()
        return "started"

    def _pump(self, proc: subprocess.Popen):
        try:
            for line in proc.stdout:
                self.log.append(line)
        except Exception as e:
            self.log.append(f"[gcs] log reader ended: {e!r}")
        finally:
            code = proc.wait()
            with self._lock:
                self.last_exit = code
            verb = "stopped" if self.stopping else "EXITED"
            self.log.append(f"[gcs] {verb} (exit code {code})")

    def stop(self, timeout: float = 6.0) -> str:
        if self.spec.kind == "service":
            self._stop_journal()
            self._run_systemctl("stop")
            return "service stop requested"

        with self._lock:
            proc = self.proc
            self.stopping = True
        if proc is None or proc.poll() is not None:
            return "not running"

        try:
            pgid = os.getpgid(proc.pid)
        except ProcessLookupError:
            return "gone"

        # SIGINT lets rclpy nodes shut down cleanly and deregister from DDS.
        for sig, wait in ((signal.SIGINT, timeout), (signal.SIGTERM, 3.0)):
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                return "stopped"
            deadline = time.time() + wait
            while time.time() < deadline:
                if proc.poll() is not None:
                    return "stopped"
                time.sleep(0.15)
            self.log.append(f"[gcs] still alive after {sig.name}, escalating")

        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        return "killed"

    def restart(self) -> str:
        if self.spec.kind == "service":
            self._stop_journal()
            self._run_systemctl("restart")
            self._start_journal()
            return "service restart requested"
        self.stop()
        time.sleep(1.0)
        return self.start()

    # -- systemd plumbing ------------------------------------------------
    def _run_systemctl(self, action: str):
        cmd = ["sudo", "-n", "systemctl", action, self.spec.service]
        self.log.append(f"[gcs] {' '.join(cmd)}")
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=25)
            for stream in (r.stdout, r.stderr):
                for line in (stream or "").splitlines():
                    self.log.append(line)
            if r.returncode != 0:
                self.log.append(
                    f"[gcs] systemctl exited {r.returncode}. If this is a password "
                    f"prompt, add a NOPASSWD sudoers rule (see README_GCS.md).")
        except Exception as e:
            self.log.append(f"[gcs] systemctl failed: {e!r}")

    def _start_journal(self):
        """Follow the unit's journal so service logs show up in the same pane."""
        self._stop_journal()
        try:
            self._journal = subprocess.Popen(
                ["journalctl", "-u", self.spec.service, "-f", "-n", "40",
                 "--no-pager", "-o", "cat"],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                bufsize=1, text=True, errors="replace", start_new_session=True)
        except Exception as e:
            self.log.append(f"[gcs] journalctl unavailable: {e!r}")
            return
        threading.Thread(target=self._pump_journal,
                         args=(self._journal,), daemon=True).start()

    def _pump_journal(self, proc):
        try:
            for line in proc.stdout:
                self.log.append(line)
        except Exception:
            pass

    def _stop_journal(self):
        if self._journal and self._journal.poll() is None:
            try:
                os.killpg(os.getpgid(self._journal.pid), signal.SIGTERM)
            except Exception:
                pass
        self._journal = None

    def status(self):
        running = self.is_running()
        st = {
            "key": self.spec.key,
            "label": self.spec.label,
            "group": self.spec.group,
            "desc": self.spec.desc,
            "kind": self.spec.kind,
            "cmd": self.spec.cmd or f"systemctl … {self.spec.service}",
            "running": running,
            "uptime": (round(time.time() - self.started_at)
                       if running and self.started_at else None),
            "last_exit": self.last_exit,
            "usage": self.usage() if running else None,
        }
        return st


UNITS: Dict[str, Unit] = {s.key: Unit(s) for s in SPECS}


# ---------------------------------------------------------------------------
# Sequence runner
# ---------------------------------------------------------------------------

class SequenceRunner:
    """Starts a list of units in order with settle delays, in the background,
    so the HTTP request returns immediately and the UI can show progress."""

    def __init__(self):
        self.active: Optional[str] = None
        self.step = ""
        self.remaining = 0.0
        self._cancel = threading.Event()
        self._lock = threading.Lock()

    def start(self, name: str) -> str:
        keys = SEQUENCES.get(name)
        if not keys:
            return "unknown sequence"
        with self._lock:
            if self.active:
                return f"sequence '{self.active}' already running"
            self.active = name
            self._cancel.clear()
        threading.Thread(target=self._run, args=(name, keys), daemon=True).start()
        return "sequence started"

    def _run(self, name: str, keys: List[str]):
        try:
            for i, key in enumerate(keys):
                if self._cancel.is_set():
                    break
                unit = UNITS[key]
                self.step = f"{i+1}/{len(keys)} {unit.spec.label}"
                unit.start()
                if i == len(keys) - 1:
                    break
                delay = unit.spec.startup
                end = time.time() + delay
                while time.time() < end:
                    if self._cancel.is_set():
                        break
                    self.remaining = round(end - time.time(), 1)
                    time.sleep(0.2)
        finally:
            with self._lock:
                self.active = None
                self.step = ""
                self.remaining = 0.0

    def cancel(self):
        self._cancel.set()
        return "cancel requested"

    def stop_sequence(self, name: str) -> str:
        keys = SEQUENCES.get(name)
        if not keys:
            return "unknown sequence"
        self._cancel.set()
        # Reverse order: tear down consumers before the things they subscribe to.
        for key in reversed(keys):
            UNITS[key].stop()
        return "sequence stopped"

    def status(self):
        return {"active": self.active, "step": self.step, "remaining": self.remaining}


sequences = SequenceRunner()


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------

CAM_LOCK = threading.Lock()
CAM = {
    "width": 640, "height": 480,
    "fps": 15,
    "quality": 50,          # rpicam-vid's own MJPEG quality knob
    "rotation": 180,        # 0/180 are free (done in the ISP); 90/270 cost CPU
    "hflip": 0, "vflip": 0,
    "denoise": "cdn_off",   # cdn_off is markedly cheaper than the default
    "flush": True,          # push each frame out immediately instead of buffering
    "idle_timeout": 8.0,    # stop the camera this long after the last viewer
}

RESOLUTIONS = [(320, 240), (640, 480), (800, 600), (1280, 720)]
CAM_LOG = LogBuffer(120)


class CameraStream:
    """rpicam-vid -> MJPEG fan-out.

    Three things make this materially faster than decode-rotate-reencode:

      1. Rotation is handed to rpicam-vid (`--rotation 0|180`), so frames are
         never decoded in Python. 90/270 are not supported by the ISP; if you
         pick one we fall back to a Pillow rotate and say so in the log,
         because that is what actually costs the frame rate.
      2. The capture process only exists while a browser is pulling frames,
         and exits `idle_timeout` seconds after the last one goes away.
      3. Viewers always get the newest frame -- a slow client skips frames
         instead of building a backlog, so latency never accumulates.
    """

    def __init__(self):
        self._cond = threading.Condition()
        self._frame = None
        self._frame_id = 0
        self._viewers = 0
        self._last_viewer = 0.0
        self._restart = threading.Event()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._fps_marks = deque(maxlen=30)
        self._blocked_reason = ""
        # Tracked so shutdown() can kill the capture process directly. The
        # thread is a daemon blocked in a read(), and rpicam-vid runs in its
        # own session -- so at interpreter exit the thread is killed without
        # ever reaching its finally block, and the camera process survives,
        # holding the sensor and locking web_teleop out of it.
        self._proc = None
        self._proc_lock = threading.Lock()

    # -- lifecycle -------------------------------------------------------
    def ensure_thread(self):
        if self._thread is None or not self._thread.is_alive():
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def acquire(self):
        with self._cond:
            self._viewers += 1
        self.ensure_thread()

    def release(self):
        with self._cond:
            self._viewers = max(0, self._viewers - 1)
            self._last_viewer = time.time()

    def request_restart(self):
        self._restart.set()

    def shutdown(self):
        self._stop.set()
        self._restart.set()
        self._kill_proc()

    def _kill_proc(self):
        with self._proc_lock:
            proc, self._proc = self._proc, None
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=2.0)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass

    def get_frame(self, last_id: int, timeout: float = 3.0):
        with self._cond:
            if self._frame_id == last_id:
                self._cond.wait(timeout=timeout)
            return self._frame, self._frame_id

    def stats(self):
        with self._cond:
            viewers = self._viewers
            fid = self._frame_id
            marks = list(self._fps_marks)
        fps = 0.0
        if len(marks) > 1:
            span = marks[-1] - marks[0]
            if span > 0:
                fps = round((len(marks) - 1) / span, 1)
        return {"viewers": viewers, "frames": fid, "fps": fps,
                "live": bool(self._thread and self._thread.is_alive() and viewers),
                "blocked": self._blocked_reason}

    # -- capture ---------------------------------------------------------
    @staticmethod
    def _build_cmd(s):
        native_rot = 180 if s["rotation"] == 180 else 0
        cmd = [
            "rpicam-vid",
            "--width", str(s["width"]), "--height", str(s["height"]),
            "--framerate", str(s["fps"]),
            "--codec", "mjpeg", "--quality", str(s["quality"]),
            "--rotation", str(native_rot),
            "--denoise", s["denoise"],
            "--nopreview", "-t", "0", "-o", "-",
        ]
        if s["hflip"]:
            cmd += ["--hflip"]
        if s["vflip"]:
            cmd += ["--vflip"]
        if s["flush"]:
            cmd += ["--flush"]
        return cmd

    def _camera_busy_by_teleop(self) -> bool:
        # web_teleop.py owns rpicam-vid itself; two readers of the same sensor
        # just produce two black streams, so we stand down while it is up.
        return UNITS["web_teleop"].is_running()

    def _run(self):
        backoff = 0
        while not self._stop.is_set():
            with self._cond:
                idle = self._viewers == 0
            if idle:
                if time.time() - self._last_viewer > CAM["idle_timeout"]:
                    self._blocked_reason = ""
                    return                       # no viewers: release the sensor
                time.sleep(0.25)
                continue

            if self._camera_busy_by_teleop():
                self._blocked_reason = "web_teleop is using the camera"
                time.sleep(1.0)
                continue
            self._blocked_reason = ""

            with CAM_LOCK:
                s = dict(CAM)

            try:
                proc = subprocess.Popen(self._build_cmd(s), stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE, bufsize=0,
                                        start_new_session=True)
            except FileNotFoundError:
                self._blocked_reason = "rpicam-vid not found"
                CAM_LOG.append("[cam] rpicam-vid not installed")
                time.sleep(3.0)
                continue
            except Exception as e:
                CAM_LOG.append(f"[cam] launch failed: {e!r}")
                time.sleep(2.0)
                continue

            with self._proc_lock:
                self._proc = proc
            CAM_LOG.append(f"[cam] {s['width']}x{s['height']}@{s['fps']} q{s['quality']} rot{s['rotation']}")
            self._restart.clear()
            got_frame = False
            buf = b""
            sw_rot = s["rotation"] if s["rotation"] in (90, 270) else 0
            if sw_rot:
                CAM_LOG.append("[cam] 90/270 rotation is done in software and costs "
                               "real CPU on the Pi -- prefer 0 or 180.")
            try:
                while not self._stop.is_set() and not self._restart.is_set():
                    with self._cond:
                        if self._viewers == 0 and \
                           time.time() - self._last_viewer > CAM["idle_timeout"]:
                            break
                    chunk = proc.stdout.read(65536)
                    if not chunk:
                        break
                    buf += chunk
                    # Drain every complete JPEG in the buffer, keep only the last.
                    while True:
                        start = buf.find(b"\xff\xd8")
                        if start < 0:
                            break
                        end = buf.find(b"\xff\xd9", start + 2)
                        if end < 0:
                            if start > 0:
                                buf = buf[start:]
                            break
                        jpg = buf[start:end + 2]
                        buf = buf[end + 2:]
                        got_frame = True
                        if sw_rot:
                            jpg = self._sw_rotate(jpg, sw_rot, s["quality"])
                        with self._cond:
                            self._frame = jpg
                            self._frame_id += 1
                            self._fps_marks.append(time.time())
                            self._cond.notify_all()
            finally:
                self._kill_proc()

            if not got_frame and not self._stop.is_set():
                backoff += 1
                err = b""
                try:
                    err = proc.stderr.read() or b""
                except Exception:
                    pass
                msg = err.decode(errors="replace").strip()
                CAM_LOG.append(f"[cam] no frames. {msg or 'Is another process holding the camera?'}")
                time.sleep(min(backoff, 5))
            else:
                backoff = 0

    @staticmethod
    def _sw_rotate(jpg: bytes, deg: int, quality: int) -> bytes:
        try:
            import io
            from PIL import Image
            img = Image.open(io.BytesIO(jpg)).rotate(-deg, expand=True)
            out = io.BytesIO()
            img.save(out, format="JPEG", quality=quality)
            return out.getvalue()
        except Exception:
            return jpg


camera = CameraStream()


def update_camera(**kw):
    restart = False
    with CAM_LOCK:
        for k in ("width", "height", "fps", "quality", "rotation", "hflip", "vflip"):
            if kw.get(k) is not None:
                v = int(kw[k])
                if k == "fps":
                    v = max(2, min(30, v))
                if k == "quality":
                    v = max(10, min(95, v))
                if k == "rotation" and v not in (0, 90, 180, 270):
                    continue
                if CAM[k] != v:
                    CAM[k] = v
                    restart = True
        if kw.get("denoise") in ("auto", "off", "cdn_off", "cdn_fast", "cdn_hq"):
            if CAM["denoise"] != kw["denoise"]:
                CAM["denoise"] = kw["denoise"]
                restart = True
        if kw.get("flush") is not None:
            CAM["flush"] = bool(kw["flush"])
            restart = True
        snapshot = dict(CAM)
    if restart:
        camera.request_restart()
    return snapshot


# ---------------------------------------------------------------------------
# System stats
# ---------------------------------------------------------------------------

def system_stats():
    try:
        temp = None
        temps = psutil.sensors_temperatures() or {}
        for entries in temps.values():
            if entries:
                temp = round(entries[0].current, 1)
                break
    except Exception:
        temp = None
    vm = psutil.virtual_memory()
    try:
        load1, load5, _ = os.getloadavg()
    except OSError:
        load1 = load5 = 0.0
    return {
        "cpu": psutil.cpu_percent(None),
        "per_cpu": psutil.cpu_percent(None, percpu=True),
        "mem_pct": vm.percent,
        "mem_used_mb": round(vm.used / 1e6),
        "mem_total_mb": round(vm.total / 1e6),
        "temp": temp,
        "load1": round(load1, 2),
        "load5": round(load5, 2),
        "cores": psutil.cpu_count(),
    }


# Prime the CPU counters so the first UI frame isn't all zeros.
psutil.cpu_percent(None)
psutil.cpu_percent(None, percpu=True)


# ---------------------------------------------------------------------------
# HTTP / WebSocket API
# ---------------------------------------------------------------------------

app = FastAPI(title="Drone GCS Launcher")


@app.get("/", response_class=HTMLResponse)
def index():
    # HTML_PAGE is a raw string (the JS is full of braces and backticks, so it
    # cannot be an f-string). The one value the page needs from the config is
    # patched in here.
    return HTML_PAGE.replace("__SHAFT_PORT__", str(SHAFT_DASH_PORT))


@app.get("/api/units")
def api_units():
    return {"units": [u.status() for u in UNITS.values()],
            "groups": GROUP_LABELS,
            "sequences": SEQUENCES}


@app.post("/api/unit/{key}/{action}")
def api_unit(key: str, action: str):
    unit = UNITS.get(key)
    if not unit:
        return JSONResponse({"error": "unknown unit"}, status_code=404)
    if action not in ("start", "stop", "restart"):
        return JSONResponse({"error": "unknown action"}, status_code=400)
    result = getattr(unit, action)()
    return {"ok": True, "result": result}


@app.post("/api/sequence/{name}/{action}")
def api_sequence(name: str, action: str):
    if action == "start":
        return {"ok": True, "result": sequences.start(name)}
    if action == "stop":
        return {"ok": True, "result": sequences.stop_sequence(name)}
    if action == "cancel":
        return {"ok": True, "result": sequences.cancel()}
    return JSONResponse({"error": "unknown action"}, status_code=400)


@app.post("/api/stop_all")
def api_stop_all():
    sequences.cancel()
    order = ["shaft_mission", "shaft_mapper", "shaft_dashboard",
             "shaft_perception",
             "explore", "nav2", "web_teleop", "offboard", "obstacle_dist",
             "avoidance",
             "rviz", "scan_3d", "pose_3d", "vision_odom", "slam_toolbox",
             "rf2o", "odom_bridge"]
    for key in order:
        UNITS[key].stop()
    return {"ok": True, "result": "all managed processes stopped "
                                  "(systemd services left alone)"}


@app.get("/api/logs/{key}")
def api_logs(key: str, since: int = 0):
    if key == "camera":
        lines, seq = CAM_LOG.since(since)
    else:
        unit = UNITS.get(key)
        if not unit:
            return JSONResponse({"error": "unknown unit"}, status_code=404)
        lines, seq = unit.log.since(since)
    return {"lines": [t for _, t in lines], "seq": seq}


@app.post("/api/logs/{key}/clear")
def api_logs_clear(key: str):
    if key == "camera":
        CAM_LOG.clear()
    elif key in UNITS:
        UNITS[key].log.clear()
    return {"ok": True}


@app.get("/api/camera")
def api_camera_get():
    with CAM_LOCK:
        s = dict(CAM)
    return {"settings": s, "stats": camera.stats(), "resolutions": RESOLUTIONS}


@app.post("/api/camera")
async def api_camera_set(request: Request):
    body = await request.json()
    return {"settings": update_camera(**body), "stats": camera.stats()}


@app.get("/video_feed")
def video_feed():
    def gen():
        camera.acquire()
        last = 0
        try:
            while True:
                frame, last = camera.get_frame(last)
                if frame is None:
                    time.sleep(0.05)
                    continue
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n"
                       b"Content-Length: " + str(len(frame)).encode() +
                       b"\r\n\r\n" + frame + b"\r\n")
        finally:
            camera.release()

    return StreamingResponse(gen(),
                             media_type="multipart/x-mixed-replace; boundary=frame",
                             headers={"Cache-Control": "no-store",
                                      "X-Accel-Buffering": "no"})


@app.websocket("/ws")
async def ws(websocket: WebSocket):
    """One status frame per second plus incremental logs for whichever unit
    the browser currently has open. Deliberately small: no poses, no scans,
    no map -- this link has to stay usable over the drone's wifi."""
    import asyncio
    await websocket.accept()
    watching = None
    log_seq = 0

    async def reader():
        nonlocal watching, log_seq
        while True:
            msg = json.loads(await websocket.receive_text())
            if msg.get("type") == "watch":
                watching = msg.get("key")
                log_seq = 0

    task = None
    try:
        task = asyncio.create_task(reader())
        while True:
            payload = {
                "type": "status",
                "units": [u.status() for u in UNITS.values()],
                "sequence": sequences.status(),
                "system": system_stats(),
                "camera": camera.stats(),
                "t": time.time(),
            }
            if watching:
                if watching == "camera":
                    lines, log_seq = CAM_LOG.since(log_seq)
                elif watching in UNITS:
                    lines, log_seq = UNITS[watching].log.since(log_seq)
                else:
                    lines = []
                if lines:
                    payload["log"] = {"key": watching,
                                      "lines": [t for _, t in lines]}
            await websocket.send_text(json.dumps(payload))
            await asyncio.sleep(1.0 / STATUS_HZ)
    except (WebSocketDisconnect, RuntimeError):
        pass
    finally:
        if task:
            task.cancel()


# ---------------------------------------------------------------------------
# UI
# ---------------------------------------------------------------------------

HTML_PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Drone GCS Launcher</title>
<style>
:root{
  --bg:#0e1116; --panel:#161b22; --raised:#1c232d; --line:#2a323d;
  --text:#e6edf3; --dim:#8b949e; --accent:#2f81f7; --green:#3fb950;
  --amber:#d29922; --red:#f85149; --mono:ui-monospace,"Cascadia Code","Roboto Mono",Consolas,monospace;
}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--text);
  font:14px/1.5 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif}
header{position:sticky;top:0;z-index:20;background:#0a0d12;
  border-bottom:1px solid var(--line);padding:10px 18px;
  display:flex;align-items:center;gap:16px;flex-wrap:wrap}
h1{margin:0;font-size:14px;letter-spacing:.16em;text-transform:uppercase;white-space:nowrap}
.stat{font-family:var(--mono);font-size:11px;color:var(--dim);
  background:var(--panel);border:1px solid var(--line);border-radius:4px;padding:4px 9px;white-space:nowrap}
.stat b{color:var(--text);font-weight:600}
.stat.hot b{color:var(--red)}
.stat.warm b{color:var(--amber)}
.wrap{padding:16px;display:grid;gap:16px;grid-template-columns:minmax(0,1fr) minmax(0,420px)}
@media(max-width:1000px){.wrap{grid-template-columns:1fr}}
.card{background:var(--panel);border:1px solid var(--line);border-radius:8px;overflow:hidden}
.card>h2{margin:0;padding:10px 14px;font-size:11px;letter-spacing:.14em;
  text-transform:uppercase;color:var(--dim);border-bottom:1px solid var(--line);
  display:flex;align-items:center;gap:10px;flex-wrap:wrap}
.card>h2 .grow{flex:1}
.unit{display:flex;align-items:center;gap:10px;padding:9px 14px;border-bottom:1px solid #1d232c}
.unit:last-child{border-bottom:none}
.unit.busy{background:#161f2b}
.dot{width:9px;height:9px;border-radius:50%;background:#39414d;flex:none}
.dot.on{background:var(--green);box-shadow:0 0 7px var(--green)}
.dot.err{background:var(--red);box-shadow:0 0 7px var(--red)}
.uinfo{flex:1;min-width:0}
.uname{font-weight:600;font-size:13px;display:flex;align-items:center;gap:8px}
.udesc{color:var(--dim);font-size:11.5px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.umetric{font-family:var(--mono);font-size:11px;color:var(--dim);white-space:nowrap;text-align:right;min-width:112px}
.umetric b{color:var(--text);font-weight:600}
.umetric .hot{color:var(--red)}
button{font:inherit;font-size:12px;background:var(--raised);color:var(--text);
  border:1px solid var(--line);border-radius:5px;padding:5px 11px;cursor:pointer}
button:hover{border-color:#455060;background:#252d38}
button:active{transform:translateY(1px)}
button:disabled{opacity:.4;cursor:not-allowed}
button.go{border-color:#1f6f36;color:#7ee787}
button.stop{border-color:#6e2b28;color:#ff9c96}
button.primary{background:var(--accent);border-color:var(--accent);color:#fff;font-weight:600}
button.danger{background:#7d1f1c;border-color:#a33;color:#fff;font-weight:600}
button.tiny{padding:3px 8px;font-size:11px}
.btns{display:flex;gap:6px;flex:none}
.seqbar{display:flex;gap:8px;align-items:center;padding:9px 14px;background:#131922;
  border-bottom:1px solid var(--line);flex-wrap:wrap}
.seqbar .note{color:var(--dim);font-size:11.5px;flex:1;min-width:140px}
#log{margin:0;padding:10px 12px;height:340px;overflow:auto;background:#0a0d12;
  font-family:var(--mono);font-size:11.5px;line-height:1.45;white-space:pre-wrap;
  word-break:break-word;color:#c9d1d9}
#log .gcs{color:var(--accent)}
#log .warn{color:var(--amber)}
#log .err{color:var(--red)}
select,input[type=range]{font:inherit;font-size:12px;background:var(--raised);
  color:var(--text);border:1px solid var(--line);border-radius:5px;padding:4px 7px}
input[type=range]{padding:0;vertical-align:middle}
.grid2{display:grid;grid-template-columns:auto 1fr auto;gap:8px 10px;align-items:center;padding:12px 14px}
.grid2 label{color:var(--dim);font-size:12px}
.grid2 .val{font-family:var(--mono);font-size:11px;color:var(--text);min-width:56px;text-align:right}
#cam{width:100%;display:block;background:#000;aspect-ratio:4/3;object-fit:contain}
.camnote{padding:8px 14px;color:var(--dim);font-size:11.5px;border-top:1px solid var(--line)}
.tabs{display:flex;gap:4px;flex-wrap:wrap}
.tabs button.active{background:var(--accent);border-color:var(--accent);color:#fff}
.shaftwrap{padding:0 16px 16px}
#shaftframe{width:100%;height:720px;border:0;display:block;background:#0a0d12}
#shaftoff{padding:22px 16px;color:var(--dim);font-size:12.5px;text-align:center;line-height:1.7}
#shaftoff b{color:var(--text)}
.warnbar{padding:9px 14px;background:#2a1f10;border-bottom:1px solid #4a3612;
  color:#e8c37a;font-size:11.5px;line-height:1.6}
.toast{position:fixed;bottom:16px;left:50%;transform:translateX(-50%);
  background:#1c232d;border:1px solid var(--line);border-radius:6px;padding:9px 16px;
  font-size:12.5px;opacity:0;transition:opacity .2s;pointer-events:none;z-index:50}
.toast.show{opacity:1}
</style>
</head>
<body>
<header>
  <h1>Drone GCS</h1>
  <span class="stat" id="s-cpu">CPU <b>--</b></span>
  <span class="stat" id="s-mem">MEM <b>--</b></span>
  <span class="stat" id="s-temp">TEMP <b>--</b></span>
  <span class="stat" id="s-load">LOAD <b>--</b></span>
  <span class="stat" id="s-link">LINK <b>…</b></span>
  <span style="flex:1"></span>
  <button class="danger" onclick="stopAll()">Stop everything</button>
</header>

<div class="wrap">
  <div style="display:grid;gap:16px;align-content:start">

    <div class="card">
      <h2>Launch sequences<span class="grow"></span><span id="seqstate" style="text-transform:none;letter-spacing:0"></span></h2>
      <div class="seqbar">
        <button class="primary" onclick="seq('slam','start')">▶ SLAM stack</button>
        <button class="stop" onclick="seq('slam','stop')">■</button>
        <span class="note">odom_bridge → rf2o → slam_toolbox → vision_odom, with the settle delays from slam.sh</span>
      </div>
      <div class="seqbar">
        <button class="primary" onclick="seq('nav','start')">▶ Nav2</button>
        <button class="stop" onclick="seq('nav','stop')">■</button>
        <span class="note">needs /map and TF from the SLAM stack first</span>
      </div>
      <div class="seqbar">
        <button class="primary" onclick="seq('explore','start')">▶ Explore</button>
        <button class="stop" onclick="seq('explore','stop')">■</button>
        <span class="note">frontier exploration; needs Nav2 active</span>
      </div>
      <div class="seqbar">
        <button class="primary" onclick="seq('flight','start')">▶ Flight control</button>
        <button class="stop" onclick="seq('flight','stop')">■</button>
        <span class="note">reactive avoidance → offboard node → web teleop on :8000. The offboard
        node is launched with remaps so every velocity command passes the avoidance filter first.</span>
      </div>
      <div class="seqbar">
        <button class="primary" onclick="seq('mapping3d','start')">▶ 3D mapping</button>
        <button class="stop" onclick="seq('mapping3d','stop')">■</button>
        <span class="note">3D pose + /scan voxel accumulation for RViz</span>
      </div>
      <div class="seqbar" style="background:#1a1410;border-top:1px solid #3a2c14">
        <button class="primary" onclick="seq('shaft','start')">▶ Shaft inspection</button>
        <button class="stop" onclick="seq('shaft','stop')">■</button>
        <span class="note">perception → mapper → dashboard → mission. Starting this stops
        Vision Odom and PX4 Offboard Control: they publish to the same PX4 topics.
        The mission node sits in WAIT and commands nothing until <b>you</b> switch
        the aircraft to Offboard over the shaft.</span>
      </div>
    </div>

    <div class="card" id="units"><h2>Nodes</h2></div>
  </div>

  <div style="display:grid;gap:16px;align-content:start">
    <div class="card">
      <h2>Logs<span class="grow"></span>
        <select id="logsel" onchange="watch(this.value)"></select>
        <button class="tiny" onclick="clearLog()">clear</button>
        <label style="font-size:11px;text-transform:none;letter-spacing:0;display:flex;gap:4px;align-items:center">
          <input type="checkbox" id="autoscroll" checked> follow
        </label>
      </h2>
      <pre id="log"></pre>
    </div>

    <div class="card">
      <h2>Camera<span class="grow"></span><span id="camstat" style="text-transform:none;letter-spacing:0"></span>
        <button class="tiny" id="camtoggle" onclick="toggleCam()">start</button>
      </h2>
      <img id="cam" alt="camera off">
      <div class="grid2">
        <label>Resolution</label>
        <select id="c-res" onchange="pushCam()"></select><span class="val"></span>

        <label>FPS</label>
        <input type="range" id="c-fps" min="2" max="30" oninput="camLabel()" onchange="pushCam()">
        <span class="val" id="v-fps"></span>

        <label>Quality</label>
        <input type="range" id="c-q" min="10" max="95" oninput="camLabel()" onchange="pushCam()">
        <span class="val" id="v-q"></span>

        <label>Rotation</label>
        <select id="c-rot" onchange="pushCam()">
          <option value="0">0°</option><option value="90">90° (software)</option>
          <option value="180">180°</option><option value="270">270° (software)</option>
        </select><span class="val"></span>

        <label>Denoise</label>
        <select id="c-dn" onchange="pushCam()">
          <option value="cdn_off">cdn_off (cheapest)</option>
          <option value="off">off</option><option value="cdn_fast">cdn_fast</option>
          <option value="auto">auto</option><option value="cdn_hq">cdn_hq (costly)</option>
        </select><span class="val"></span>
      </div>
      <div class="camnote">
        Lowest latency: 640×480, 15 fps, quality 50, rotation 0/180, denoise cdn_off.
        90°/270° are decoded and re-encoded in Python — they will cost you frames.
        The camera process only runs while this panel is streaming, and stands down
        automatically while Web Teleop is up.
      </div>
    </div>
  </div>
</div>

<div class="shaftwrap">
  <div class="card">
    <h2>Shaft inspection<span class="grow"></span>
      <span id="shaftnote" style="text-transform:none;letter-spacing:0"></span>
      <button class="tiny" onclick="shaftReload()">reload</button>
      <button class="tiny" onclick="window.open(shaftURL(),'_blank')">open in a tab ↗</button>
    </h2>
    <div class="warnbar">
      Flight procedure: fly the drone manually to a stable hover over the shaft mouth,
      confirm <b>Perception</b> below shows a bore fix, then switch to <b>Offboard</b>.
      The mission takes over from there. Moving the sticks at any point returns control
      to you instantly (PX4 leaves Offboard); the mission node returns to WAIT and will
      not resume by itself.
    </div>
    <div id="shaftoff">
      Dashboard is not running.<br>
      Start the <b>Shaft inspection</b> sequence above, or just the
      <b>Shaft Dashboard</b> node, and the live view appears here.
    </div>
    <iframe id="shaftframe" style="display:none" title="Shaft inspection dashboard"></iframe>
  </div>
</div>

<div class="toast" id="toast"></div>

<script>
let UNITS = {}, GROUPS = {}, watching = null, ws = null, camOn = false;

function toast(msg){
  const t = document.getElementById('toast');
  t.textContent = msg; t.classList.add('show');
  clearTimeout(t._h); t._h = setTimeout(()=>t.classList.remove('show'), 2200);
}
async function post(url, body){
  const r = await fetch(url, {method:'POST', headers:{'Content-Type':'application/json'},
                             body: body ? JSON.stringify(body) : null});
  return r.json();
}
async function act(key, action){
  const r = await post(`/api/unit/${key}/${action}`);
  const name = UNITS[key] ? UNITS[key].label : key;
  toast(`${name}: ${r.result || r.error}`);
}
async function seq(name, action){
  const r = await post(`/api/sequence/${name}/${action}`);
  toast(r.result || r.error);
}
async function stopAll(){
  if(!confirm('Stop every managed node? (systemd services stay up)')) return;
  toast((await post('/api/stop_all')).result);
}

/* ---- unit list ---- */
function renderUnits(units){
  const host = document.getElementById('units');
  const byGroup = {};
  units.forEach(u => (byGroup[u.group] = byGroup[u.group] || []).push(u));

  if(!host._built){
    let html = '';
    for(const g of Object.keys(GROUPS)){
      if(!byGroup[g]) continue;
      html += `<h2>${GROUPS[g]}</h2>`;
      for(const u of byGroup[g]){
        html += `<div class="unit" id="u-${u.key}">
          <span class="dot"></span>
          <span class="uinfo">
            <span class="uname">${u.label}${u.kind==='service'?' <span style="font-size:10px;color:var(--dim)">systemd</span>':''}</span>
            <span class="udesc" title="${u.cmd.replace(/"/g,'&quot;')}">${u.desc}</span>
          </span>
          <span class="umetric"></span>
          <span class="btns">
            <button class="go tiny" onclick="act('${u.key}','start')">start</button>
            <button class="stop tiny" onclick="act('${u.key}','stop')">stop</button>
            <button class="tiny" onclick="act('${u.key}','restart')">↻</button>
          </span></div>`;
      }
    }
    host.innerHTML = html;
    host._built = true;

    const sel = document.getElementById('logsel');
    sel.innerHTML = units.map(u=>`<option value="${u.key}">${u.label}</option>`).join('')
                  + '<option value="camera">Camera</option>';
    watch(units[0].key);
  }

  for(const u of units){
    const row = document.getElementById('u-'+u.key);
    if(!row) continue;
    const dot = row.querySelector('.dot');
    dot.className = 'dot' + (u.running ? ' on' : (u.last_exit ? ' err' : ''));
    const m = row.querySelector('.umetric');
    if(u.running && u.usage){
      const hot = u.usage.cpu > 70 ? ' class="hot"' : '';
      m.innerHTML = `<span${hot}>${u.usage.cpu.toFixed(0)}%</span> · <b>${u.usage.mem_mb}</b>MB`
                  + `<br>${fmtUp(u.uptime)} · pid ${u.usage.pid}`;
    } else if(u.running){
      m.innerHTML = `running<br>${fmtUp(u.uptime)}`;
    } else {
      m.innerHTML = u.last_exit === null || u.last_exit === undefined
        ? '<span style="color:var(--dim)">stopped</span>'
        : `<span style="color:${u.last_exit ? 'var(--red)':'var(--dim)'}">exit ${u.last_exit}</span>`;
    }
  }
}
/* ---- shaft dashboard panel ----
   The dashboard is its own websockets server on another port, not part of this
   app, so it is embedded rather than proxied. The iframe src is only set once
   the node reports running: pointing it at a dead port leaves the browser
   showing a connection error that never clears by itself. */
function shaftURL(){
  return `${location.protocol}//${location.hostname}:__SHAFT_PORT__/`;
}
function shaftReload(){
  const f = document.getElementById('shaftframe');
  if(f.style.display !== 'none') f.src = shaftURL() + '?t=' + Date.now();
}
let shaftUp = null;
function renderShaft(units){
  const u = units.find(x => x.key === 'shaft_dashboard');
  const mission = units.find(x => x.key === 'shaft_mission');
  const perc = units.find(x => x.key === 'shaft_perception');
  const up = !!(u && u.running);
  const note = document.getElementById('shaftnote');
  note.textContent = up
    ? `dashboard :__SHAFT_PORT__ · mission ${mission && mission.running ? 'up' : 'DOWN'}`
      + ` · perception ${perc && perc.running ? 'up' : 'DOWN'}`
    : 'stopped';
  note.style.color = (up && mission && mission.running && perc && perc.running)
    ? 'var(--green)' : 'var(--dim)';
  if(up === shaftUp) return;                 // only touch the iframe on a change
  shaftUp = up;
  const f = document.getElementById('shaftframe');
  const off = document.getElementById('shaftoff');
  if(up){
    // The node binds its socket a moment after the process starts.
    setTimeout(() => { f.src = shaftURL(); f.style.display = 'block'; off.style.display = 'none'; }, 1200);
  } else {
    f.style.display = 'none'; f.removeAttribute('src'); off.style.display = 'block';
  }
}

function fmtUp(s){
  if(s == null) return '';
  if(s < 60) return s + 's';
  if(s < 3600) return Math.floor(s/60)+'m'+(s%60)+'s';
  return Math.floor(s/3600)+'h'+Math.floor((s%3600)/60)+'m';
}

/* ---- logs ---- */
function watch(key){
  watching = key;
  document.getElementById('logsel').value = key;
  document.getElementById('log').textContent = '';
  if(ws && ws.readyState === 1) ws.send(JSON.stringify({type:'watch', key}));
}
async function clearLog(){
  await post(`/api/logs/${watching}/clear`);
  document.getElementById('log').textContent = '';
}
function appendLog(lines){
  const el = document.getElementById('log');
  const stick = document.getElementById('autoscroll').checked;
  for(const line of lines){
    const span = document.createElement('span');
    const low = line.toLowerCase();
    if(line.startsWith('[gcs]')) span.className = 'gcs';
    else if(low.includes('error') || low.includes('fatal')) span.className = 'err';
    else if(low.includes('warn')) span.className = 'warn';
    span.textContent = line + '\n';
    el.appendChild(span);
  }
  while(el.childNodes.length > 800) el.removeChild(el.firstChild);
  if(stick) el.scrollTop = el.scrollHeight;
}

/* ---- header stats ---- */
function renderSystem(s){
  const set = (id, txt, warn, hot) => {
    const el = document.getElementById(id);
    el.innerHTML = txt;
    el.className = 'stat' + (hot ? ' hot' : warn ? ' warm' : '');
  };
  set('s-cpu', `CPU <b>${s.cpu.toFixed(0)}%</b> <span style="opacity:.6">${
        s.per_cpu.map(c=>c.toFixed(0)).join('/')}</span>`, s.cpu>70, s.cpu>88);
  set('s-mem', `MEM <b>${(s.mem_used_mb/1000).toFixed(1)}G</b>/${(s.mem_total_mb/1000).toFixed(1)}G`,
      s.mem_pct>80, s.mem_pct>92);
  set('s-temp', `TEMP <b>${s.temp==null?'--':s.temp+'°C'}</b>`, s.temp>70, s.temp>80);
  set('s-load', `LOAD <b>${s.load1}</b> (${s.cores} cores)`, s.load1>s.cores, s.load1>s.cores*1.5);
}

/* ---- camera ---- */
function toggleCam(){
  camOn = !camOn;
  const img = document.getElementById('cam');
  img.src = camOn ? '/video_feed?t=' + Date.now() : '';
  if(!camOn) img.removeAttribute('src');
  document.getElementById('camtoggle').textContent = camOn ? 'stop' : 'start';
}
function camLabel(){
  document.getElementById('v-fps').textContent = document.getElementById('c-fps').value + ' fps';
  document.getElementById('v-q').textContent = 'q' + document.getElementById('c-q').value;
}
async function pushCam(){
  const [w,h] = document.getElementById('c-res').value.split('x').map(Number);
  const body = {width:w, height:h,
                fps:+document.getElementById('c-fps').value,
                quality:+document.getElementById('c-q').value,
                rotation:+document.getElementById('c-rot').value,
                denoise:document.getElementById('c-dn').value};
  await post('/api/camera', body);
  if(camOn){ // force the <img> to reconnect to the restarted pipeline
    const img = document.getElementById('cam');
    img.src = '/video_feed?t=' + Date.now();
  }
}
async function initCam(){
  const d = await (await fetch('/api/camera')).json();
  const res = document.getElementById('c-res');
  res.innerHTML = d.resolutions.map(([w,h])=>`<option value="${w}x${h}">${w}×${h}</option>`).join('');
  res.value = `${d.settings.width}x${d.settings.height}`;
  document.getElementById('c-fps').value = d.settings.fps;
  document.getElementById('c-q').value = d.settings.quality;
  document.getElementById('c-rot').value = d.settings.rotation;
  document.getElementById('c-dn').value = d.settings.denoise;
  camLabel();
}

/* ---- websocket ---- */
function connect(){
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen = () => {
    document.getElementById('s-link').innerHTML = 'LINK <b style="color:var(--green)">up</b>';
    if(watching) ws.send(JSON.stringify({type:'watch', key:watching}));
  };
  ws.onclose = () => {
    document.getElementById('s-link').innerHTML = 'LINK <b style="color:var(--red)">down</b>';
    setTimeout(connect, 1500);
  };
  ws.onmessage = ev => {
    const d = JSON.parse(ev.data);
    d.units.forEach(u => UNITS[u.key] = u);
    renderUnits(d.units);
    renderShaft(d.units);
    renderSystem(d.system);
    const s = d.sequence;
    document.getElementById('seqstate').textContent =
      s.active ? `${s.active}: ${s.step}${s.remaining ? ' — settling ' + s.remaining + 's' : ''}` : '';
    const c = d.camera;
    document.getElementById('camstat').textContent =
      c.blocked ? c.blocked : (c.live ? `${c.fps} fps · ${c.viewers} viewer(s)` : 'idle');
    if(d.log && d.log.key === watching) appendLog(d.log.lines);
  };
}

(async () => {
  const meta = await (await fetch('/api/units')).json();
  GROUPS = meta.groups;
  await initCam();
  connect();
})();
</script>
</body>
</html>
"""


def main():
    print(f"\n  Drone GCS Launcher  ->  http://0.0.0.0:{GUI_PORT}\n")
    # Belt and braces: uvicorn installs its own SIGINT/SIGTERM handlers and
    # normally returns cleanly, but atexit also covers the paths where it
    # doesn't, so the camera is never left running without us.
    import atexit
    atexit.register(camera.shutdown)
    try:
        uvicorn.run(app, host="0.0.0.0", port=GUI_PORT, log_level="warning",
                    ws_ping_interval=20, ws_ping_timeout=20)
    finally:
        camera.shutdown()


if __name__ == "__main__":
    main()
