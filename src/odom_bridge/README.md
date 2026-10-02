# LiDAR-anchored positioning for the GPS-denied indoor drone

## Why

EKF2's horizontal position was resetting to near-(0,0) whenever optical flow
quality dropped (low light / low texture / out-of-range). This chain removes
flow as the *only* horizontal aiding source and replaces it with a
LiDAR-anchored estimate that doesn't reset the same way:

```
LiDAR /scan
     │
     ▼
 rf2o_laser_odometry  (frame-to-frame scan matching)
     │  publishes: odom -> base_link TF   (replaces the TF your
     │                                      PX4 odom bridge used to publish)
     ▼
 slam_toolbox (online_async)  (scan-matches against a persistent map,
     │                          uses rf2o's odom as its motion prior,
     │                          does loop closure -> corrects drift)
     │  publishes: map -> odom TF + a slam pose
     ▼
 vision_odom_bridge.py  (new node — converts slam pose from
     │                    ROS ENU/FLU back to PX4 FRD-locked frame)
     ▼
 /fmu/in/vehicle_visual_odometry  ->  EKF2 fuses it (EKF2_EV_CTRL)
```

Your original `px4_odom_bridge.py` / `odom_converter.py` still runs — it now
only publishes `/odom` (for visualization/logging) and the static
`base_link -> lidar_sensor_link` TF. **It must stop broadcasting the dynamic
`odom -> base_link` TF**, since rf2o owns that now — two publishers on the
same TF edge will fight and corrupt the tree.

---

## 1. Install rf2o_laser_odometry

Not in apt for Jazzy as of now — build from source:

```bash
cd ~/px4_ros2_ws/src
git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git
cd ~/px4_ros2_ws
colcon build --packages-select rf2o_laser_odometry
source install/setup.bash
```

## 2. Stop your PX4 odom bridge from publishing the dynamic TF

In `px4_odom_bridge.py`, comment out (or delete) the `TransformStamped`
broadcast block inside `odom_cb()` — keep everything else (the `/odom`
publish, the static lidar TF, the gz bridge subprocess spawning).

## 3. Launch file: rf2o + slam_toolbox + vision bridge

See `rf2o_slam_launch.py`. Key points:
- rf2o subscribes to `/scan`, publishes `odom -> base_link` TF and a
  `nav_msgs/Odometry` on `/odom_rf2o` — **not** `/odom`, so it doesn't
  collide with the PX4 odom bridge's own `/odom` topic (they can coexist as
  long as they're on different topic names; only the TF edge is exclusive).
- slam_toolbox's `odom_frame` param stays `odom` — it'll pick up rf2o's TF
  automatically without further wiring, since TF is global.
- `vision_odom_bridge.py` reads the `map -> base_link` TF (composed by tf2
  from `map->odom` + `odom->base_link`), not a raw topic, so it always gets
  the fully corrected pose regardless of which node last updated which link.

## 4. slam_toolbox params

See `slam_toolbox_params.yaml`. Set `odom_frame: odom`, `base_frame:
base_link`, `map_frame: map`, `scan_topic: /scan`, `mode: mapping`.

## 5. EKF2 parameters (set via QGC param editor or `param set` on the MAVLink
console)

| Param | Value | Why |
|---|---|---|
| `EKF2_EV_CTRL` | enable HPOS + YAW bits (add VPOS if you want SLAM height too) | turns on vision fusion |
| `EKF2_EV_DELAY` | measured pipeline latency (start ~50-100ms, tune from .ulg) | mistimed fusion looks noisy/gets rejected |
| `EKF2_OF_CTRL` | lower priority or 0 once vision fusion is verified working | stops flow from still yanking the estimate |
| `EKF2_HGT_REF` | keep baro/rangefinder as height reference unless you enable VPOS above | avoid two disagreeing height sources |

**Verify before disabling flow**: check the `.ulg` for `EV` innovations
staying small and no new reset events, over the same conditions that used to
trigger the flow-quality reset (dim lighting, blank walls, etc).

## 6. Test

Re-run the scenario that broke before. Watch `vehicle_odometry`'s
`reset_counter` — it should stop incrementing under the same conditions that
used to trigger it, since the map-anchored SLAM pose doesn't degrade with
lighting/texture the way flow does.
