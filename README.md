# egrobots-rover-actions — Goal-Based Robot Movement with ROS 2 Actions (Humble)

The Egrobots rover accepts a movement goal, drives to it while avoiding
obstacles, reports progress along the way, and can be cancelled mid-flight.

Built for the Egrobots ROS 2 Week 3 Task (Goal-Based Robot Movement), extending
the Week 2 rover with its URDF, TF2 localization, and IMU/EKF sensor fusion.

---

## 1. Why an action, and not a service

This is the point of the task, so it is worth being precise.

A **service** is one request paired with one response. Once called there is no
channel back to the caller until it returns. A **topic** is a one-way stream with
no notion of completion at all. An **action** is a goal, a stream of feedback
while the work runs, and a final result — and the goal can be cancelled or
rejected.

Two of this task's requirements are not merely awkward over a service, they are
structurally impossible:

| ID | Requirement | Service | Action |
|---|---|---|---|
| R1 | Accept a target position | ✅ | ✅ |
| R2 | Move the robot toward it | ⚠️ caller blocks for the whole drive | ✅ |
| R3 | Avoid obstacles while moving | ✅ | ✅ |
| R4 | Indicate the target is reached | ✅ | ✅ |
| **R5** | **Cancel an active movement** | ❌ **no channel back** | ✅ |
| **R6** | **Feedback while moving** | ❌ **response comes only at the end** | ✅ |
| R7 | Handle an unreachable target | ✅ | ✅ |

Driving to a goal takes tens of seconds, can fail, and the operator wants to
watch it. That is the shape of work an action exists for.

**The geofence behaviour from Week 2 is deliberately left on plain services.**
`/start_avoidance` sets a flag and returns in microseconds — a switch, not a
task. Keeping both mechanisms side by side in one node is the clearest available
demonstration that the choice should follow the shape of the work: a toggle stays
a service, a long-running job becomes an action.

---

## 2. Package Layout

Two packages, because a custom action interface cannot be generated from a
Python package — `rosidl` requires `ament_cmake`.

| Package | Build type | Contents |
|---|---|---|
| `egrobots_rover_interfaces` | `ament_cmake` | `action/MoveToGoal.action` |
| `egrobots_rover_navigation` | `ament_python` | The rover node, URDF, launch, config, RViz |

### The action interface

```
# Goal — R1
geometry_msgs/Point target
float64 tolerance               # <= 0 uses the node's default
---
# Result — R4, R7
bool success
string message
float64 final_distance_error
geometry_msgs/Point final_position
---
# Feedback — R6
float64 distance_remaining
float64 heading_error_deg
geometry_msgs/Point current_position
string state                    # DRIVING | TURNING | AVOIDING | CLEARING
```

`tolerance` is per-goal so a caller can ask for a loose or tight arrival without
reconfiguring the node. The result carries a `message` because "it failed" is not
useful on its own — R7 needs to say *why*.

---

## 3. Build & Run

```bash
cd ~/ros2_ws
colcon build --packages-select egrobots_rover_interfaces egrobots_rover_navigation
source install/setup.bash
```

```bash
ros2 launch egrobots_rover_navigation rover.launch.py
```

Send a goal and watch feedback stream (`-f`):

```bash
ros2 action send_goal -f /move_to_goal egrobots_rover_interfaces/action/MoveToGoal \
  "{target: {x: 5.0, y: 0.0, z: 0.0}, tolerance: 0.0}"
```

Press `Ctrl+C` during that command to cancel the movement (R5) — the rover stops
and the result comes back as `CANCELED`, not as a crash.

The Week 2 geofence patrol is still available on services:

```bash
ros2 service call /start_avoidance std_srvs/srv/Trigger
ros2 service call /stop_avoidance  std_srvs/srv/Trigger
```

Obstacles are not baked into the world; add them from the Gazebo GUI.

---

## 4. ROS 2 Interface

**Action:** `/move_to_goal` (`egrobots_rover_interfaces/action/MoveToGoal`)

**Services:** `/start_avoidance`, `/stop_avoidance` (`std_srvs/srv/Trigger`)

**Subscribes:** `/scan`, plus `/tf` and `/tf_static`.

**Publishes:** `/cmd_vel` (remapped to `/diff_drive_controller/cmd_vel_unstamped`),
`/robot_pose`, `/detected_obstacle`, `/geofence_marker`, and the
`detected_obstacle` frame on `/tf`.

Frames are unchanged from Week 2: `odom → base_link → mast → sensor_mount`, with
`odom → base_link` published by `ekf_filter_node` fusing wheel velocity with IMU
heading.

---

## 5. Design Decisions

**A MultiThreadedExecutor is required, not a preference.** An action's execute
callback loops until the goal completes. Under the default single-threaded
executor that loop monopolises the node, `scan_callback` never fires, and the
rover drives blind and never finishes — it hangs. The scan subscription, the
timers, and the action server therefore sit in separate callback groups and the
node is spun by a `MultiThreadedExecutor`. Shared state is guarded by a lock
because callbacks now genuinely run on different threads.

**Goals are rejected rather than queued.** `goal_callback` declines a new goal
while another is running, while the geofence patrol is active, or before TF has
produced a pose. Rejection is itself an action feature with no service
equivalent — declining the work before starting it, with a reason. It also
guarantees only one behaviour ever drives `/cmd_vel`.

**Obstacle avoidance outranks the goal.** Each control cycle runs the avoidance
state machine first; only if it declines to steer does goal-seeking get the
command. A collision reflex must not wait on a navigation decision, and it reads
only the LiDAR, so it never depends on odometry.

**R7 is a stall detector, not a map.** The rover aborts when it has failed to
close the gap by `stall_min_progress` within `stall_timeout` seconds. This covers
being boxed in, wedged against geometry, and a target that simply cannot be
reached — without needing a map or a planner to reason about reachability. A
separate `goal_timeout` bounds the total attempt. The abort message names the
distance it gave up at, so the caller can tell "unreachable" from "gave up early".

**Speed is capped by proximity to the goal in every phase**, including while
clearing an obstacle. Clearing drives at full speed, so a target inside its
clearing run would otherwise be crossed at 1.0 m/s and overshot by the rover's
0.25 m stopping distance.

**Turn to face, then drive straight.** Beyond `heading_tolerance_deg` the rover
pivots rather than arcing. A pivot scrubs more per degree, but it minimises
*total* rotation, and rotation is where nearly all of this platform's odometry
error originates.

---

## 6. Verified Behaviour

Measured against Gazebo ground truth:

| Requirement | Test | Result |
|---|---|---|
| R1, R2, R4 | Goal (4.0, 0.0) | `SUCCEEDED`, truth (3.91, −0.01) |
| R3 | Wall between rover and target | Drove around it |
| R5 | Cancel 4 s into a 9 m goal | `CANCELED` after 41 feedback msgs; rover halted and stayed halted |
| R6 | Any goal | Feedback at 10 Hz with distance, heading error, state |
| R7 | Target inside a 2×4 m wall | `ABORTED` — "no progress for 12 s (stuck 2.86 m from the target)" |
| — | Goal sent while geofence running | `REJECTED`; accepted after `/stop_avoidance` |

Localization accuracy carried over from Week 2: a 7 m straight run finished with
odometry at 7.068 m against a true 7.056 m — 1.2 cm.

---

## 7. Known Limitations

- **No path planning.** Avoidance is reactive: the rover turns away and drives on.
  It can be defeated by a concave obstacle, where it will circle until the stall
  detector aborts. That abort is correct behaviour, but a planner would route
  around instead of giving up.
- **Odometry still drifts.** The EKF slows accumulation but cannot bound it —
  there is no absolute reference. Long missions degrade. LiDAR localization
  against a map (`nav2_amcl` / `slam_toolbox`) publishing `map → odom` is the
  remaining fix, and is out of scope here.
- **The simulated IMU is better than a real one.** Gazebo derives it from ground
  truth plus configured noise; a physical IMU has gyro bias drift that would
  return some of the error.
- **One goal at a time.** Concurrent goals are rejected rather than queued or
  preempted. Preemption would be the natural extension.
- **The stall detector cannot distinguish "unreachable" from "slow".** A rover
  legitimately taking a long detour around a large obstacle can trip it; the
  thresholds are tuned for the test world, not proven generally.
