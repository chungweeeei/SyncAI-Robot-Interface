# syncai_common

The stack's shared ROS 2 interface definitions — 15 messages, 16 services, 2
actions. No code, no nodes: `rosidl_generate_interfaces` only.

An interface lives here only when two or more packages share it; single-package
interfaces stay in their package.

```
syncai_driver_manager ──IMUState / MotorStates──►  (telemetry consumers)
        ▲  SetMotionKey / SetPolicyMode / SetSpeedScale       │ MotorStates
        │                                                     ▼
syncai_backend ─────────┼──────────► syncai_robot_state ──RobotState──► syncai_backend
        │  ScanWifiNetworks / ConnectWifiNetwork              ▲
        ▼                                                     │ WifiStatus
syncai_sys_manager ───────────────────────────────────────────┘

syncai_backend ──StartMapping / SaveMaps / ResetMapping──► syncai_mapping (pgo_node) ──ResetLIO──► syncai_pointlio
syncai_backend ◄──MappingStatus──────────────────────────── syncai_mapping (pgo_node)
syncai_backend ──Relocalize / IsValid──────► syncai_localizer (localizer_node)
syncai_backend ──NavigateToGoal────────────► syncai_task_runner   (the MOVE step, once it migrates off nav2_msgs/NavigateToPose)
```

## This repository

It is its own repo so consumers import *this*, not the whole workspace. It
depends only on `ament_cmake`, `rosidl_default_generators`,
`builtin_interfaces` and `geometry_msgs` — no first-party package. Keep it that
way: an interface package that depends on its consumers is no longer an
interface.

Consumers pull it in with vcstool:

```bash
vcs import < interface.repos          # -> src/syncai_common
colcon build --packages-select syncai_common
```

Consumers: `syncai_backend`, `syncai_robot_state`, `syncai_sys_manager`,
`syncai_driver_manager`, `syncai_pointlio`, `syncai_mapping`,
`syncai_localizer` and `syncai_task_runner`. Any change here is an ABI break
for all of them — see **Gotchas**.

## Messages

### Robot state aggregate

`RobotState` is published by `syncai_robot_state` on the relative topic
`robot_state` (BEST_EFFORT, KeepLast(1)) at 1 Hz as shipped (`publish_rate` in
`syncai_robot_state/params/robot_state_params.yaml`; code default 10 Hz).
`syncai_backend` re-serialises **a subset** of it for
`GET /api/v1/robot/state`.

```
RobotState
├─ uint64 timestamp                 SECONDS (whole seconds — see the gotcha)
├─ string robot_id, map
├─ uint8  mode                    ← constants from RobotMode
├─ uint8  state                   ← constants from RobotStatus
├─ RobotLocalizationStatus localization_status
│    ├─ RobotPose position        (x, y, z, yaw — yaw in RADIANS)
│    └─ float64 velocity          (forward linear speed from odom)
├─ RobotNetworkStatus network_status
│    └─ string wifi_info          ← a JSON object, see below
├─ RobotBatteryStatus battery_status
│    └─ float64 battery_percentage
├─ bool          localization_valid   false ⇒ localization_status is ZEROED
├─ MotorStates   motor_status         the whole motor_states message (timestamp rescaled)
│    ├─ uint64        timestamp       SECONDS here (the topic carries NANOSECONDS)
│    └─ MotorState[]  states          per-joint temperature / tau_est / error
└─ RobotLowLevelMode low_level_mode   the GAIT CONTROLLER's state machine — not `mode` above
     ├─ int32  policy_state           0 PPO / 1 HIMLOCO / 2 CHAMP / 3 ISSAC (no sentinel)
     ├─ int32  motion_state           0 Stand / 1 Locomotion / 2 LieDown / 3 Damping /
     │                                4 ESTOP / 8 UNKNOWN (MPC's code is unknown)
     └─ bool   safety_state           driver_manager's safety lock engaged (OURS, not the controller's)
```

`localization_valid` and `motor_status` **must not reach the REST payload**
(`low_level_mode` is exposed deliberately). That payload is a frozen
third-party contract whose fields the router names one by one. Adding a field
here must not add one there.

`RobotState` stays on a per-robot relative topic: one DDS domain hosts several
robots, so a shared absolute `/robot_state` would interleave them.

| Message | Fields | Notes |
|---|---|---|
| `RobotPose` | `x`, `y`, `z`, `yaw` | Plain floats, **not** `geometry_msgs/Pose`. `yaw` in radians; REST converts to degrees. |
| `RobotLocalizationStatus` | `position`, `velocity` | |
| `RobotNetworkStatus` | `wifi_info` | A JSON string — see below |
| `RobotBatteryStatus` | `battery_percentage` | 0–100, scaled from `sensor_msgs/BatteryState.percentage` |
| `RobotMode` | `MAINTENANCE=0`, `MANUAL=1`, `AUTO=2` | **Constants only**, names the values of `RobotState.mode`. |
| `RobotStatus` | `UNINITIALIZED=0`, `IDLE=1`, `RUNNING=2`, `WARNING=3`, `ERROR=4`, `CHARGING=5` | Constants only, for `state`. `0` is `UNINITIALIZED` so a default-constructed message does not claim `IDLE`. |
| `RobotLowLevelMode` | `policy_state`, `motion_state`, `safety_state` | The gait controller's state machine, from the `mode` topic. Both indices are the **controller's** vocabularies and carry **no constants** (see `SetPolicyMode`). No freshness field: `0 / 0` before the first sample looks like a real `PPO / Stand`. `safety_state` is instead `syncai_driver_manager`'s own lock (latched `safety_locked` topic). Nothing triggers that lock outside tests yet, so `false` is not a checked safety signal. |

`mode` is the live byobu session. `syncai_robot_state` polls
`syncai_sys_manager`'s `get_mode` at 1 Hz (`mode_poll_rate`, one request in
flight) because sys_manager publishes no mode topic. It reads `AUTO` until the
first answer, so a mapping session shows `AUTO` for up to one poll before
`MANUAL`. The REST layer exposes it.

`state` uses three of the six values, most severe first:

- `UNINITIALIZED` — the `map → base_link` TF did not resolve, so
  `localization_status` is zeroed. It outranks `WARNING` so a low battery never
  hides that the pose is a placeholder. `localization_valid` comes from the same
  lookup, so the two cannot disagree.
- `WARNING` — battery below 20%, with hysteresis (clears above 25%).
- `IDLE` — otherwise.

`RUNNING` and `ERROR` are not emitted yet. **`CHARGING` cannot be derived**:
`syncai_driver_manager` hardcodes `BatteryState.power_supply_status` to
`UNKNOWN`, and the sign convention of `current` is undocumented. REST does not
expose `state`.

**Why `wifi_info` is a JSON string.** `syncai_robot_state` flattens the latest
`WifiStatus` into `{"ssid", "bssid", "rssi", "ip_address", "mac_address"}`.
Before the first `wifi_status` it is the literal `"null"`, so the backend parses
it defensively. A string keeps the aggregate stable while wifi reporting is in
flux.

### Wifi

| Message | Fields | Used by |
|---|---|---|
| `WifiNetwork` | `bssid`, `ssid`, `rssi` | A scan result, returned by `ScanWifiNetworks`. |
| `WifiStatus` | `bssid`, `ssid`, `rssi`, `ip_address`, `mac_address` | Published at 1 Hz on `wifi_status` by `syncai_sys_manager`; consumed by `syncai_robot_state`. |

`rssi` is `int8` dBm (roughly −100…0).

### Driver telemetry

Published by `syncai_driver_manager` from the gait controller's UDP telemetry.

| Message | Topic | Fields |
|---|---|---|
| `IMUState` | `imu` (SensorDataQoS) | `timestamp`, `quaternion[4]`, `gyroscope[3]`, `accelerometer[3]`, `rpy[3]`, `temperature` |
| `MotorStates` | `motor_states` (SensorDataQoS) | `timestamp` (**nanoseconds here**) + `MotorState[] states`. Also nested as `RobotState.motor_status`, where `timestamp` is **seconds** |
| `MotorState` | — | `name`, `q`, `dq`, `ddq`, `tau_est`, `temperature`, `error` |

Custom rather than `sensor_msgs/Imu` / `JointState` because they carry
per-motor `temperature` and `error`, and `IMUState` mirrors the controller's
layout.

### MappingStatus

`pgo_node`'s run state on the relative topic `mapping_status`
(`<robot_id>/pgo/mapping_status`), **RELIABLE + TRANSIENT_LOCAL depth 1** so a
reconnecting backend gets the state at once. Sent on every transition and at
1 Hz. Consumed by `syncai_backend` for `GET /api/v1/mapping`.

```
uint8 IDLE      = 0   # no graph, intake dropped, /dev/shm empty; the session starts here
uint8 MAPPING   = 1   # keyframes accumulate; reset_mapping discards and stays here
uint8 RESETTING = 2   # transient: a start/reset is waiting on the ResetLIO round trip
uint8 state
uint32 key_poses
uint32 loop_closures
builtin_interfaces/Time stamp
```

`start_mapping`: IDLE → MAPPING. Successful `save_maps`: MAPPING → IDLE.
`reset_mapping`: MAPPING → MAPPING. `IDLE = 0` so a default-constructed message
reads as the safe state.

### ArtifactState

**No publisher or subscriber in this workspace.** It mirrors the interface of
the separate artifact stack (`SyncAI-Artifact-Workspace`) for a future
ROS-side artifact monitor.

```
uint64 timestamp     # ms since epoch
string artifact_id   # "conveyor_0", "door_1"
string type          # live_info discriminator: "conveyor", "door", ...
bool   connected     # transport session (e.g. modbus tcp) is up
bool   stale         # connected but the device stopped updating
uint16 error_code    # device-reported, 0 = ok
string live_info     # JSON of DECODED values, never raw registers
```

The four health fields are uniform so a monitor can alarm without parsing;
everything type-specific goes in `live_info`, keyed by `type`. A conveyor's
`live_info.phase` is the backend's `ConveyorPhase`
(`belt`/`handoff`/`carried`/`dropped`).

## Services

| Service | Request | Response | Served by |
|---|---|---|---|
| `ScanWifiNetworks` | *(empty)* | `success`, `message`, `WifiNetwork[] networks` | `syncai_sys_manager` on `scan_wifi` |
| `ConnectWifiNetwork` | `ssid`, `password` | `success`, `message` | `syncai_sys_manager` on `connect_wifi` |
| `SetMotionKey` | `key` | `success`, `message` | `syncai_driver_manager` on `set_motion_key` |
| `SetPolicyMode` | `uint8 mode` | `success`, `message` | `syncai_driver_manager` on `set_policy_mode` |
| `SetSpeedScale` | six `float64` scales | `success` | `syncai_driver_manager` on `set_speed_scale` |
| `SwitchMode` | `uint8 mode` | `success`, `message` | `syncai_sys_manager` on `switch_mode` |
| `GetMode` | *(empty)* | `success`, `message`, `uint8 mode`, `string session` | `syncai_sys_manager` on `get_mode` |
| `RestartMode` | *(empty)* | `success`, `message`, `uint8 mode`, `string session` | `syncai_sys_manager` on `restart_mode` |
| `ResetLIO` | *(empty)* | `success`, `message`, `float64 last_odom_time` | `syncai_pointlio` on `pointlio/reset` |
| `SaveMaps` | `file_path`, `save_patches` | `success`, `message` | `syncai_mapping` on `pgo/save_maps` |
| `ResetMapping` | `reset_lio` | `success`, `message`, `float64 lio_last_odom_time`, `uint32 dropped_key_poses` | `syncai_mapping` on `pgo/reset_mapping` |
| `StartMapping` | `reset_lio` | `success`, `message`, `float64 lio_last_odom_time` | `syncai_mapping` on `pgo/start_mapping` |
| `RefineMap` | `maps_path` | `success`, `message` | `syncai_mapping` (`hba_node`, offline, by hand) on `hba/refine_map` |
| `SavePoses` | `file_path` | `success`, `message` | `syncai_mapping` (`hba_node`) on `hba/save_poses` |
| `Relocalize` | `pcd_path`, `x`, `y`, `z`, `yaw`, `pitch`, `roll` (`float32`, radians) | `success`, `message` | `syncai_localizer` on `relocalize` (bare: `<robot_id>/relocalize`) |
| `IsValid` | `int32 code` | `bool valid` | `syncai_localizer` on `relocalize_check` |

Notes:

- **`SetMotionKey.key` is a string**, forwarded verbatim to the gait
  controller: `0` stand, `1` locomotion, `2` lie down, `3` damping, `4`
  emergency stop, `5` MPC. Backend callers: the `STANDUP` / `LIEDOWN` task
  steps (`syncai_backend/temporal/activities.py`) and
  `POST /api/v1/robot/set_motion_key`. That endpoint **does not forward `"4"`**
  (answers 200 with `sent: false`): an e-stop over unauthenticated HTTP on an
  unacknowledged one-way link is not a safety guarantee.
- **`SetSpeedScale` has six scales** (`fwd`, `back`, `left`, `right`,
  `turn_l`, `turn_r`) because the controller tracks velocity asymmetrically;
  the driver manager applies them to `cmd_vel`. Its response has no `message`.
- **`SetPolicyMode.mode` is a bare `uint8`**, the controller's policy index —
  not `RobotMode`. `POST /api/v1/robot/set_policy_mode` accepts only `0` (PPO)
  and `1` (HIMLOCO); non-REST callers can still send `2` (CHAMP) or `3`
  (ISSAC). The vocabulary is documented only in `msg/RobotLowLevelMode.msg`,
  which explains why neither place declares constants.
- **`ResetLIO` and `ResetMapping` are one contract.** The backend calls
  `reset_mapping` on `pgo_node`, which pauses intake, calls `ResetLIO` on
  `pointlio_node`, rebuilds its graph and resumes, dropping every cloud/odom
  pair at or before the returned `last_odom_time`. `ResetLIO`'s request is
  empty on purpose (a reset is not a reconfigure); `reset_lio: false` resets
  the graph alone, for bag replay. The `.srv` comments are the design record.
  **The robot must be standing still** during a reset — the LIO re-runs a
  static IMU init, and nothing enforces it.
- **`StartMapping` is the same sequence from IDLE.** pgo starts idle and banks
  nothing until `start_mapping` (refused while MAPPING, as `reset_mapping` is
  refused while IDLE). A successful `SaveMaps` returns pgo to IDLE, frees its
  keyframes and clears `/dev/shm`, so a run is `start_mapping … save_maps`.
  The stillness rule applies here too.
- **`SaveMaps` writes the layout the map catalogue depends on** (`map.pcd`,
  `patches/<i>.pcd`, `poses.txt` with bare basenames); the `.srv` documents it.
- **`RefineMap` / `SavePoses` are offline mapping** (`hba_node`): load the
  `patches/` + `poses.txt` from a `SaveMaps` with `save_patches: true`, refine
  the poses, write them to a separate file. Nothing in a session or the
  backend calls them.
- **`Relocalize` success is a receipt; `IsValid` is the result.** Registration
  runs async; `IsValid` with `code: 0` reports whether the first registration
  after the guess converged (`code: 1` always returns `valid: true`, a liveness
  probe). The backend follows `Relocalize` with an `initialpose` publish — the
  tilted lidar mount means a flat guess never converges. Both are under the
  bare robot_id namespace (`<robot_id>/relocalize`).
- **`RestartMode` restarts the live mode's session.** `SwitchMode` to the live
  mode is a no-op, so this is the only way to restart a wedged stack. Refused
  in MAINTENANCE, with both sessions up, and **always** in MANUAL (`pgo_node`
  may hold an unsaved map) — so in practice it restarts AUTO only. The backend
  does not call it yet.

Convention: callers check `success` and surface `message` verbatim (e.g. a
failed wifi connect becomes HTTP 400 with that string).

## Actions

### NavigateToGoal

`nav2_msgs/NavigateToPose` plus a failure reason.

- **Goal:** `pose` (`PoseStamped`, in `map`), `behavior_tree` (absolute XML
  path; empty = default `move.xml`).
- **Result:** `error_code`, `error_msg`.
- **Feedback:** nav2's `current_pose`, `navigation_time`,
  `estimated_time_remaining`, `number_of_recoveries`, `distance_remaining`,
  plus `number_of_replans` (plans after the first, i.e. detours) and
  `status_msg` (a free-text English sentence for the operator console, e.g.
  "Following the path" — never switch on it).

Served by `syncai_task_runner` on `navigate_to_goal`
(`<robot_id>/navigate_to_goal`), alongside the **unchanged**
`nav2_msgs/NavigateToPose` on `navigate_to_pose` — same navigator, tree and
preemption rules. It exists so the backend's MOVE step can tell the operator
whether the robot failed to plan or to drive. The backend still uses the nav2
action; that one is removed after the backend migrates.

| `error_code` | Meaning |
|---|---|
| `NONE` = 0 | Also carried by `SUCCEEDED` and `CANCELED`. **`ABORTED` + `NONE` is not a navigation failure**: the goal was preempted by a newer one, or rejected before the tree ran (other navigator busy, or `behavior_tree` failed to load) |
| `UNKNOWN` = 1 | The tree failed with no node reporting: an exception, or a server that never acknowledged the goal (`error_msg` names it). 1–99 is the navigator / tree range |
| `FOLLOW_PATH_FAILED` = 100 | Path following failed after retries: blocked with no detour, no progress, or robot pose lost. 1xx is the controller range |
| `PLAN_FAILED` = 200 | Planning failed twice (after a global-costmap clear): no route, start or goal in an obstacle / keepout zone, or TF. 2xx is the planner range |

Ranges follow nav2 Iron's, so finer codes can be added as `base + n` once
Humble's planner/controller results carry a reason. Clients may switch on the
hundreds; the two bases are never renumbered. Field names mirror nav2's
because one navigator template in `syncai_task_runner` serves both actions.

### ExecuteTask

Goal `uuid` / `timestamp` / `behavior_tree` (an **inline BT XML string**),
result `success` / `message` / `finished_timestamp`, feedback `status` /
`elapsed_ms`.

**Nothing implements or calls it.** The backend's `RobotWorkflow` sequences
steps itself (`MOVE` → nav2, `ARTIFACT` → artifact REST API). It is kept in
case tick-level parallelism ever needs a whole tree on the robot.

## Depending on this package

C++ (`CMakeLists.txt` + `package.xml`):

```cmake
find_package(syncai_common REQUIRED)
ament_target_dependencies(${target} syncai_common)   # or list it in `set(dependencies …)`
```

```xml
<depend>syncai_common</depend>
```

```cpp
#include "syncai_common/msg/robot_state.hpp"
#include "syncai_common/srv/set_motion_key.hpp"
```

Python (once the workspace is sourced):

```python
from syncai_common.msg import RobotState, RobotMode
from syncai_common.srv import ScanWifiNetworks, ConnectWifiNetwork, SetMotionKey
from syncai_common.action import NavigateToGoal
```

Use constant-only messages as class attributes:

```python
if state.mode == RobotMode.AUTO: ...
```

## Build

```bash
colcon build --packages-select syncai_common
source install/setup.bash
```

Then rebuild every dependent package — generated code does not update in
place, and `--symlink-install` does not help:

```bash
colcon build --packages-up-to syncai_robot_state syncai_driver_manager
```

Inspect the generated interfaces:

```bash
ros2 interface show syncai_common/msg/RobotState
ros2 interface list | grep syncai_common
```

## Gotchas

- **Changing a field is an ABI break.** Rebuild *and restart* every node using
  the message; a mismatch fails at the type-hash level with no useful error. On
  a live robot, rebuild the whole workspace.
- **Renumbering a constant is a silent ABI break.** Constants compile into
  consumers, so nothing fails loudly — but old bags decode to the wrong names
  (as happened when `RobotStatus` gained `UNINITIALIZED = 0`).
  `NavigateToGoal`'s `100` / `200` bases are fixed because the backend switches
  on them.
- **Timestamp units differ.** `ArtifactState` and `ExecuteTask`: milliseconds.
  `RobotState.timestamp` and `motor_status.timestamp`: **seconds** (the
  frontend multiplies by 1000). The `motor_states` topic: **nanoseconds**
  (the telemetry WebSocket needs sub-second ordering at 20 Hz). Each `.msg`
  states its unit — check before mixing.
- **`RobotState.timestamp` cannot order samples.** Whole seconds repeat at
  10 Hz, and jitter can repeat them at 1 Hz. Use it for display; subscribe
  `motor_states` for sub-second resolution.
- **No message has a `std_msgs/Header`** and none has a `frame_id`; timestamps
  are bare `uint64` (except `MappingStatus.stamp`, a
  `builtin_interfaces/Time`). Anything needing TF uses a `geometry_msgs` type,
  as `NavigateToGoal` does. `RobotLowLevelMode` has no timestamp at all — its
  upstream source carries no clock.
- **Constant-only messages (`RobotMode`, `RobotStatus`) are publishable types
  with zero fields.** Publishing one is legal and meaningless.
- **Every new interface must be listed in `CMakeLists.txt`** — the directories
  are not globbed. (The `.gitkeep` files in `msg/`, `srv/`, `action/` are
  harmless leftovers.)
