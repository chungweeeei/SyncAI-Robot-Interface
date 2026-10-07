# syncai_common

The stack's shared ROS 2 interface definitions — 15 messages, 15 services, 1
action. No code, no nodes: `rosidl_generate_interfaces` and nothing else.

Everything here exists because two or more packages need to agree on a wire
format. Interfaces used by exactly one package generally stay in that package;
these crossed a boundary.

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
```

## This repository

`SyncAI-Robot-Interface` — the package lives at the repo **root**, the same
layout `SyncAI-Robot-Backend` uses. It was split out of `SyncAI-Robot-Workspace`
in 2026-09 with `git subtree split`, so the history below predates the split.

It is the one thing every other repo has to agree on, which is exactly why it is
its own repo: a consumer imports *this*, not the whole workspace. It depends on
nothing but `ament_cmake`, `rosidl_default_generators` and `builtin_interfaces`
— no first-party package, no workspace path. Keep it that way; an interface
package that needs one of its own consumers is no longer an interface.

Consumers materialise it with vcstool, from their own root:

```bash
vcs import < interface.repos          # -> src/syncai_common
colcon build --packages-select syncai_common
```

Known consumers: `syncai_backend`, `syncai_robot_state`, `syncai_sys_manager`,
`syncai_driver_manager`, `syncai_pointlio`, `syncai_mapping` and
`syncai_localizer`, which serves `Relocalize` / `IsValid`. The localizer used
to be the one consumer in a third repository (`SyncAI-Fast-LIO2`); it moved
into the workspace in 2026-09, and nothing reads that fork any more. A change here is an ABI break for all of them — see
**Gotchas** at the bottom, which is not boilerplate now that they rebuild
separately.

## Messages

### Robot state aggregate

`RobotState` is published at 1 Hz as shipped (`publish_rate: 1.0` in
`syncai_robot_state/params/robot_state_params.yaml`; the code default is 10 Hz)
by `syncai_robot_state` on the relative topic
`robot_state` (BEST_EFFORT, KeepLast(1)) and consumed by `syncai_backend`, which
re-serialises **a subset** of it for `GET /api/v1/robot/state`. It nests seven of
the other messages:

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

`localization_valid` and `motor_status.timestamp` are **operator-facing and must
not reach the REST payload** (`low_level_mode` is exposed, deliberately). That
payload is a frozen third-party contract and the router names its fields one by
one; nothing else stops joint temperatures and motor error codes from leaking into
a public response. Adding a field here must not add one there.

An outward-facing variant on an absolute, fleet-wide `/robot_state` was built and
then reverted: a single DDS domain hosts several robots, so a shared root topic
interleaves them, and every per-robot consumer here is scoped to exactly one.

| Message | Fields | Notes |
|---|---|---|
| `RobotPose` | `x`, `y`, `z`, `yaw` | Plain floats, **not** a `geometry_msgs/Pose`. `yaw` is radians here; the REST layer converts to degrees. |
| `RobotLocalizationStatus` | `position`, `velocity` | |
| `RobotNetworkStatus` | `wifi_info` | Deliberately a JSON string, not a typed field — see below |
| `RobotBatteryStatus` | `battery_percentage` | 0–100, already scaled from `sensor_msgs/BatteryState.percentage` |
| `RobotMode` | `MAINTENANCE=0`, `MANUAL=1`, `AUTO=2` | **Constants only** — no data fields. Never published on its own; it exists so `RobotState.mode` has named values. |
| `RobotStatus` | `UNINITIALIZED=0`, `IDLE=1`, `RUNNING=2`, `WARNING=3`, `ERROR=4`, `CHARGING=5` | Same pattern, for `state`. `UNINITIALIZED` holds `0` on purpose, so a default-constructed message does not claim to be `IDLE`. |
| `RobotLowLevelMode` | `policy_state`, `motion_state`, `safety_state` | The gait controller's own state machine, from the `mode` topic. Both indices are the **controller's** vocabularies, not ours, and carry **no constants** for the same reason `SetPolicyMode.mode` does not — see the note there. It carries **no freshness field**, so `0 / 0` before the first sample is indistinguishable from a real `PPO / Stand`. `safety_state` is the exception on both counts: it is `syncai_driver_manager`'s own safety lock (from its latched `safety_locked` topic), not something the controller reports. |

`mode` is still a placeholder: `syncai_robot_state` hardcodes `AUTO` (`{TODO}` in
the source), and the REST layer surfaces it.

`state` carries three of the six values, evaluated most-severe-first by
`syncai_robot_state`:

- `UNINITIALIZED` — the `map → base_link` TF did not resolve, so
  `localization_status` is zeroed. It outranks `WARNING` because "we do not know
  where the robot is" is the more fundamental fact: a low battery is worth
  reporting, but not at the cost of hiding that the pose in the same message is a
  placeholder. `localization_valid` is the precise answer for consumers that only
  care about pose trust; `state` is the coarse rollup, and the two are derived
  from one lookup so they cannot disagree.
- `WARNING` — battery below 20%, latched with hysteresis (clears above 25%).
- `IDLE` — otherwise.

`RUNNING` and `ERROR` are never emitted yet. **`CHARGING` cannot be derived at
all**: `syncai_driver_manager` hardcodes `BatteryState.power_supply_status` to
`UNKNOWN`, and the only other candidate is the sign of `current`, whose convention
is undocumented. The REST layer does not expose `state`, so none of this reaches
the UI.

**Why `wifi_info` is a JSON string.** `syncai_robot_state` flattens the latest
`WifiStatus` into `{"ssid", "bssid", "rssi", "ip_address", "mac_address"}` and
dumps it into this one field. Before the first `wifi_status` arrives it is the
literal string `"null"`, which is why the backend parses it defensively and falls
back to an empty object. A typed sub-message would be cleaner; the string keeps
the aggregate stable while wifi reporting is still in flux.

### Wifi

| Message | Fields | Used by |
|---|---|---|
| `WifiNetwork` | `bssid`, `ssid`, `rssi` | A scan result. Returned in bulk by `ScanWifiNetworks`. |
| `WifiStatus` | `bssid`, `ssid`, `rssi`, `ip_address`, `mac_address` | Published at 1 Hz on `wifi_status` by `syncai_sys_manager`'s wifi manager; consumed only by `syncai_robot_state`. |

`WifiStatus` is `WifiNetwork` plus the two local-interface fields. `rssi` is
`int8` in both (dBm, so roughly −100…0).

### Driver telemetry

Published by `syncai_driver_manager` from the ASCII telemetry it receives over
its UDP link to the gait controller.

| Message | Topic | Fields |
|---|---|---|
| `IMUState` | `imu` (SensorDataQoS) | `timestamp`, `quaternion[4]`, `gyroscope[3]`, `accelerometer[3]`, `rpy[3]`, `temperature` |
| `MotorStates` | `motor_states` (SensorDataQoS) | `timestamp` (**nanoseconds here**) + `MotorState[] states`. Also nested as `RobotState.motor_status` — which is why that message has no flat `motor_timestamp` beside a bare array any more, and where `timestamp` is scaled to **seconds** |
| `MotorState` | — | `name`, `q`, `dq`, `ddq`, `tau_est`, `temperature`, `error` — generalized position / velocity / acceleration / estimated torque |

These are the robot's own formats rather than `sensor_msgs/Imu` and
`sensor_msgs/JointState` because they carry per-motor `temperature` and `error`
fields that the standard messages have nowhere to put, and because `IMUState`
mirrors the field layout the gait controller already sends.

### MappingStatus

`pgo_node`'s run state, published on the relative topic `mapping_status`
(`<robot_id>/pgo/mapping_status`) with **RELIABLE + TRANSIENT_LOCAL depth 1**
— the same latched QoS as its `map_cloud_file` notice, so a backend that
(re)connects mid-session learns the state at once. Sent on every transition
and at 1 Hz. Consumed by `syncai_backend` for `GET /api/v1/mapping`.

```
uint8 IDLE      = 0   # no graph, intake dropped, /dev/shm empty; the session starts here
uint8 MAPPING   = 1   # keyframes accumulate; reset_mapping discards and stays here
uint8 RESETTING = 2   # transient: a start/reset is waiting on the ResetLIO round trip
uint8 state
uint32 key_poses
uint32 loop_closures
builtin_interfaces/Time stamp
```

`start_mapping` is IDLE → MAPPING, a successful `save_maps` is MAPPING → IDLE,
`reset_mapping` is MAPPING → MAPPING. `IDLE = 0` for the same reason
`RobotStatus.UNINITIALIZED = 0`: a default-constructed message reads as the
safe state. Added 2026-10 with `StartMapping`, when pgo gained an idle state.

### ArtifactState

`ArtifactState` has **no publisher or subscriber in this workspace**. It is the
robot-side mirror of the interface used by the separate artifact stack
(`SyncAI-Artifact-Workspace`), kept here so a future ROS-side artifact monitor
can be written against it without a second definition.

```
uint64 timestamp     # ms since epoch
string artifact_id   # "conveyor_0", "door_1"
string type          # live_info discriminator: "conveyor", "door", ...
bool   connected     # transport session (e.g. modbus tcp) is up
bool   stale         # connected but the device stopped updating
uint16 error_code    # device-reported, 0 = ok
string live_info     # JSON of DECODED values, never raw registers
```

The split is the point: the four health fields are uniform across artifact types
so a monitor can alarm without parsing anything, while everything type-specific
lives in the `live_info` JSON discriminated by `type`. The backend's
`ConveyorPhase` enum (`belt`/`handoff`/`carried`/`dropped`) is what shows up in
a conveyor's `live_info.phase`.

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
| `ResetLIO` | *(empty)* | `success`, `message`, `float64 last_odom_time` | `syncai_pointlio` on `pointlio/reset` |
| `SaveMaps` | `file_path`, `save_patches` | `success`, `message` | `syncai_mapping` on `pgo/save_maps` |
| `ResetMapping` | `reset_lio` | `success`, `message`, `float64 lio_last_odom_time`, `uint32 dropped_key_poses` | `syncai_mapping` on `pgo/reset_mapping` |
| `StartMapping` | `reset_lio` | `success`, `message`, `float64 lio_last_odom_time` | `syncai_mapping` on `pgo/start_mapping` |
| `RefineMap` | `maps_path` | `success`, `message` | `syncai_mapping` (`hba_node`, offline, by hand) on `hba/refine_map` |
| `SavePoses` | `file_path` | `success`, `message` | `syncai_mapping` (`hba_node`) on `hba/save_poses` |
| `Relocalize` | `pcd_path`, `x`, `y`, `z`, `yaw`, `pitch`, `roll` (`float32`, radians) | `success`, `message` | `syncai_localizer` on `relocalize` (bare: `<robot_id>/relocalize`) |
| `IsValid` | `int32 code` | `bool valid` | `syncai_localizer` on `relocalize_check` |

Notes:

- **`SetMotionKey.key` is a string, not an enum**, because it is forwarded
  verbatim to the gait controller. Values in use: `0` stand, `1` locomotion,
  `2` lie down, `3` damping, `4` emergency stop, `5` MPC. Two callers in the
  backend: the `STANDUP` / `LIEDOWN` task steps
  (`syncai_backend/temporal/activities.py`, keys `0` / `2`) and
  `POST /api/v1/robot/set_motion_key`. REST exposure has been round-tripped —
  added in `daca318`, removed in `191b484` when the task steps took over, and
  re-added since for manual operator control alongside them. The endpoint accepts
  all six keys but **does not forward `"4"`**: it answers 200 with
  `sent: false` and produces no `ESTOP` datagram, because an emergency stop over
  unauthenticated HTTP on an unacknowledged one-way link is not a safety claim
  the REST layer can honour.
- **`SetSpeedScale` has six independent scales** (`fwd`, `back`, `left`, `right`,
  `turn_l`, `turn_r`) because the gait controller tracks commanded velocity
  asymmetrically per direction; the driver manager applies these as a correction
  to `cmd_vel`. It is also the only service here whose response has no `message`
  field.
- `SetPolicyMode.mode` is a bare `uint8` and does **not** reuse `RobotMode`'s
  constants — it is the gait controller's policy index, a different namespace
  that happens to share the type. Exposed as
  `POST /api/v1/robot/set_policy_mode`, which narrows the accepted values to `0`
  (PPO) and `1` (HIMLOCO) through a router-level enum. That constrains the **REST
  vocabulary only**: this field stays a bare `uint8` and
  `RobotGateway.set_policy_mode()` stays `int`, so a non-REST caller can still
  send `2` (CHAMP) or `3` (ISSAC). Those names come from the reference
  implementation's Readme, and the only place they are written down in this
  workspace is a comment in `msg/RobotLowLevelMode.msg` — which documents the same
  policy vocabulary for `RobotState.low_level_mode.policy_state` and explains why
  neither place declares constants for it.

- **`ResetLIO` and `ResetMapping` are the two halves of one contract.** The
  backend calls `reset_mapping` on `pgo_node` (`syncai_mapping`); `pgo_node`
  pauses intake, calls `reset` on `pointlio_node` (`syncai_pointlio`) over
  `ResetLIO`, rebuilds its graph and resumes, dropping every cloud/odom pair
  stamped at or before the `last_odom_time` the LIO returned. Both moved here
  from SyncAI-Fast-LIO2's `interface` package in 2026-09, with the nodes that
  serve them (`ResetLIO` first with pointlio, `ResetMapping` and `SaveMaps`
  with pgo). `ResetLIO`'s request is empty on purpose — a reset is not a
  reconfigure — and `ResetMapping.reset_lio: false` resets the graph alone, a
  bag-replay affordance the console never sends. The comments in the two
  `.srv` files are the design record; read them before changing any side.
  **The robot must be standing still** when a reset lands: the LIO re-runs a
  static, gravity-aligning IMU init, and nothing in either node enforces that.
- **`StartMapping` is the same sequence from IDLE.** Since 2026-10 pgo comes
  up idle in a mapping session and banks nothing until `start_mapping`, which
  runs `ResetMapping`'s pause → `ResetLIO` → fresh graph → resume with the
  precondition inverted (refused while MAPPING, as `reset_mapping` is refused
  while IDLE). A successful `SaveMaps` ends the run — pgo returns to IDLE,
  frees its keyframes and clears `/dev/shm` — so a run is bracketed
  `start_mapping … save_maps`. `MappingStatus` (above) is how a consumer
  tells the two states apart. The same stillness rule applies to a start.
- **`SaveMaps` writes a directory layout the map catalogue depends on**
  (`map.pcd`, `patches/<i>.pcd`, `poses.txt` with bare basenames and no
  absolute paths); the `.srv` documents it. It was served from
  SyncAI-Fast-LIO2 until 2026-09 and is now `syncai_mapping`'s.
- **`RefineMap` / `SavePoses` are the offline half of mapping** (`hba_node`,
  also `syncai_mapping`): load the `patches/` + `poses.txt` a `SaveMaps` with
  `save_patches: true` wrote, refine the poses, write them to a separate file.
  Nothing in a session or in the backend calls them; they are here because
  the node moved here and this package is where the stack's interfaces live.
- **`Relocalize` success is a receipt, not a result; `IsValid` is the
  result.** Registration runs async on the localizer's timer; `IsValid` with
  `code: 0` reports whether the first registration after the guess converged
  (`code: 1` always answers `valid: true`, a liveness probe). `Relocalize`
  takes the raw 6-DOF pose, so the backend follows it with an `initialpose`
  publish — the tilted lidar mount means a flat guess never converges. Both
  are served by `syncai_localizer` in the workspace, under the bare robot_id
  namespace — `<robot_id>/relocalize`, not `<robot_id>/localizer/relocalize`
  as these docs said until the localizer was ported from `SyncAI-Fast-LIO2`
  (whose `interface` package they came from).

The `success`/`message` pair is the convention for everything here: callers check
`success` and surface `message` verbatim (the backend maps a failed wifi connect
to HTTP 400 with that string as the detail).

## Action

`ExecuteTask` — goal `uuid` / `timestamp` / `behavior_tree` (an **inline BT XML
string**, not a file path), result `success` / `message` / `finished_timestamp`,
feedback `status` / `elapsed_ms`.

**Nothing implements or calls it.** It was defined for a design where the backend
would hand a whole behavior tree to the robot per task. The standing decision
went the other way: `RobotWorkflow` in `syncai_backend` sequences steps itself
and dispatches `MOVE` to `nav2_msgs/NavigateToPose` and `ARTIFACT` to the
artifact REST API, with the behavior-tree route reserved for a future need for
tick-level parallelism. The definition is kept because that need may still
arrive; its comments are in Chinese, unlike the rest of the package.

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

Python — the generated module is importable once the workspace is sourced:

```python
from syncai_common.msg import RobotState, RobotMode
from syncai_common.srv import ScanWifiNetworks, ConnectWifiNetwork, SetMotionKey
```

Constant-only messages are accessed as class attributes, never instantiated:

```python
if state.mode == RobotMode.AUTO: ...
```

## Build

```bash
colcon build --packages-select syncai_common
source install/setup.bash
```

Then rebuild every dependent package — generated headers and Python modules do
not update in place, and `--symlink-install` does not help here because these are
generated artifacts, not source files:

```bash
colcon build --packages-up-to syncai_robot_state syncai_driver_manager
```

Inspect what actually got generated:

```bash
ros2 interface show syncai_common/msg/RobotState
ros2 interface list | grep syncai_common
```

## Gotchas

- **Changing a field is an ABI break.** Every node that publishes or subscribes
  the message must be rebuilt *and restarted*; a mismatched pair fails at the
  type-hash level with no useful error. On a live robot, rebuild the whole
  workspace rather than one package.
- **Renumbering a constant is an ABI break with no type-hash warning.**
  `RobotStatus` gained `UNINITIALIZED = 0` and everything else shifted up by one.
  Constants are compiled into consumers, so the wire format is unchanged and
  nothing fails loudly — but `state` values in bags recorded before the change
  now decode to the wrong name. It was safe to do because no consumer read the
  numeric value; that will not be true forever.
- **Timestamp units are not uniform.** `ArtifactState` and `ExecuteTask` are in
  milliseconds; `RobotState.timestamp` is in **seconds** (`now().seconds()` cast
  to `uint64`), because it is passed through verbatim to
  `GET /api/v1/robot/state` and the frontend already multiplies by 1000. Both of
  `RobotState`'s timestamps are seconds — but `motor_status.timestamp` only because
  `syncai_robot_state` scales it on the way in, while the **same `MotorStates`
  message on the `motor_states` topic carries nanoseconds** (the topic keeps them
  because the backend's telemetry WebSocket needs sub-second ordering at 20 Hz).
  So a `MotorStates`' unit depends on where you found it. Each `.msg` states its
  unit — check it before doing arithmetic across two of them.
- **`RobotState.timestamp` cannot order samples.** Whole seconds at a 10 Hz
  publish rate means ten consecutive messages carry the same value. It is a wall
  clock for display, and `motor_status.timestamp` is no better now that it is
  seconds too — subscribe `motor_states` directly for sub-second resolution.
- **No message carries a `std_msgs/Header`.** Timestamps are bare `uint64`
  fields — except `MappingStatus.stamp`, a `builtin_interfaces/Time`, the one
  field here of another package's type — and there is no `frame_id` anywhere:
  these are status messages, not sensor data to be transformed. Anything needing TF uses a `geometry_msgs` type
  instead. `RobotLowLevelMode` is the extreme case: its upstream
  `std_msgs/Int32MultiArray` has no header either and the telemetry link carries no
  clock, so there is no timestamp available anywhere on that path — which is why
  that message can say what the controller reports but never when.
- **Constant-only messages (`RobotMode`, `RobotStatus`) generate a publishable
  type with zero fields.** Publishing one is legal and meaningless; they exist
  purely as a constant namespace for `RobotState`'s `uint8` fields.
- `msg/`, `srv/` and `action/` each still carry a `.gitkeep` from when they were
  empty. Harmless, and every new interface must also be listed in
  `CMakeLists.txt` — the directories are not globbed.
