# cmeresearch_robot_state

ROS 2 Jazzy package that produces the high-level state and host telemetry the
webapp / MQTT layer expects from a CMEXAIII robot. It bundles three small,
independent executables that together populate three ROS topics:

| Topic | Type | Producer |
|---|---|---|
| `/robot_state` | `cmeresearch_msgs/msg/RobotState` | `sm_robot_node` |
| `/nav_status` | `std_msgs/msg/String` | `nav_status_node` |
| `/system_stats` | `cmeresearch_msgs/msg/SystemStats` | `system_stats_node` |

All three are bridged to MQTT by `cmeresearch_bringup`'s `mqtt_bridge` config
(`bridge5`, `bridge6`, `bridge12`) so the webapp can render the robot's state
machine, the active Nav2 goal, and the controller host's CPU / memory / disk /
temperature / Raspberry Pi throttle state in real time.

The same three executables run unchanged in the Webots simulation
(`cmeresearch_simulation`), which is what makes the webapp behave identically
against sim and real hardware.

## Why the split into three nodes

Each node corresponds to a distinct concern and a distinct lifecycle:

- **`sm_robot_node`** owns the robot-level finite state machine. It needs to
  react to stepper-driver lifecycle events and to high-level commands from the
  operator. It is the only node here that uses SMACC2.
- **`nav_status_node`** is a thin, stateless adapter over the Nav2 action
  status array. It exists so subscribers (webapp, dashboards) don't have to
  understand `action_msgs/GoalStatusArray`.
- **`system_stats_node`** reads `/proc`, `/sys`, `/etc/os-release`,
  `/proc/device-tree/model`, `statvfs(2)`, `uname(2)`, and `vcgencmd` to
  produce a typed host snapshot. It has no opinion on robot logic; killing it
  must not affect locomotion or navigation.

Keeping them as separate processes also means a crash in one (e.g. a malformed
`vcgencmd` output during a kernel upgrade) cannot bring down the SMACC FSM.

---

## `sm_robot_node` — robot state machine

Implemented as a [SMACC2](https://github.com/robosoft-ai/SMACC2) state machine
in `include/cmeresearch_robot_state/sm_robot.hpp`; the entry point in
`src/sm_robot_node.cpp` just calls `smacc2::run<SmRobot>()`.

### States and transitions

```
                EvStateFinished                EvMissionStart
   Initializing ───────────────►   Idle    ────────────────►   Moving
        ▲                          │  ▲                          │
        │                          │  │   EvStateFinished         │
        │                          │  └──────────────────────────┘
        │              EvEmergencyStop │              EvEmergencyStop
        │                              ▼                          ▼
        │                            EmergencyStop ◄──────────────┘
        │                                  │
        └──────────────────────────────────┘
                       EvReset
```

| State | Entry side-effect | Exit trigger(s) |
|---|---|---|
| `Initializing` | publishes `state="initializing"`; waits for every configured driver topic to publish `"initialized"` or `"idle"`, or for `init_timeout_sec` to elapse | `EvStateFinished` → `Idle` |
| `Idle` | publishes `state="idle"` | `EvMissionStart` → `Moving`, `EvEmergencyStop` → `EmergencyStop` |
| `Moving` | publishes `state="moving"` | `EvStateFinished` → `Idle`, `EvEmergencyStop` → `EmergencyStop` |
| `EmergencyStop` | publishes `state="emergency_stop"` (also logs `RCLCPP_ERROR`) | `EvReset` → `Initializing` |

### Events

| Event | Source | Trigger |
|---|---|---|
| `EvStateFinished` | internal | All configured drivers reported ready, OR the init-timeout fired |
| `EvMissionStart` | `/robot_cmd` `String` | `data == "start_mission"` |
| `EvEmergencyStop` | `/robot_cmd` `String` OR a driver topic | `data == "emergency_stop"`, OR any driver topic publishes `"low_voltage"` (rising-edge only) |
| `EvReset` | `/robot_cmd` `String` | `data == "reset"` |

### Driver readiness gating

On startup, `sm_robot_node` subscribes to each topic listed in the
`driver_topics` parameter (default: the four `/cmexa_base/<wheel>/state`
topics). It remembers the last value seen on each. The state machine only
leaves `Initializing` once **every** subscribed topic has reported `initialized`
or `idle`.

Two escape hatches keep this robust:

1. **`driver_topics: []`** disables the gate entirely — the SM transitions to
   `Idle` immediately after `onInitialize()`. Used by the SMACC launch_test
   fixture and by any environment that has no stepper-driver mocks.
2. **`init_timeout_sec`** (default 30 s) forces an `EvStateFinished` even if
   one or more drivers never reported. The driver names + last-seen values are
   logged at WARN so the failure mode is visible.

A driver topic publishing `"error"` is treated as a hard stop: the SM stays
in `Initializing` and republishes `state="driver_error"` on `/robot_state`
until the topic clears.

A driver topic publishing `"low_voltage"` (on a rising edge — repeats are
ignored) triggers `EvEmergencyStop` from any state. The corresponding
`state="emergency_stop"` is published *before* the SMACC transition runs so
the webapp / MQTT layer sees the e-stop even if SMACC takes a tick to react.

### Published message — `cmeresearch_msgs/msg/RobotState`

```
std_msgs/Header header
string         state            # initializing | idle | moving | emergency_stop | driver_error
string[]       driver_names     # wheel identifiers, e.g. "front_left"
string[]       driver_states    # last-seen value per driver topic
```

`driver_names` is computed from each driver topic by
`wheel_name_from_topic()` (defined in `sm_robot.hpp`), which extracts the
second-to-last `/`-separated segment. So `/cmexa_base/front_left/state`
becomes `front_left` and `/cmexaiii/front_left/state` also becomes
`front_left` — the webapp gets stable wheel labels regardless of the topic
namespace used on a given robot.

`/robot_state` is published with `transient_local` durability so a late
subscriber (e.g. the webapp re-connecting after a network blip) immediately
sees the most recent state instead of waiting for the next transition.

### Parameters

| Name | Type | Default | Purpose |
|---|---|---|---|
| `driver_topics` | `string[]` | 4 × `/cmexa_base/<wheel>/state` | Driver state topics the SM listens to for readiness gating |
| `init_timeout_sec` | `double` | 30.0 | Hard cap on the Initializing wait. `0` disables. |

### Subscribed topics

| Topic | Type | Purpose |
|---|---|---|
| `/robot_cmd` | `std_msgs/String` | High-level operator commands: `start_mission`, `emergency_stop`, `reset` |
| `<driver_topics[i]>` | `std_msgs/String` (transient_local QoS) | Per-driver lifecycle: `initialized`, `idle`, `error`, `low_voltage`, … |

---

## `nav_status_node` — Nav2 action-status adapter

`src/nav_status_node.cpp` is a single-file `rclcpp::Node` that converts the
Nav2 `navigate_to_pose` action status into a human-readable string so the
webapp / MQTT consumers don't need to depend on `action_msgs`.

### Mapping

| `GoalStatus` field | Published `String` |
|---|---|
| `STATUS_ACCEPTED`, `STATUS_EXECUTING` | `navigating` |
| `STATUS_CANCELING` | `canceling` |
| `STATUS_CANCELED` | `canceled` |
| `STATUS_ABORTED` | `aborted` |
| `STATUS_SUCCEEDED` | `succeeded` |
| (no goals in array) | `idle` |

The status array can contain multiple goals (Nav2 keeps historical entries).
The first goal whose mapped status is anything other than `idle` wins. A
de-duplication check (`last_status_`) suppresses redundant publishes; only
real transitions reach `/nav_status`.

`idle` is published once at startup so a fresh webapp connection sees the
robot as "not navigating" before Nav2 emits its first status array.

### Topics

| Direction | Topic | Type | Notes |
|---|---|---|---|
| sub | `/navigate_to_pose/_action/status` | `action_msgs/GoalStatusArray` | depth 10, default reliability |
| pub | `/nav_status` | `std_msgs/String` | `transient_local`, depth 1 |

No parameters.

---

## `system_stats_node` — host telemetry

`src/system_stats_node.cpp` runs a periodic timer (default every 5 s) and
publishes a `cmeresearch_msgs/msg/SystemStats` snapshot of the controller
host. It deliberately does no shell-outs except `vcgencmd` (and only on
Raspberry Pi targets); everything else is `/proc` or syscalls so the cost
per sample is negligible.

### What it reads

| Field | Source | Notes |
|---|---|---|
| `cpu_percent` | `/proc/stat` first line (jiffies) | `100 × (1 − Δidle / Δtotal)` between consecutive samples; idle includes iowait so transient I/O doesn't read as CPU-busy. `NaN` if `/proc/stat` is unavailable. Counters are seeded at construction so the first publish reflects an actual interval, not jiffies-since-boot. |
| `memory_*` | `/proc/meminfo` | `memory_used_mb` = `MemTotal − MemAvailable`; `memory_percent` uses `MemAvailable` (kernel's "what apps can actually have" estimate) rather than `MemFree`. |
| `temperature_c` | `vcgencmd measure_temp` or `/sys/class/thermal/thermal_zone0/temp` | Switchable via `temperature_source` parameter (`auto`, `vcgencmd`, `sysfs`). `auto` tries vcgencmd first, falls back to sysfs. |
| `throttle_bits` / `throttle_active` | `vcgencmd get_throttled` | Raw 32-bit mask. `throttle_active` is true iff any of `0x1` (under-voltage), `0x2` (ARM freq cap), `0x4` (currently throttled), `0x8` (soft temp limit) is set. Sticky "has-occurred-since-boot" bits (`0x10000`–`0x80000`) are passed through unchanged for dashboards that want to flag past events. `0` on non-Pi hosts. |
| `disk_*` | `statvfs(disk_mount_point)` | Uses `f_bavail` (blocks available to non-root) so numbers match `df -h`, not `f_bfree`. |
| `os_pretty_name` | `/etc/os-release` `PRETTY_NAME` | Read **once at startup** and re-published every sample. |
| `kernel_release` | `uname(2)` `release` field | Read once at startup. |
| `hardware_model` | `/proc/device-tree/model` | Read once at startup. Trailing NUL/whitespace stripped (device-tree exports are typically null-terminated). Empty on non-Pi hosts. |

The three identification fields are cached once because they don't change at
runtime, but they are written into every sample so the MQTT `retain` flag
keeps them visible to subscribers that connect after node startup, without
needing a separate "hello" topic.

`vcgencmd` is invoked via `popen` with stderr redirected to `/dev/null`.
A missing binary or a non-zero exit produces `std::nullopt`, which the node
treats as "field unavailable" — temperature becomes `NaN`, throttle stays
`0`. The node never errors out on non-Pi hardware.

When the active throttle bits are non-zero the node logs an
`RCLCPP_WARN_THROTTLE` (60 s) with a decoded line showing which specific bit
fired. This is the recommended signal for "robot is under-voltage right now"
alerts.

### Parameters

| Name | Type | Default | Purpose |
|---|---|---|---|
| `publish_rate_hz` | `double` | 0.2 | Sample / publish rate. 0.2 Hz = every 5 s. |
| `temperature_source` | `string` | `auto` | `auto` (vcgencmd then sysfs), `vcgencmd`, or `sysfs`. |
| `disk_mount_point` | `string` | `/` | Mount point passed to `statvfs(2)`. |

### Topics

| Direction | Topic | Type | Notes |
|---|---|---|---|
| pub | `/system_stats` | `cmeresearch_msgs/SystemStats` | `transient_local`, depth 10 |

---

## How the package is used

### Hardware (cmexaiii-001)

`cmeresearch_bringup/launch/cmexaiii_hardware.launch.py` starts all three
nodes. The only non-default parameter on the real robot is `driver_topics`
on `sm_robot_node`, overridden to the cmexaiii namespace:

```python
parameters=[{
    'driver_topics': [
        '/cmexaiii/front_left/state',
        '/cmexaiii/front_right/state',
        '/cmexaiii/rear_left/state',
        '/cmexaiii/rear_right/state',
    ],
    'init_timeout_sec': 30.0,
}]
```

The default `/cmexa_base/<wheel>/state` topics are not produced on the
cmexaiii hardware because the tinkerforge stepper drivers there remap their
state output to `/cmexaiii/<wheel>/state`. Without this override `sm_robot`
would wait out the full `init_timeout_sec` on every boot.

`cmeresearch_robot_state` itself is pulled into the cmexa-hardware Docker
image via `cmexa_install/docker/hardware.repos`, pinned by SHA and
auto-bumped by the hourly `bump-sub-repos` workflow.

### Simulation (Webots)

`cmeresearch_simulation/launch/cmexaiii_webots_full.launch.py` and
`cmexaiii_nav_stack.launch.py` start the same three nodes. The Webots world
has no tinkerforge bricklets, so `sm_robot_node` runs with
`driver_topics: []` — readiness gating is skipped, the SM transitions to
`Idle` immediately, and the rest of the stack works as on hardware.

### MQTT bridge

`cmeresearch_bringup/config/cmexaiii/mqtt_bridge_params.yaml` configures the
following relays (the sim mirrors them in
`cmeresearch_simulation/config/cmexaiii/mqtt_bridge_sim_params.yaml`):

| Bridge | ROS topic | MQTT topic | Direction |
|---|---|---|---|
| `bridge5` | `/robot_state` | `cmeresearch/cmexaiii-001/robot_state` | ROS → MQTT |
| `bridge6` | `/nav_status` | `cmeresearch/cmexaiii-001/navigation/status` | ROS → MQTT |
| `bridge12` | `/system_stats` | `cmeresearch/cmexaiii-001/system/stats` | ROS → MQTT |

Messages are JSON-serialised by `mqtt_bridge`'s default `json:dumps` /
`json:loads` serializer — typed ROS messages become flat JSON objects with the
field names of the `.msg` definition.

### Webapp

`cmeresearch_amr_webcontrol` (template `templates/amr_control/viewport.html`)
subscribes to all three MQTT topics and renders:

- Robot-state badge with per-driver pills (one per entry in
  `driver_names` / `driver_states`)
- Navigation-status badge
- Host card with model + OS + kernel + CPU bar + Memory bar + Disk bar +
  Temperature gauge + Throttle badge (lit when `throttle_active` is `true`)

---

## Build and test

The package targets ROS 2 Jazzy and builds with `colcon`.

```bash
# From a ROS 2 Jazzy workspace that has cmeresearch_msgs and smacc2 available:
colcon build --packages-select cmeresearch_robot_state

# Run the SMACC state-machine launch test:
colcon test --packages-select cmeresearch_robot_state \
            --event-handlers console_direct+
colcon test-result --verbose
```

### Launch test

`test/test_state_transitions.py` is a `launch_testing_ament_cmake` fixture
that boots `sm_robot_node` with `driver_topics: []` (so it short-circuits to
`Idle`) and exercises every transition:

1. `test_01_startup_reaches_idle` — `state` reaches `idle` after spawn.
2. `test_02_start_mission_idle_to_moving` — `/robot_cmd "start_mission"` →
   `state == moving`.
3. `test_03_emergency_stop_from_moving` — `/robot_cmd "emergency_stop"` →
   `state == emergency_stop`.
4. `test_04_reset_from_emergency_stop` — `/robot_cmd "reset"` → back to
   `idle` (via a fresh `Initializing` pass; the fixture's empty
   `driver_topics` makes that pass instant).

### CI

`.github/workflows/build.yml` runs the build + launch test on every push to
`jazzy` / `jazzy_dev` and every PR targeting `jazzy`, using
`ros-tooling/action-ros-ci` against the `ros:jazzy-ros-core` container.
`.ci.repos` pulls `cmeresearch_msgs` from the same `jazzy_dev` branch so
the message definitions resolve.

---

## Standalone launch file

`launch/sm_robot.launch.py` starts the three nodes with package defaults —
useful for poking at the SM in isolation on a workstation:

```bash
ros2 launch cmeresearch_robot_state sm_robot.launch.py
```

For production use, prefer the corresponding `cmeresearch_bringup` /
`cmeresearch_simulation` launch files, which set the correct
`driver_topics` for the target.

---

## Files at a glance

```
include/cmeresearch_robot_state/
  sm_robot.hpp                 # SMACC2 state machine (states, events, SmRobot)
src/
  sm_robot_node.cpp            # main() — smacc2::run<SmRobot>()
  nav_status_node.cpp          # Nav2 action-status adapter
  system_stats_node.cpp        # /proc + vcgencmd + statvfs → SystemStats
launch/
  sm_robot.launch.py           # Standalone launch of all three nodes
test/
  test_state_transitions.py    # launch_testing fixture exercising the SM
CMakeLists.txt                 # 3 ament executables, install + launch + tests
package.xml                    # ament_cmake, smacc2, cmeresearch_msgs, …
.ci.repos                      # pulls cmeresearch_msgs for CI
.github/workflows/build.yml    # build + launch test on push / PR
```
