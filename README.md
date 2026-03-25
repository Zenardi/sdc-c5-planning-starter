# Project: Motion Planning and Decision Making for Autonomous Vehicles

- [Project: Motion Planning and Decision Making for Autonomous Vehicles](#project-motion-planning-and-decision-making-for-autonomous-vehicles)
  - [Architecture](#architecture)
    - [FSM State Machine](#fsm-state-machine)
  - [Prerequisites](#prerequisites)
  - [Setup](#setup)
    - [Step 1 — Clone the repository](#step-1--clone-the-repository)
    - [Step 2 — Install C++ build dependencies](#step-2--install-c-build-dependencies)
    - [Step 3 — Install WebSocket library (uWebSockets)](#step-3--install-websocket-library-uwebsockets)
    - [Step 4 — Install Python dependencies](#step-4--install-python-dependencies)
    - [Step 5 — Build the C++ planner](#step-5--build-the-c-planner)
  - [Running](#running)
    - [Terminal 1 — Start CARLA](#terminal-1--start-carla)
    - [Terminal 2 — Start the planner](#terminal-2--start-the-planner)
  - [Expected Behavior](#expected-behavior)
  - [Implementation Details](#implementation-details)
    - [`planning_params.h`](#planning_paramsh)
    - [`behavior_planner_FSM.cpp` — 10 TODOs](#behavior_planner_fsmcpp--10-todos)
    - [`motion_planner.cpp` — 2 TODOs](#motion_plannercpp--2-todos)
    - [`cost_functions.cpp` — 3 TODOs](#cost_functionscpp--3-todos)
    - [`velocity_profile_generator.cpp` — 2 TODOs](#velocity_profile_generatorcpp--2-todos)
  - [Tested Environment](#tested-environment)
    - [Python packages](#python-packages)
    - [System packages (apt)](#system-packages-apt)
  - [Pre-Implementation Fixes](#pre-implementation-fixes)
    - [Fix 1 — CARLA simulator path](#fix-1--carla-simulator-path)
    - [Fix 2 — CARLA C++ SDK not included in binary release](#fix-2--carla-c-sdk-not-included-in-binary-release)
    - [Fix 3 — C++ standard version](#fix-3--c-standard-version)
    - [Fix 4 — Missing Python dependencies](#fix-4--missing-python-dependencies)
    - [Fix 5 — Python 3.11 asyncio compatibility](#fix-5--python-311-asyncio-compatibility)
    - [Fix 6 — `is` with string literals (SyntaxWarning → broken logic)](#fix-6--is-with-string-literals-syntaxwarning--broken-logic)
    - [Fix 7 — Low simulation FPS (GPU performance)](#fix-7--low-simulation-fps-gpu-performance)
  - [Troubleshooting](#troubleshooting)
  - [Project Structure](#project-structure)


This project implements two of the main components of a traditional hierarchical planner: the **Behavior Planner** (FSM) and the **Motion Planner** (cubic spiral paths). Both work in unison to:

1. Avoid static objects (cars, bicycles, and trucks) parked on the side of the road by executing either a "nudge" or a "lane change" maneuver.
2. Handle any type of intersection (3-way, 4-way, and roundabouts) by stopping for 5 seconds before proceeding.
3. Track the centerline on the traveling lane.

The planner is implemented in C++ and communicates with the [CARLA](https://carla.org/) simulator via a WebSocket connection bridged by a Python API.

---

## Architecture

```
CARLA Simulator
      ↕  (Python, simulatorAPI.py — WebSocket bridge)
   main.cpp  ←→  WebSocket :4567
      │
      ├── BehaviorPlannerFSM        (behavior_planner_FSM.cpp)
      │     └── state_transition()  →  goal State + active Maneuver
      │
      └── MotionPlanner             (motion_planner.cpp)
            ├── generate_offset_goals()  →  N lateral goal candidates
            ├── CubicSpiral              →  smooth path per goal
            ├── VelocityProfileGenerator →  velocity profile per path
            └── cost_functions           →  pick best trajectory
```

### FSM State Machine

```
FOLLOW_LANE  ──[goal in junction]──►  DECEL_TO_STOP
                                            │
                              [dist to stop line ≤ threshold]
                                            ▼
                                         STOPPED
                                            │
                              [timer ≥ 5s && traffic light ≠ Red]
                                            │
                                            └──────────────► FOLLOW_LANE
```

---

## Prerequisites

- **OS**: Ubuntu 20.04 or 22.04 (or WSL2)
- **CARLA**: The simulator binary must be located at `project/CARLA/` (i.e., `project/CARLA/CarlaUE4.sh` must exist).
- **CARLA C++ SDK** (`libcarla_client.a` and headers): Place a `libcarla-install/` directory inside `project/starter_files/` with the following structure:
  ```
  starter_files/libcarla-install/
  ├── include/   (CARLA C++ headers)
  └── lib/       (libcarla_client.a, librpc.a, and Boost/Recast static libs)
  ```
  The SDK can be obtained from the CARLA GitHub releases (look for `CARLA_0.9.x_SDK.tar.gz`), or reused from another CARLA-based project on the same machine.
- **Python 3.8+** (tested with Python 3.11)
- **Build tools**: `cmake`, `make`, `gcc`/`g++` with C++17 support
- **GPU**: An NVIDIA GPU is recommended. The `run_carla.sh` script is configured for NVIDIA PRIME offload (Optimus laptops). Adjust if running on a desktop with a single GPU.

---

## Setup

### Step 1 — Clone the repository

```bash
git clone https://github.com/<your-fork>/nd013-c5-planning-starter.git
cd nd013-c5-planning-starter
```

### Step 2 — Install C++ build dependencies

```bash
sudo apt-get install -y libgoogle-glog-dev libgtest-dev cmake make
```

### Step 3 — Install WebSocket library (uWebSockets)

```bash
cd project
./install-ubuntu.sh
cd ..
```

This installs `libuv1-dev`, `libssl-dev`, `libz-dev`, and builds/installs `uWebSockets`.

### Step 4 — Install Python dependencies

```bash
pip install numpy pygame websocket-client

# Install the CARLA Python package (adjust path/version as needed)
pip install project/CARLA/PythonAPI/carla/dist/carla-*.whl
```

### Step 5 — Build the C++ planner

```bash
cd project/starter_files
cmake .
make
cd ../..
```

The build produces `project/starter_files/spiral_planner`.

> **Note**: If `cmake .` fails with a missing `libcarla_client.a`, ensure `project/starter_files/libcarla-install/lib/` exists with the correct structure (see Prerequisites).

---

## Running

The project requires **two terminal windows**.

### Terminal 1 — Start CARLA

```bash
cd project
./run_carla.sh
```

CARLA runs headless (`-RenderOffScreen`). Wait until startup output settles (typically 10–20 seconds) before proceeding.

> To stop a running CARLA process: `ps -aux | grep CarlaUE4` then terminate it by PID.

### Terminal 2 — Start the planner

```bash
cd project
./run_main.sh
```

This launches `spiral_planner` (C++ WebSocket server on port 4567), waits 1 second, then starts `simulatorAPI.py` which connects to CARLA and opens the pygame visualization window.

> If you see `bind failed. Error: Address already in use`, find the old `spiral_planner` process with `ps -aux | grep spiral_planner` and terminate it by PID.

---

## Expected Behavior

Once running, the pygame window shows the ego vehicle (white dot) and surrounding actors. You should observe:

| Behavior | What to look for |
|---|---|
| **Lane following** | Vehicle smoothly tracks the lane centerline |
| **Obstacle avoidance** | Vehicle nudges left/right around parked cars |
| **Intersection handling** | Vehicle decelerates and stops fully at junctions |
| **Stop-and-go** | Vehicle waits ~5 seconds, then resumes when light is not red |

---

## Implementation Details

All implementation was done in the files listed below. Each TODO in the starter code is fully resolved.

### `planning_params.h`

| Parameter | Value | Description |
|---|---|---|
| `P_NUM_PATHS` | `5` | Number of lateral offset spiral paths generated per planning cycle |
| `P_NUM_POINTS_IN_SPIRAL` | `15` | Discretization points per spiral (balances collision precision vs. compute) |
| `P_SPEED_LIMIT` | `3.0 m/s` | Maximum desired speed in FOLLOW_LANE state |
| `P_LOOKAHEAD_MIN/MAX` | `8 / 20 m` | Lookahead distance clamp range |
| `P_MAX_ACCEL` | `1.5 m/s²` | Maximum comfortable acceleration/deceleration |
| `P_REQ_STOPPED_TIME` | `5.0 s` | Duration to hold STOPPED before releasing |
| `P_STOP_LINE_BUFFER` | `0.5 m` | Buffer behind the stop line for the stop goal |
| `CIRCLE_OFFSETS` | `{-1, 1, 3} m` | Longitudinal positions of collision-check circles along the vehicle body |
| `CIRCLE_RADII` | `{1.5, 1.5, 1.5} m` | Radii of those collision circles |

---

### `behavior_planner_FSM.cpp` — 10 TODOs

**Lookahead distance** (`get_look_ahead_distance`):
Computed as `v² / (2a)` — the distance needed to stop from the current speed — then clamped to `[P_LOOKAHEAD_MIN, P_LOOKAHEAD_MAX]`.

```cpp
auto look_ahead_distance = (velocity_mag * velocity_mag) / (2.0 * _max_accel);
look_ahead_distance = std::min(std::max(look_ahead_distance, _lookahead_distance_min),
                               _lookahead_distance_max);
```

**Stop-line goal offset** (DECEL_TO_STOP entry):
The goal is placed `P_STOP_LINE_BUFFER` behind the stop line by offsetting in the `yaw + π` direction.

```cpp
auto ang = goal.rotation.yaw + M_PI;
goal.location.x += _stop_line_buffer * std::cos(ang);
goal.location.y += _stop_line_buffer * std::sin(ang);
```

**Goal velocity vectors**:
- At stop: `{0, 0, 0}`
- In nominal (FOLLOW_LANE): `{speed_limit * cos(yaw), speed_limit * sin(yaw), 0}`

**FSM transitions**:
- `FOLLOW_LANE → DECEL_TO_STOP`: triggered when the lookahead goal is inside a junction.
- `DECEL_TO_STOP → STOPPED`: triggered when Euclidean distance to the stop goal is ≤ `P_STOP_THRESHOLD_DISTANCE`. Distance is used instead of speed to avoid false triggers on teleport (where speed is always 0).
- `STOPPED → FOLLOW_LANE`: triggered when `stopped_secs >= P_REQ_STOPPED_TIME` and traffic light is not Red.

**Goal caching**: `_goal` is saved at the end of every cycle. DECEL_TO_STOP and STOPPED both return `_goal` to hold the stop-line position across frames.

---

### `motion_planner.cpp` — 2 TODOs

**Perpendicular direction**: The offset direction is computed as `yaw + π/2` to place goals laterally across the lane.

```cpp
auto yaw = goal_state.rotation.yaw + M_PI / 2.0;
```

**Lateral goal positions**: Each of the `P_NUM_PATHS` goals is offset from the main goal along the perpendicular direction. Offset values are symmetric around 0 (e.g., with 5 paths: −2, −1, 0, +1, +2 × `P_GOAL_OFFSET`).

```cpp
goal_offset.location.x += offset * std::cos(yaw);
goal_offset.location.y += offset * std::sin(yaw);
```

---

### `cost_functions.cpp` — 3 TODOs

**Circle placement**: Three circles are placed along the vehicle body at longitudinal offsets defined by `CIRCLE_OFFSETS`, using the current waypoint's heading.

```cpp
auto circle_center_x = cur_x + CIRCLE_OFFSETS[c] * std::cos(cur_yaw);
auto circle_center_y = cur_y + CIRCLE_OFFSETS[c] * std::sin(cur_yaw);
```

**Collision detection**: Euclidean distance between each ego circle center and each obstacle circle center. A collision is flagged when `dist < CIRCLE_RADII[c] + CIRCLE_RADII[c2]`.

```cpp
double dist = std::sqrt(dx * dx + dy * dy);
collision = (dist < (CIRCLE_RADII[c] + CIRCLE_RADII[c2]));
```

**Proximity-to-goal cost**: 3D Euclidean distance from the last point on the spiral to the main (center-lane) goal. Spirals ending closer to the lane center score lower cost and are preferred.

```cpp
auto delta_x = main_goal.location.x - spiral[n - 1].x;
// ... (similarly for y and z)
```

---

### `velocity_profile_generator.cpp` — 2 TODOs

**`calc_distance`** — distance to accelerate/decelerate from `v_i` to `v_f` at acceleration `a`:

```
d = (v_f² − v_i²) / (2a)
```

**`calc_final_speed`** — final speed after traveling distance `d` from initial speed `v_i` at acceleration `a`:

```
v_f = sqrt(v_i² + 2ad)
```

Both handle edge cases: division by zero (returns infinity), negative discriminant (returns 0), and NaN/infinity discriminant.

---

## Tested Environment

This implementation was developed and verified on the following local setup. The original Udacity project targets Ubuntu 20.04 + CARLA 0.9.9 + Python 3.8; the fixes described below were required to run on a more recent environment.

| Component | Version |
|---|---|
| **OS** | Ubuntu 25.10 (kernel 6.17.0-19-generic) |
| **GPU** | NVIDIA GeForce RTX 5070 Laptop GPU (8 GB VRAM) |
| **NVIDIA Driver** | 580.126.09 |
| **CARLA Simulator** | 0.9.16 (binary at `project/CARLA/`) |
| **Python** | 3.11.15 (via pyenv) |
| **GCC** | 15.2.0 |
| **CMake** | 3.31.6 |
| **Make** | 4.4.1 |

### Python packages

| Package | Version |
|---|---|
| `carla` | 0.9.16 (installed from `project/CARLA/PythonAPI/carla/dist/carla-0.9.16-cp311-*.whl`) |
| `numpy` | 2.4.3 |
| `pygame` | 2.6.1 |
| `websocket-client` | 1.9.0 |

### System packages (apt)

```bash
sudo apt-get install -y libgoogle-glog-dev libgtest-dev libuv1-dev libssl-dev libz-dev
```

| Package | Version installed |
|---|---|
| `libgoogle-glog-dev` | 0.6.0 |
| `libgtest-dev` | 1.17.0 |
| `libuv1-dev` | 1.50.0 |
| `libssl-dev` | 3.5.3 |

---

## Pre-Implementation Fixes

The starter code was written for CARLA 0.9.9 and Python 3.8 running on a Udacity VM. Several fixes were needed before the planning TODOs could be implemented.

### Fix 1 — CARLA simulator path

**Problem**: `run_carla.sh` pointed to `/opt/carla-simulator/CarlaUE4.sh`, which does not exist on a local install.

**Fix**: Updated `run_carla.sh` to use the local CARLA binary and added NVIDIA PRIME offload flags for an Optimus (hybrid GPU) laptop:

```bash
# Before
/opt/carla-simulator/CarlaUE4.sh

# After
__NV_PRIME_RENDER_OFFLOAD=1 \
__NV_PRIME_RENDER_OFFLOAD_PROVIDER=NVIDIA-G0 \
VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json \
  ./CARLA/CarlaUE4.sh -RenderOffScreen -preferNvidia -quality-level=Low &
```

---

### Fix 2 — CARLA C++ SDK not included in binary release

**Problem**: `CMakeLists.txt` expected the CARLA C++ SDK (`libcarla_client.a` and headers) at `/opt/carla-source/Build/`, which only exists if you compile CARLA from source. The binary release does not include it.

**Fix**: Reused the pre-built `libcarla-install/` directory from another CARLA project on the same machine (symlinked into `project/starter_files/libcarla-install/`). Rewrote `CMakeLists.txt` to use a local flat `libcarla-install/` structure instead of `/opt/carla-source`.

---

### Fix 3 — C++ standard version

**Problem**: The system `libgtest-dev` (1.17.0) requires C++17. The original `CMakeLists.txt` set `-std=c++14`, causing hard compile errors.

**Fix**: Changed the C++ standard in `CMakeLists.txt` from `c++14` to `c++17`.

---

### Fix 4 — Missing Python dependencies

**Problem**: `simulatorAPI.py` imports `websocket`, `pygame`, and `numpy`, none of which were installed.

**Fix**:
```bash
pip install websocket-client pygame numpy
pip install project/CARLA/PythonAPI/carla/dist/carla-0.9.16-cp311-cp311-manylinux_2_31_x86_64.whl
```

---

### Fix 5 — Python 3.11 asyncio compatibility

**Problem**: Three breaking changes in Python 3.11 broke `simulatorAPI.py`:

1. `asyncio.coroutine` decorator was removed in Python 3.11.
2. `yield from` syntax inside coroutines must be replaced with `await`.
3. `asyncio.wait()` no longer accepts bare coroutines (must use `asyncio.gather()`).
4. `asyncio.get_event_loop()` is deprecated when no running loop exists.

**Fix**: Updated `simulatorAPI.py`:

```python
# Before — Python 3.8 style
@asyncio.coroutine
def game_loop(args):
    yield from asyncio.sleep(0.1)

asyncio.get_event_loop().run_until_complete(asyncio.wait([game_loop(args), server()]))

# After — Python 3.11 compatible
async def game_loop(args):
    await asyncio.sleep(0.1)

loop = asyncio.new_event_loop()
loop.run_until_complete(asyncio.gather(game_loop(args), server()))
loop.close()
```

---

### Fix 6 — `is` with string literals (SyntaxWarning → broken logic)

**Problem**: Two comparisons used `is` and `is not` with string literals. In Python 3.8 these were warnings; in Python 3.11 they produce incorrect results.

```python
# Before (broken)
if cur_junction_id is 'Green':
if _tl_state is not 'none':

# After (correct)
if cur_junction_id == 'Green':
if _tl_state != 'none':
```

---

### Fix 7 — Low simulation FPS (GPU performance)

**Problem**: The CARLA server ran at ~8 FPS because:
- CARLA defaulted to Epic rendering quality even in headless (`-RenderOffScreen`) mode.
- The RGB camera sensor was rendering at the full display resolution (1920×1080).

**Fix**:
- Added `-quality-level=Low` to `run_carla.sh`.
- Reduced the default pygame window resolution from `1920x1080` to `1280x720` (camera sensor resolution matches the window, so both shrink together).

---

## Troubleshooting

| Error | Fix |
|---|---|
| `ModuleNotFoundError: No module named 'websocket'` | `pip install websocket-client` |
| `RuntimeError: time-out of 2000ms while waiting for the simulator` | Start CARLA first (`./run_carla.sh`) and wait ~15s before running `./run_main.sh` |
| `bind failed. Error: Address already in use` | Find and terminate the old `spiral_planner` process by PID (`ps -aux | grep spiral`) |
| `libcarla_client.a: No such file or directory` | Ensure `starter_files/libcarla-install/lib/` contains the CARLA SDK static libs |
| `AttributeError: module 'asyncio' has no attribute 'coroutine'` | Python ≥ 3.11: apply Fix 5 above to `simulatorAPI.py` |
| CARLA uses integrated GPU instead of NVIDIA | Add PRIME offload env vars to `run_carla.sh` (see Fix 1) |
| Pygame window shows large black area | Camera sensor resolution and window resolution must match; do not set them independently |

---

## Project Structure

```
project/
├── CARLA/                              # CARLA simulator binary (not committed)
├── course-content/                     # Udacity course materials
├── starter_files/
│   ├── behavior_planner_FSM.cpp        # FSM implementation
│   ├── cost_functions.cpp              # Collision + proximity costs
│   ├── motion_planner.cpp              # Lateral goal generation
│   ├── velocity_profile_generator.cpp  # Kinematic velocity profiles
│   ├── planning_params.h               # Tunable parameters
│   ├── CMakeLists.txt
│   └── spiral_planner                  # Built binary (not committed)
├── simulatorAPI.py                     # Python CARLA <-> WebSocket bridge
├── run_carla.sh                        # Launch CARLA
├── run_main.sh                         # Launch planner + simulator API
└── install-ubuntu.sh                   # Install uWebSockets
```

---
