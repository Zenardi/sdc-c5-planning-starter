# Copilot Instructions

## Project Overview

This is a C++ autonomous vehicle motion planning project (Udacity SDC ND, Course 5). It implements a hierarchical planner with a **Behavior Planner FSM** and a **Motion Planner** that communicate with the **CARLA 0.9.9.4 simulator** via WebSocket.

## Build & Run

All commands assume the CARLA simulator is under `project/CARLA/` (local binary at `0.9.16`).

```bash
# Terminal 1 – start CARLA
cd project && ./run_carla.sh

# Terminal 2 – install dependencies (first time only)
cd project && ./install-ubuntu.sh   # installs uWebSockets

# Build (from starter_files/)
cd project/starter_files
cmake .
make

# Run the planner (from project/)
cd project && ./run_main.sh
# Launches spiral_planner binary, waits 1 s, then starts simulatorAPI.py
```

If port 4567 is already in use: `ps -aux | grep carla` then `kill <PID>`.

### Local CARLA C++ Client Library

`CMakeLists.txt` uses `libcarla-install/` (a symlink inside `starter_files/`) which points to the pre-built CARLA client SDK from the SDC C3 localization project:

```
starter_files/libcarla-install → .../sdc-localization-scan-matching/Lesson_7.../c3-project/libcarla-install
```

If that path changes, re-create the symlink:
```bash
ln -s <path-to-libcarla-install> project/starter_files/libcarla-install
```

The `carla` Python package (0.9.16) is installed from the local wheel:
```bash
pip install project/CARLA/PythonAPI/carla/dist/carla-0.9.16-cp311-cp311-manylinux_2_31_x86_64.whl
```

There is no automated test suite. Validation is done visually in the CARLA simulator.

## Architecture

```
CARLA Simulator
      ↕  (Python, simulatorAPI.py)
   main.cpp  ←→  WebSocket :4567
      │
      ├── BehaviorPlannerFSM        (behavior_planner_FSM.cpp/.h)
      │     └── state_transition()  →  goal State + active Maneuver
      │
      └── MotionPlanner             (motion_planner.cpp/.h)
            ├── generate_offset_goals()  →  N lateral goal candidates
            ├── CubicSpiral              →  smooth path per goal
            ├── VelocityProfileGenerator →  velocity profile per path
            └── cost_functions           →  pick best trajectory
```

- **`simulatorAPI.py`** bridges CARLA Python API ↔ WebSocket.
- **`main.cpp`** receives JSON sensor data (ego pose, waypoints, obstacles, traffic lights) and sends back `trajectory_x/y/v` + throttle/steer.
- **`cserver_dir/`** contains an alternative lightweight C WebSocket server; not used in the main flow.

## Files to Implement

The following files contain `// TODO` stubs that students must complete:

| File | What to implement |
|---|---|
| `behavior_planner_FSM.cpp` | `get_look_ahead_distance()`, stop-line goal offset, goal velocity vectors, all FSM state transitions |
| `motion_planner.cpp` | perpendicular offset direction (`yaw + π/2`), lateral goal positions |
| `cost_functions.cpp` | collision circle placement along vehicle body, Euclidean distance between circles, distance-to-main-goal cost |
| `velocity_profile_generator.cpp` | `calc_distance()` using `d = (v_f²−v_i²)/(2a)`, `calc_final_speed()` using `v_f = √(v_i²+2ad)` |
| `planning_params.h` | `P_NUM_PATHS` (try 3–5), `P_NUM_POINTS_IN_SPIRAL` (try 10–20) |

## Key Data Structures (`structs.h`)

```cpp
enum Maneuver { FOLLOW_LANE, FOLLOW_VEHICLE, DECEL_TO_STOP, STOPPED };

struct PathPoint   { double x, y, z, theta, kappa, s, dkappa, ddkappa; };
struct TrajectoryPoint { PathPoint path_point; double v, a, relative_time; };
struct State       { cg::Location location; cg::Rotation rotation;
                     cg::Vector3D velocity, acceleration; };
```

CARLA namespace aliases used throughout: `namespace cc = carla::client`, `cg = carla::geom`, `cr = carla::road`, `cf = carla::sensor::data`.

## Key Planning Parameters (`planning_params.h`)

| `#define` | Default | Meaning |
|---|---|---|
| `P_NUM_PATHS` | 1 | Number of lateral offset paths (**TODO**: increase) |
| `P_NUM_POINTS_IN_SPIRAL` | 2 | Spiral discretization points (**TODO**: increase) |
| `P_SPEED_LIMIT` | 3.0 m/s | Maximum desired speed |
| `P_LOOKAHEAD_MIN/MAX` | 8 / 20 m | Lookahead distance bounds |
| `P_GOAL_OFFSET` | 1.0 m | Lateral spacing between offset goals |
| `P_MAX_ACCEL` | 1.5 m/s² | Max acceleration magnitude |
| `P_STOP_THRESHOLD_SPEED` | 0.02 m/s | Speed threshold to enter STOPPED |
| `P_REQ_STOPPED_TIME` | 1.0 s | Time to hold STOPPED before releasing |
| `P_STOP_LINE_BUFFER` | 0.5 m | Buffer distance behind stop line |
| `CIRCLE_OFFSETS` | {-1, 1, 3} m | Longitudinal positions of collision circles |
| `CIRCLE_RADII` | {1.5, 1.5, 1.5} m | Radii of collision circles |

## Coding Conventions

- **Private members** prefixed with underscore: `_active_maneuver`, `_speed_limit`
- **Naming**: `snake_case` functions/variables, `PascalCase` classes and enums
- **Header guards**: `#pragma once`
- **Smart pointers**: `boost::shared_ptr<T>` (not `std::shared_ptr`)
- **Style**: Google-based, enforced by `.clang-format` (2-space indent, 80-char limit, pointer-left, brace-attach)

  Apply formatting with: `clang-format -i *.cpp *.h` from `starter_files/`

## FSM State Transitions

```
FOLLOW_LANE  ──[goal in junction]──►  DECEL_TO_STOP
                                            │
                              [speed ≤ threshold or dist ≤ threshold]
                                            ▼
                                         STOPPED
                                            │
                              [timer ≥ P_REQ_STOPPED_TIME
                               && traffic light ≠ Red]
                                            │
                                            └──────────────► FOLLOW_LANE
```

`_goal` is cached in `BehaviorPlannerFSM` so DECEL_TO_STOP and STOPPED states can return `goal = _goal` to hold the stop-line position.
