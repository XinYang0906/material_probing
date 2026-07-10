# FR3 Probing Primitives

This repository contains probing primitives that were first validated in
MuJoCo and then connected to the official Franka FR3 ROS2/MoveIt pipeline.
The current ROS2 route targets `frankarobotics/franka_ros2` with
`franka_fr3_moveit_config`, `fr3_arm`, `fr3_link0`, `spatula_tip`, and
`fr3_arm_controller`.

The Panda MoveIt demo package `moveit_resources_panda_moveit_config` is not
used as the primary ROS2 path.

## Project Structure

```text
probing/
├── backends/
│   ├── mujoco_backend.py             # MuJoCo probing backend
│   ├── mujoco_warp_backend.py        # Experimental/future MuJoCo-Warp backend
│   └── ros2_moveit_backend.py        # FR3 ROS2/MoveIt pose-goal backend
├── controllers/
│   └── cartesian_contact_controller.py # Skeleton for future contact control
├── data/                             # Example MuJoCo CSV outputs
├── docs/
│   ├── fr3_ros2_moveit_architecture.md
│   ├── fr3_real_robot_migration_runbook.md
│   └── fr3_tool_tcp_plan.md
├── models/
│   └── franka_emika_panda/           # MuJoCo Panda model assets
├── primitives/
│   ├── micro_compression.py
│   ├── horizontal_drag.py
│   ├── lift_detach.py
│   └── inclined_insertion.py
├── scripts/
│   ├── run_single_probe_mujoco.py    # MuJoCo primitive runner
│   ├── run_fr3_probe.py              # FR3 fake-hardware wrapper
│   ├── run_fr3_probe.launch.py       # FR3 fake-hardware launch entry
│   ├── run_real_robot_probe.py       # Conservative real-robot wrapper
│   └── run_real_robot_probe.launch.py
├── sensors/
│   ├── ft_sensor.py                  # F/T sample and CSV interfaces
│   ├── ros2_probe_telemetry.py       # ROS2 telemetry logger
│   └── trajectory_plot.py            # Stage-end 3D trajectory plot output
├── config.py                         # MuJoCo primitive defaults
└── ros2_probe_runner.py              # Shared FR3 primitive/backend/safety runner
```

## Supported Primitives

The FR3 MoveIt runner supports these pose-stage entries:

| Primitive | Stage sequence |
| --- | --- |
| `approach_smoke` | `APPROACH` |
| `trajectory_smoke` | `APPROACH -> DOWN -> HOLD -> RETRACT` |
| `micro_compression` | `APPROACH -> COMPRESS -> HOLD -> RETRACT` |
| `horizontal_drag` | `APPROACH -> INSERT -> DRAG -> RETRACT` |
| `lift_detach` | `APPROACH -> COMPRESS -> HOLD -> LIFT` |
| `inclined_insertion` | `APPROACH -> TILT -> INSERT -> HOLD -> RETRACT -> UNTILT` |

Current fake-hardware safety limits:

- `compression_depth <= 0.03 m`
- `insert_depth <= 0.03 m`
- `inclined_insertion` tilt angle must be `10-15 deg`

These limits are for fake-hardware trajectory validation only. They are not a
complete real-contact safety policy.

## System Requirements

### FR3 ROS2/MoveIt Route

- Ubuntu host with Docker Compose
- X11 forwarding configured if RViz should appear from the container
- A built `frankarobotics/franka_ros2` workspace in the container
- ROS2 Jazzy in the container
- `franka_fr3_moveit_config` available in `/ros2_ws/install`
- This repository copied or mounted at `/ros2_ws/probing`

The tested container layout was:

```text
container name: franka_ros2
workspace:      /ros2_ws
probing path:   /ros2_ws/probing
```

### MuJoCo Route

- Python 3
- `mujoco`
- `numpy`

MuJoCo is useful for primitive development and offline checks. The FR3
hardware/fake-hardware path does not replay MuJoCo joint trajectories; it maps
primitive stage semantics to MoveIt pose goals.

## Installation

### 1. Build the FR3 ROS2 Workspace

In the `franka_ros2` container:

```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
vcs import src < src/dependency.repos --recursive --skip-existing
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 2. Copy This Repository Into the Container

From the host:

```bash
docker cp ~/2026livsurf_project/probing franka_ros2:/ros2_ws/probing
```

Then enter the container:

```bash
docker exec -it franka_ros2 /bin/bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
```

### 3. Enable RViz From Docker

On the host:

```bash
xhost +local:docker
```

Then start or restart the container from the `franka_ros2` checkout:

```bash
cd ~/franka_ros2
docker compose up -d
```

If RViz does not appear, check for X11 errors such as `could not connect to
display :0` in the MoveIt launch log.

## Running FR3 Fake Hardware With RViz

Use two terminals inside the container.

### Terminal 1: Start FR3 MoveIt/RViz

```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=dont-care \
  use_fake_hardware:=true \
  use_spatula_tip:=true
```

Expected checks:

```bash
ros2 control list_controllers
ros2 node list | grep -E '^/controller_manager$|^/move_group$|^/robot_state_publisher$'
```

Expected controller state:

```text
joint_state_broadcaster active
fr3_arm_controller active
```

### Terminal 2: Run Probing Commands

Source the workspace first:

```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
```

Approach smoke test:

```bash
ros2 launch /ros2_ws/probing/scripts/run_fr3_probe.launch.py \
  probe_primitive:=approach_smoke
```

Micro-compression:

```bash
ros2 launch /ros2_ws/probing/scripts/run_fr3_probe.launch.py \
  probe_primitive:=micro_compression \
  compression_depth:=0.005 \
  hold_s:=1.0
```

Horizontal drag:

```bash
ros2 launch /ros2_ws/probing/scripts/run_fr3_probe.launch.py \
  probe_primitive:=horizontal_drag \
  insert_depth:=0.005 \
  drag_distance:=-0.030 \
  hold_s:=1.0
```

Lift-detach:

```bash
ros2 launch /ros2_ws/probing/scripts/run_fr3_probe.launch.py \
  probe_primitive:=lift_detach \
  compression_depth:=0.005 \
  lift_height:=0.040 \
  hold_s:=1.0
```

Inclined insertion:

```bash
ros2 launch /ros2_ws/probing/scripts/run_fr3_probe.launch.py \
  probe_primitive:=inclined_insertion \
  insert_depth:=0.005 \
  tilt_deg:=12.0 \
  hold_s:=1.0
```

## Telemetry and Trajectory Plotting

To write a CSV with stage, commanded pose, measured tip pose, and wrench fields:

```bash
ros2 launch /ros2_ws/probing/scripts/run_fr3_probe.launch.py \
  probe_primitive:=micro_compression \
  compression_depth:=0.005 \
  telemetry_log_path:=/ros2_ws/probing/output/micro_compression_telemetry.csv
```

To save a stage-end 3D trajectory plot:

```bash
ros2 launch /ros2_ws/probing/scripts/run_fr3_probe.launch.py \
  probe_primitive:=horizontal_drag \
  insert_depth:=0.005 \
  drag_distance:=-0.030 \
  trajectory_plot_path:=/ros2_ws/probing/output/horizontal_drag_trajectory.svg
```

The trajectory plot records the actual `spatula_tip` TF after each motion
stage and overlays the commanded pose points.

## Running MuJoCo

From the parent directory that contains the `probing` package:

```bash
python3 -m probing.scripts.run_single_probe_mujoco \
  --primitive micro_compression
```

Other primitive choices:

```bash
python3 -m probing.scripts.run_single_probe_mujoco --primitive horizontal_drag
python3 -m probing.scripts.run_single_probe_mujoco --primitive lift_detach
python3 -m probing.scripts.run_single_probe_mujoco --primitive inclined_insertion
```

## Real Robot Safety

The real-robot launch path is intentionally conservative. By default it allows
only a high, non-contact `approach_smoke` test:

```bash
ros2 launch /ros2_ws/probing/scripts/run_real_robot_probe.launch.py \
  robot_ip:=<FR3_IP> \
  use_fake_hardware:=false \
  probe_primitive:=approach_smoke \
  approach_z:=0.55 \
  telemetry_log_path:=/tmp/fr3_real_approach_preflight.csv
```

Do not run `trajectory_smoke`, `micro_compression`, `horizontal_drag`,
`lift_detach`, or `inclined_insertion` on hardware until all of the following
are ready:

- calibrated `spatula_tip` transform
- tool payload and collision geometry
- F/T logging with real samples
- contact abort thresholds
- constant-speed Cartesian/contact-stage controller
- tested timeout, force, stale-sensor, and travel-limit stops

See `docs/fr3_real_robot_migration_runbook.md` for the full checklist.

## Uploading to GitHub

Recommended files to commit:

- Python source: `backends/`, `controllers/`, `primitives/`, `scripts/`,
  `sensors/`, `config.py`, `ros2_probe_runner.py`, `__init__.py`
- Documentation: `README.md`, `docs/`
- MuJoCo assets needed by the offline demo: `models/`
- Small example CSV files: `data/`
- Repo hygiene: `.gitignore`

Do not commit generated caches or local runtime outputs:

- `__pycache__/`
- `*.pyc`
- `output/`
- ROS logs and colcon `build/`, `install/`, `log/`
- local editor or agent metadata

If you want to include example trajectory SVGs in the repository, move them to
a documented examples directory or force-add selected files from `output/`.
