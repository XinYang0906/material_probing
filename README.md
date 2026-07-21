# Material Probing

This repository combines the probing primitive code with the ROS2/MoveIt
workspace support needed to run the primitives on a Franka FR3 setup.

The repository is intentionally split into two top-level project folders:

```text
material_probing/
  probing/              # Python package, primitive scripts, data, docs, models
  robot_ros2_control/   # ROS2 workspace setup, Docker files, dependency repos, patches
  README.md
```

## Folder Roles

`probing/` contains the material probing code:

```text
probing/
  backends/             # MuJoCo and FR3 ROS2/MoveIt backends
  controllers/          # Contact-control skeletons
  data/                 # Small example MuJoCo CSV outputs
  docs/                 # FR3 probing notes and runbooks
  models/               # MuJoCo Panda model assets
  primitives/           # Probe primitive definitions
  scripts/              # MuJoCo, fake-hardware, and real-robot launch entrypoints
  sensors/              # F/T, telemetry, and trajectory plotting utilities
  config.py
  ros2_probe_runner.py
```

`robot_ros2_control/` contains the complete ROS2 source workspace together
with its Docker setup. Third-party source trees are vendored so a normal GitHub
clone includes the exact local versions and patches used by this project:

```text
robot_ros2_control/
  franka_description/
  franka_fr3_moveit_config/
  libfranka/
  ros2_control/
  ...                    # Remaining ROS2 packages and model assets
  dependency.repos      # ROS2/Franka source dependencies
  Dockerfile
  docker-compose.yml
  franka_entrypoint.sh
  limits.conf
  patches/              # Local ros2_control/franka patches applied in the container
```

The ROS2 route targets `frankarobotics/franka_ros2` with
`franka_fr3_moveit_config`, `fr3_arm`, `fr3_link0`, `spatula_tip`, and
`fr3_arm_controller`.

## Supported Primitives

| Primitive | Stage sequence |
| --- | --- |
| `approach_smoke` | `APPROACH` |
| `trajectory_smoke` | `APPROACH -> DOWN -> HOLD -> RETRACT` |
| `micro_compression` | `APPROACH -> COMPRESS -> HOLD -> RETRACT` |
| `horizontal_drag` | `APPROACH -> INSERT -> DRAG -> RETRACT` |
| `lift_detach` | `APPROACH -> COMPRESS -> HOLD -> LIFT` |
| `inclined_insertion` | `APPROACH -> TILT -> INSERT -> HOLD -> RETRACT -> UNTILT` |

Fake-hardware safety limits:

- `compression_depth <= 0.03 m`
- `insert_depth <= 0.03 m`
- `inclined_insertion` tilt angle must be `10-15 deg`

These limits are for fake-hardware trajectory validation only. They are not a
complete real-contact safety policy.

## ROS2 Workspace Setup

From the repository root, start the ROS2/Franka workspace helper container:

```bash
cd robot_ros2_control
cp .env.example .env
docker compose up -d --build
```

Inside the container, the checked-in source directories are already mounted at
`/ros2_ws/src`. The `vcs import` command is optional and uses `--skip-existing`,
so it only restores a dependency if its directory is missing:

```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
vcs import src < src/dependency.repos --recursive --skip-existing
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The entrypoint applies the local patch files from
`robot_ros2_control/patches/` into the ROS2 source workspace when possible.

## Copy or Mount the Probing Package

The probing package should be available in the container at a path whose parent
directory is on `PYTHONPATH`. A common layout is:

```text
/ros2_ws/material_probing/probing
```

For example, from the host:

```bash
docker cp probing franka_ros2:/ros2_ws/material_probing/probing
```

Then enter the container and source ROS2:

```bash
docker exec -it franka_ros2 /bin/bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
```

## Running FR3 Fake Hardware

Terminal 1, inside the container:

```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=dont-care \
  use_fake_hardware:=true \
  use_spatula_tip:=true
```

Terminal 2, inside the container:

```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
```

Approach smoke test:

```bash
ros2 launch /ros2_ws/material_probing/probing/scripts/run_fr3_probe.launch.py \
  probe_primitive:=approach_smoke
```

Micro-compression:

```bash
ros2 launch /ros2_ws/material_probing/probing/scripts/run_fr3_probe.launch.py \
  probe_primitive:=micro_compression \
  compression_depth:=0.005 \
  hold_s:=1.0
```

Horizontal drag:

```bash
ros2 launch /ros2_ws/material_probing/probing/scripts/run_fr3_probe.launch.py \
  probe_primitive:=horizontal_drag \
  insert_depth:=0.005 \
  drag_distance:=-0.030 \
  hold_s:=1.0
```

## Telemetry and Plots

Write a telemetry CSV:

```bash
ros2 launch /ros2_ws/material_probing/probing/scripts/run_fr3_probe.launch.py \
  probe_primitive:=micro_compression \
  compression_depth:=0.005 \
  telemetry_log_path:=/ros2_ws/material_probing/probing/output/micro_compression_telemetry.csv
```

Save a stage-end trajectory plot:

```bash
ros2 launch /ros2_ws/material_probing/probing/scripts/run_fr3_probe.launch.py \
  probe_primitive:=horizontal_drag \
  insert_depth:=0.005 \
  drag_distance:=-0.030 \
  trajectory_plot_path:=/ros2_ws/material_probing/probing/output/horizontal_drag_trajectory.svg
```

## Running MuJoCo

From the repository root:

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
ros2 launch /ros2_ws/material_probing/probing/scripts/run_real_robot_probe.launch.py \
  robot_ip:=<FR3_IP> \
  use_fake_hardware:=false \
  probe_primitive:=approach_smoke \
  approach_z:=0.55 \
  telemetry_log_path:=/tmp/fr3_real_approach_preflight.csv
```

Do not run `trajectory_smoke`, `micro_compression`, `horizontal_drag`,
`lift_detach`, or `inclined_insertion` on hardware until the items in
`probing/docs/fr3_real_robot_migration_runbook.md` are complete.
