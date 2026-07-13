# FR3 Real Robot Migration Runbook

This runbook is the handoff checklist for moving the validated FR3 fake
hardware probing stack to a physical Franka FR3. Do not add new primitives in
this phase. The first real-robot milestone is a high, non-contact
`approach_smoke` motion with telemetry enabled.

## Migration Package

Copy or mount these project files into the container workspace:

```bash
docker cp ~/2026livsurf_project/probing franka_ros2:/ros2_ws/probing
```

Important probing files:

- `probing/backends/ros2_moveit_backend.py`
- `probing/ros2_probe_runner.py`
- `probing/scripts/run_fr3_probe.py`
- `probing/scripts/run_fr3_probe.launch.py`
- `probing/scripts/run_real_robot_probe.py`
- `probing/scripts/run_real_robot_probe.launch.py`
- `probing/sensors/ft_sensor.py`
- `probing/sensors/ros2_probe_telemetry.py`
- `probing/controllers/cartesian_contact_controller.py`
- `probing/docs/fr3_ros2_moveit_architecture.md`
- `probing/docs/fr3_tool_tcp_plan.md`
- `probing/docs/fr3_real_robot_migration_runbook.md`

The real-robot entry is intentionally conservative:

- `run_real_robot_probe.py` defaults to `approach_z=0.55`.
- `run_real_robot_probe.py` defaults to `allow_contact_primitives=false`.
- `run_real_robot_probe.launch.py` passes `robot_ip`, `use_fake_hardware`, and
  `allow_contact_primitives` into the shared runner.
- With the default guard, only `approach_smoke` is allowed. `trajectory_smoke`
  and all contact-style primitives are blocked.

## franka_ros2 Spatula TCP Changes

These changes must exist in the target `frankarobotics/franka_ros2`
workspace before probing with `tip_link:=spatula_tip`.

`franka_description/robots/fr3/fr3.urdf.xacro`

- Adds xacro args:
  - `use_spatula_tip`
  - `spatula_tip_name`
  - `spatula_tip_xyz`
  - `spatula_tip_rpy`
- When enabled, adds a fixed joint from `fr3_hand_tcp` to `spatula_tip`.
- Current placeholder transform:

```text
parent: fr3_hand_tcp
child:  spatula_tip
xyz:    0 0 0.2366
rpy:    0 0 0
```

`franka_description/robots/fr3/fr3.srdf.xacro`

- Passes `use_spatula_tip` and `spatula_tip_name` into the shared arm SRDF
  macro.

`franka_description/robots/common/franka_arm.srdf.xacro`

- Chooses the planning chain tip from:
  - `spatula_tip` when `use_spatula_tip:=true`
  - `fr3_hand_tcp` for the standard FR3 hand path
  - `fr3_link8` for no-hand configurations

`franka_fr3_moveit_config/launch/moveit.launch.py`

- Declares and forwards:
  - `use_spatula_tip`
  - `spatula_tip_name`
  - `spatula_tip_xyz`
  - `spatula_tip_rpy`
- Defaults `use_spatula_tip:=true`.
- Spawns `franka_robot_state_broadcaster` only for real hardware
  (`UnlessCondition(use_fake_hardware)`).

Before contact experiments, replace the placeholder TCP with measured tool
calibration and update collision geometry/payload settings for the mounted
spatula assembly.

## Preflight Safety Checklist

Complete every item before the first real-robot approach:

- E-stop reachable by the operator.
- Desk, arm, tool, cable routing, and workspace are clear.
- Robot is in a known safe start state with low speed override.
- No sample, fixture, or human body part is under the tool.
- `spatula_tip_xyz` and `spatula_tip_rpy` are measured or explicitly treated
  as provisional for non-contact testing only.
- Tool payload, center of mass, and collision model are reviewed.
- `fr3_arm_controller` and `joint_state_broadcaster` are active.
- `franka_robot_state_broadcaster` is active on real hardware.
- Wrench or robot-state topic is visible before any contact-stage test.
- Telemetry CSV path is set and writable.
- `allow_contact_primitives:=false` for the first real-robot run.
- `probe_primitive:=approach_smoke` for the first real-robot run.
- `approach_z` is high enough to be visibly non-contact.

Do not run `trajectory_smoke`, `micro_compression`, `horizontal_drag`,
`lift_detach`, or `inclined_insertion` on the robot until contact aborts and
constant-speed Cartesian contact control are implemented and tested.

## Bringup Steps

Start the Docker environment:

```bash
cd ~/franka_ros2
docker compose up -d
docker exec -it franka_ros2 /bin/bash
```

Inside the container, build if the workspace changed:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Launch the official FR3 MoveIt stack against real hardware:

```bash
cd /ros2_ws
source install/setup.bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=<FR3_IP> \
  use_fake_hardware:=false \
  use_spatula_tip:=true \
  spatula_tip_xyz:="0 0 0.2366" \
  spatula_tip_rpy:="0 0 0"
```

Replace the `spatula_tip_xyz` and `spatula_tip_rpy` values with the measured
tool calibration before contact.

In a second terminal inside the same container, verify the graph:

```bash
cd /ros2_ws
source install/setup.bash
ros2 control list_controllers
ros2 node list
ros2 param get /move_group robot_description_semantic | grep spatula_tip
ros2 run tf2_ros tf2_echo fr3_link0 spatula_tip
ros2 topic list -t | grep -E 'wrench|robot_state|joint_states|tf'
```

Expected minimum state:

```text
joint_state_broadcaster active
fr3_arm_controller active
/move_group
/controller_manager
```

For real hardware, also verify that
`/franka_robot_state_broadcaster/external_wrench_in_base_frame` or
`/franka_robot_state_broadcaster/robot_state` is publishing before any
contact-stage work.

Run only the high, non-contact approach preflight:

```bash
cd /ros2_ws
source install/setup.bash
ros2 launch /ros2_ws/probing/scripts/run_real_robot_probe.launch.py \
  robot_ip:=<FR3_IP> \
  use_fake_hardware:=false \
  probe_primitive:=approach_smoke \
  approach_z:=0.55 \
  telemetry_log_path:=/tmp/fr3_real_approach_preflight.csv
```

Success criteria:

- `fr3_arm_controller` remains active.
- The arm moves only to the high approach pose.
- Terminal output includes `Reached FR3 probing approach pose.`
- The telemetry CSV exists and contains `APPROACH` start/done rows.
- No contact occurs.

## Contact Primitive Lockout

The real-robot runner blocks every entry except `approach_smoke` unless
`allow_contact_primitives` is explicitly set true. This includes
`trajectory_smoke`, because it contains a DOWN stage.

The unlock command shape is:

```bash
ros2 launch /ros2_ws/probing/scripts/run_real_robot_probe.launch.py \
  robot_ip:=<FR3_IP> \
  use_fake_hardware:=false \
  allow_contact_primitives:=true \
  probe_primitive:=micro_compression
```

Do not use that command until all of these are complete:

- Calibrated `spatula_tip` transform is installed.
- Tool payload and collision geometry are configured.
- F/T logging is receiving real samples.
- Wrench bias procedure is implemented.
- Contact abort thresholds are configured and tested.
- Constant-speed Cartesian/contact-stage controller is implemented.
- A dry run proves timeout, force, stale-sensor, and travel-limit stops.

MoveIt pose goals are acceptable for non-contact approach and retract. They are
not the final control method for real compression, insertion, drag, or lift
contact.

## Rollback

To return to the validated fake-hardware path:

```bash
cd /ros2_ws
source install/setup.bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=dont-care \
  use_fake_hardware:=true \
  use_spatula_tip:=true
```

Then run:

```bash
ros2 launch /ros2_ws/probing/scripts/run_fr3_probe.launch.py \
  probe_primitive:=approach_smoke
```

The fake-hardware launch keeps `allow_contact_primitives:=true` by default so
the four verified pose-stage primitives remain available for regression tests.
