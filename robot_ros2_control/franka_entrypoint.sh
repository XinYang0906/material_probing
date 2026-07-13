#!/bin/bash

# Clone Franka dependencies into the workspace
vcs import /ros2_ws/src < /ros2_ws/src/dependency.repos --recursive --skip-existing

# Apply manage_overruns patch to hardware_interface (not yet upstream)
if [ -d /ros2_ws/src/ros2_control/hardware_interface ]; then
  git -C /ros2_ws/src/ros2_control apply /ros2_ws/src/patches/manage_overruns.patch --verbose 2>&1 || \
    echo "Warning: manage_overruns patch may already be applied or failed to apply"
fi

# fake_components mirrors position commands into joint state, unlike the real
# FR3 effort controller. Keep the real controller unchanged and add this
# override only when MoveIt is launched with use_fake_hardware:=true.
if [ -f /ros2_ws/src/patches/fake_hardware_position_controller.patch ]; then
  git -C /ros2_ws/src apply --recount \
    /ros2_ws/src/patches/fake_hardware_position_controller.patch --verbose 2>&1 || \
    echo "Warning: fake hardware position-controller patch may already be applied or failed to apply"
fi

exec "$@"
