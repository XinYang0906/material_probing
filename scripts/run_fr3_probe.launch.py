import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace_name = "namespace"
    primitive_name = "probe_primitive"
    sequence_name = "probe_sequence"
    x_name = "probe_x"
    y_name = "probe_y"
    approach_z_name = "approach_z"
    smoke_down_z_name = "smoke_down_z"
    compression_depth_name = "compression_depth"
    insert_depth_name = "insert_depth"
    drag_distance_name = "drag_distance"
    lift_height_name = "lift_height"
    tilt_deg_name = "tilt_deg"
    hold_s_name = "hold_s"
    planning_group_name = "planning_group"
    base_frame_name = "base_frame"
    tip_link_name = "tip_link"
    controller_name = "controller"
    backend_mode_name = "backend_mode"
    pose_goal_mode_name = "pose_goal_mode"
    robot_ip_name = "robot_ip"
    use_fake_hardware_name = "use_fake_hardware"
    allow_contact_primitives_name = "allow_contact_primitives"
    telemetry_log_path_name = "telemetry_log_path"
    trajectory_plot_path_name = "trajectory_plot_path"
    wrench_topic_name = "wrench_topic"
    robot_state_topic_name = "robot_state_topic"
    telemetry_sample_period_s_name = "telemetry_sample_period_s"

    namespace = LaunchConfiguration(namespace_name)
    primitive = LaunchConfiguration(primitive_name)
    sequence = LaunchConfiguration(sequence_name)
    x = LaunchConfiguration(x_name)
    y = LaunchConfiguration(y_name)
    approach_z = LaunchConfiguration(approach_z_name)
    smoke_down_z = LaunchConfiguration(smoke_down_z_name)
    compression_depth = LaunchConfiguration(compression_depth_name)
    insert_depth = LaunchConfiguration(insert_depth_name)
    drag_distance = LaunchConfiguration(drag_distance_name)
    lift_height = LaunchConfiguration(lift_height_name)
    tilt_deg = LaunchConfiguration(tilt_deg_name)
    hold_s = LaunchConfiguration(hold_s_name)
    planning_group = LaunchConfiguration(planning_group_name)
    base_frame = LaunchConfiguration(base_frame_name)
    tip_link = LaunchConfiguration(tip_link_name)
    controller = LaunchConfiguration(controller_name)
    backend_mode = LaunchConfiguration(backend_mode_name)
    pose_goal_mode = LaunchConfiguration(pose_goal_mode_name)
    robot_ip = LaunchConfiguration(robot_ip_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_name)
    allow_contact_primitives = LaunchConfiguration(allow_contact_primitives_name)
    telemetry_log_path = LaunchConfiguration(telemetry_log_path_name)
    trajectory_plot_path = LaunchConfiguration(trajectory_plot_path_name)
    wrench_topic = LaunchConfiguration(wrench_topic_name)
    robot_state_topic = LaunchConfiguration(robot_state_topic_name)
    telemetry_sample_period_s = LaunchConfiguration(telemetry_sample_period_s_name)

    script_dir = Path(__file__).resolve().parent
    project_root = script_dir.parents[1]
    script_path = script_dir / "run_fr3_probe.py"
    moveit_py_params = script_dir / "moveit_py.yaml"

    python_path = str(project_root)
    if os.environ.get("PYTHONPATH"):
        python_path = python_path + os.pathsep + os.environ["PYTHONPATH"]

    return LaunchDescription(
        [
            DeclareLaunchArgument(namespace_name, default_value=""),
            DeclareLaunchArgument(primitive_name, default_value="approach_smoke"),
            DeclareLaunchArgument(sequence_name, default_value="approach"),
            DeclareLaunchArgument(x_name, default_value="0.40"),
            DeclareLaunchArgument(y_name, default_value="0.00"),
            DeclareLaunchArgument(approach_z_name, default_value="0.45"),
            DeclareLaunchArgument(smoke_down_z_name, default_value="0.43"),
            DeclareLaunchArgument(compression_depth_name, default_value="0.005"),
            DeclareLaunchArgument(insert_depth_name, default_value="0.005"),
            DeclareLaunchArgument(drag_distance_name, default_value="-0.030"),
            DeclareLaunchArgument(lift_height_name, default_value="0.040"),
            DeclareLaunchArgument(tilt_deg_name, default_value="12.0"),
            DeclareLaunchArgument(hold_s_name, default_value="1.0"),
            DeclareLaunchArgument(planning_group_name, default_value="fr3_arm"),
            DeclareLaunchArgument(base_frame_name, default_value="fr3_link0"),
            DeclareLaunchArgument(tip_link_name, default_value="spatula_tip"),
            DeclareLaunchArgument(controller_name, default_value="fr3_arm_controller"),
            DeclareLaunchArgument(backend_mode_name, default_value="move_group_action"),
            DeclareLaunchArgument(
                pose_goal_mode_name,
                default_value="current_orientation",
            ),
            DeclareLaunchArgument(robot_ip_name, default_value="dont-care"),
            DeclareLaunchArgument(use_fake_hardware_name, default_value="true"),
            DeclareLaunchArgument(
                allow_contact_primitives_name,
                default_value="true",
                description="Fake-hardware demos allow contact-style stage entries.",
            ),
            DeclareLaunchArgument(telemetry_log_path_name, default_value=""),
            DeclareLaunchArgument(trajectory_plot_path_name, default_value=""),
            DeclareLaunchArgument(
                wrench_topic_name,
                default_value=(
                    "/franka_robot_state_broadcaster/"
                    "external_wrench_in_base_frame"
                ),
            ),
            DeclareLaunchArgument(
                robot_state_topic_name,
                default_value="/franka_robot_state_broadcaster/robot_state",
            ),
            DeclareLaunchArgument(telemetry_sample_period_s_name, default_value="0.1"),
            Node(
                executable="python3",
                name="run_fr3_probe",
                namespace=namespace,
                output="screen",
                arguments=[
                    str(script_path),
                    "--primitive",
                    primitive,
                    "--sequence",
                    sequence,
                    "--x",
                    x,
                    "--y",
                    y,
                    "--approach-z",
                    approach_z,
                    "--smoke-down-z",
                    smoke_down_z,
                    "--compression-depth",
                    compression_depth,
                    "--insert-depth",
                    insert_depth,
                    "--drag-distance",
                    drag_distance,
                    "--lift-height",
                    lift_height,
                    "--tilt-deg",
                    tilt_deg,
                    "--hold-s",
                    hold_s,
                    "--planning-group",
                    planning_group,
                    "--base-frame",
                    base_frame,
                    "--tip-link",
                    tip_link,
                    "--controller",
                    controller,
                    "--backend-mode",
                    backend_mode,
                    "--pose-goal-mode",
                    pose_goal_mode,
                    "--robot-ip",
                    robot_ip,
                    "--use-fake-hardware",
                    use_fake_hardware,
                    "--allow-contact-primitives",
                    allow_contact_primitives,
                    "--telemetry-log-path",
                    telemetry_log_path,
                    "--trajectory-plot-path",
                    trajectory_plot_path,
                    "--wrench-topic",
                    wrench_topic,
                    "--robot-state-topic",
                    robot_state_topic,
                    "--telemetry-sample-period-s",
                    telemetry_sample_period_s,
                ],
                parameters=[str(moveit_py_params)],
                additional_env={
                    "PYTHONPATH": python_path,
                    "PYTHONUNBUFFERED": "1",
                },
            ),
        ]
    )
