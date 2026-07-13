import argparse
import math
import os
import time

from probing.backends.ros2_moveit_backend import ROS2MoveItBackend
from probing.sensors.ros2_probe_telemetry import ProbeTelemetryLogger
from probing.sensors.trajectory_plot import ProbeTrajectoryPlotter


FR3_PLANNING_GROUP = "fr3_arm"
FR3_BASE_FRAME = "fr3_link0"
FR3_TIP_LINK = "spatula_tip"
FR3_CONTROLLER = "fr3_arm_controller"
DEFAULT_WRENCH_TOPIC = (
    "/franka_robot_state_broadcaster/external_wrench_in_base_frame"
)
DEFAULT_ROBOT_STATE_TOPIC = "/franka_robot_state_broadcaster/robot_state"

MAX_VERTICAL_DEPTH_M = 0.03
MIN_INCLINED_TILT_DEG = 10.0
MAX_INCLINED_TILT_DEG = 15.0

PRIMITIVE_CHOICES = (
    "approach_smoke",
    "trajectory_smoke",
    "micro_compression",
    "horizontal_drag",
    "lift_detach",
    "inclined_insertion",
)
NONCONTACT_PRIMITIVES = ("approach_smoke",)
CONTACT_OR_DOWN_PRIMITIVES = tuple(
    primitive for primitive in PRIMITIVE_CHOICES
    if primitive not in NONCONTACT_PRIMITIVES
)


def bool_arg(value):
    if isinstance(value, bool):
        return value

    normalized = str(value).strip().lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise argparse.ArgumentTypeError("value must be true or false")


def positive_float(value):
    value = float(value)
    if value <= 0.0:
        raise argparse.ArgumentTypeError("value must be positive")
    return value


def add_probe_arguments(
    parser,
    default_tip_link=FR3_TIP_LINK,
    default_approach_z=0.45,
    default_use_fake_hardware=True,
    allow_contact_primitives_default=True,
):
    parser.add_argument(
        "--primitive",
        choices=PRIMITIVE_CHOICES,
        default="approach_smoke",
        help="FR3 probing primitive or smoke-test entry to execute.",
    )
    parser.add_argument(
        "--sequence",
        choices=("approach", "approach_down_retract"),
        default="approach",
        help="Backward-compatible smoke-test selector.",
    )
    parser.add_argument("--x", type=float, default=0.40)
    parser.add_argument("--y", type=float, default=0.00)
    parser.add_argument("--approach-z", type=float, default=default_approach_z)
    parser.add_argument("--smoke-down-z", type=float, default=0.43)
    parser.add_argument(
        "--compression-depth",
        type=positive_float,
        default=0.005,
        help="FR3 fake-hardware pose decrement for compression stages.",
    )
    parser.add_argument(
        "--insert-depth",
        type=positive_float,
        default=0.005,
        help="FR3 fake-hardware pose decrement for insertion stages.",
    )
    parser.add_argument(
        "--drag-distance",
        type=float,
        default=-0.030,
        help="Horizontal drag distance along the base-frame x axis.",
    )
    parser.add_argument(
        "--lift-height",
        type=positive_float,
        default=0.040,
        help="Lift distance upward from the compressed pose for lift_detach.",
    )
    parser.add_argument(
        "--tilt-deg",
        type=positive_float,
        default=12.0,
        help="Inclined insertion tilt angle in degrees; hard-limited to 10-15.",
    )
    parser.add_argument("--hold-s", type=positive_float, default=1.0)
    parser.add_argument(
        "--planning-group",
        default=FR3_PLANNING_GROUP,
        help="MoveIt planning group; FR3 default is fr3_arm.",
    )
    parser.add_argument(
        "--base-frame",
        default=FR3_BASE_FRAME,
        help="Pose goal reference frame; FR3 default is fr3_link0.",
    )
    parser.add_argument(
        "--tip-link",
        default=default_tip_link,
        help="MoveIt pose goal link; FR3 probing default is spatula_tip.",
    )
    parser.add_argument(
        "--controller",
        default=FR3_CONTROLLER,
        help="MoveIt execution controller name.",
    )
    parser.add_argument(
        "--backend-mode",
        choices=("auto", "moveit_py", "move_group_action"),
        default="move_group_action",
        help="Use /move_action by default for one-shot FR3 probe commands.",
    )
    parser.add_argument(
        "--pose-goal-mode",
        choices=("current_orientation", "position_only", "pose"),
        default="current_orientation",
        help=(
            "Use current_orientation for RViz/fake-hardware stage demos so "
            "MoveIt keeps the current TCP orientation while changing position. "
            "Use position_only to leave orientation unconstrained, or pose "
            "when qx/qy/qz/qw are intentional."
        ),
    )
    parser.add_argument(
        "--robot-ip",
        default="",
        help=(
            "Robot IP metadata passed through launch files. The probing node "
            "connects to an already running MoveIt graph and does not open "
            "the FCI connection itself."
        ),
    )
    parser.add_argument(
        "--use-fake-hardware",
        type=bool_arg,
        default=default_use_fake_hardware,
        help="Hardware-mode metadata used for run logging and safety checks.",
    )
    parser.add_argument(
        "--allow-contact-primitives",
        type=bool_arg,
        default=allow_contact_primitives_default,
        help=(
            "Allow trajectory_smoke and contact-style primitives. Real robot "
            "preflight defaults this to false and permits only approach_smoke."
        ),
    )
    parser.add_argument("--move-group-action", default="/move_action")
    parser.add_argument("--pipeline-id", default="move_group")
    parser.add_argument("--planner-id", default="RRTConnectkConfigDefault")
    parser.add_argument("--planning-time", type=positive_float, default=5.0)
    parser.add_argument(
        "--position-tolerance",
        type=positive_float,
        default=0.001,
        help="TCP goal tolerance in metres; 1 mm preserves 5 mm probe stages.",
    )
    parser.add_argument(
        "--orientation-tolerance",
        type=positive_float,
        default=0.035,
        help="TCP orientation tolerance in radians; about 2 degrees by default.",
    )
    parser.add_argument(
        "--max-velocity-scaling-factor",
        type=positive_float,
        default=0.1,
    )
    parser.add_argument(
        "--max-acceleration-scaling-factor",
        type=positive_float,
        default=0.1,
    )
    parser.add_argument("--action-timeout-s", type=positive_float, default=30.0)
    parser.add_argument(
        "--telemetry-log-path",
        default="",
        help=(
            "Optional CSV path for stage telemetry. When set, the runner logs "
            "time, primitive, stage, tip pose, commanded pose, and latest wrench."
        ),
    )
    parser.add_argument(
        "--trajectory-plot-path",
        default="",
        help=(
            "Optional SVG or PNG path for a stage-end 3D trajectory plot. "
            "The plot records the actual tip_link TF after each motion stage "
            "and overlays commanded pose points."
        ),
    )
    parser.add_argument(
        "--wrench-topic",
        default=DEFAULT_WRENCH_TOPIC,
        help=(
            "Optional geometry_msgs/WrenchStamped topic for F/T logging. "
            "The default is available when franka_robot_state_broadcaster is active."
        ),
    )
    parser.add_argument(
        "--robot-state-topic",
        default=DEFAULT_ROBOT_STATE_TOPIC,
        help=(
            "Optional franka_msgs/FrankaRobotState topic used as a fallback "
            "source for wrench and measured end-effector pose."
        ),
    )
    parser.add_argument(
        "--telemetry-sample-period-s",
        type=positive_float,
        default=0.1,
        help="Sampling period for telemetry rows during HOLD stages.",
    )
    parser.add_argument("--qx", type=float, default=0.0)
    parser.add_argument("--qy", type=float, default=0.0)
    parser.add_argument("--qz", type=float, default=0.0)
    parser.add_argument("--qw", type=float, default=1.0)
    return parser


def parse_probe_args(
    description=None,
    default_tip_link=FR3_TIP_LINK,
    default_approach_z=0.45,
    default_use_fake_hardware=True,
    allow_contact_primitives_default=True,
):
    parser = argparse.ArgumentParser(description=description)
    add_probe_arguments(
        parser,
        default_tip_link=default_tip_link,
        default_approach_z=default_approach_z,
        default_use_fake_hardware=default_use_fake_hardware,
        allow_contact_primitives_default=allow_contact_primitives_default,
    )
    args, _ = parser.parse_known_args()
    return args


def normalize_quat(quat):
    norm = math.sqrt(sum(component * component for component in quat))
    if norm == 0.0:
        raise ValueError("Quaternion norm must be non-zero.")
    return tuple(component / norm for component in quat)


def multiply_quat(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return normalize_quat(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )
    )


def rotation_y_quat(angle_rad):
    half_angle = 0.5 * angle_rad
    return (0.0, math.sin(half_angle), 0.0, math.cos(half_angle))


def tilted_orientation(args):
    base_orientation = normalize_quat((args.qx, args.qy, args.qz, args.qw))
    tilt = rotation_y_quat(math.radians(args.tilt_deg))
    return multiply_quat(tilt, base_orientation)


def resolve_current_tip_orientation(node, args, timeout_s=3.0):
    import rclpy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener

    tf_buffer = Buffer()
    TransformListener(tf_buffer, node, spin_thread=False)

    deadline = time.monotonic() + timeout_s
    last_error = None
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            transform = tf_buffer.lookup_transform(
                args.base_frame,
                args.tip_link,
                Time(),
            )
        except Exception as exc:
            last_error = exc
            continue

        q = transform.transform.rotation
        return normalize_quat((q.x, q.y, q.z, q.w))

    raise RuntimeError(
        "Could not resolve current TCP orientation from TF "
        f"{args.base_frame} -> {args.tip_link}: {last_error}"
    )


def prepare_goal_orientation(backend, args):
    if args.pose_goal_mode != "current_orientation":
        return

    if not hasattr(backend, "node"):
        raise RuntimeError(
            "pose_goal_mode=current_orientation requires a backend-owned ROS "
            "node. Use --backend-mode move_group_action."
        )

    qx, qy, qz, qw = resolve_current_tip_orientation(backend.node, args)
    args.qx = qx
    args.qy = qy
    args.qz = qz
    args.qw = qw
    print(
        "Using current TCP orientation from TF "
        f"{args.base_frame}->{args.tip_link}: "
        f"qx={qx:.4f}, qy={qy:.4f}, qz={qz:.4f}, qw={qw:.4f}."
    )


def selected_primitive(args):
    if args.primitive != "approach_smoke":
        return args.primitive
    if args.sequence == "approach_down_retract":
        return "trajectory_smoke"
    return "approach_smoke"


def validate_primitive_safety(args):
    primitive = selected_primitive(args)
    if primitive not in NONCONTACT_PRIMITIVES and not args.allow_contact_primitives:
        allowed = ", ".join(NONCONTACT_PRIMITIVES)
        blocked = ", ".join(CONTACT_OR_DOWN_PRIMITIVES)
        raise ValueError(
            f"Refusing {primitive}. Real-robot preflight permits only "
            f"{allowed} unless --allow-contact-primitives true is set after "
            "the safety checklist, calibrated TCP, F/T logging, and contact "
            f"abort policy are ready. Blocked entries: {blocked}."
        )
    return primitive


def build_backend(args, node_name="probing_moveit_backend"):
    print(
        "Using ROS2MoveItBackend "
        f"mode={args.backend_mode}, group={args.planning_group}, "
        f"base_frame={args.base_frame}, tip_link={args.tip_link}, "
        f"controller={args.controller}."
    )
    print(
        "Run metadata "
        f"robot_ip={args.robot_ip or '<unset>'}, "
        f"use_fake_hardware={args.use_fake_hardware}, "
        f"allow_contact_primitives={args.allow_contact_primitives}."
    )
    return ROS2MoveItBackend(
        planning_group=args.planning_group,
        base_frame=args.base_frame,
        tip_link=args.tip_link,
        controller_names=(args.controller,),
        node_name=node_name,
        move_group_action=args.move_group_action,
        backend_mode=args.backend_mode,
        pipeline_id=args.pipeline_id,
        planner_id=args.planner_id,
        planning_time=args.planning_time,
        position_tolerance=args.position_tolerance,
        orientation_tolerance=args.orientation_tolerance,
        max_velocity_scaling_factor=args.max_velocity_scaling_factor,
        max_acceleration_scaling_factor=args.max_acceleration_scaling_factor,
        action_timeout_s=args.action_timeout_s,
    )


def print_tooling_warning(args):
    if args.tip_link == "fr3_hand_tcp":
        print(
            "WARNING: FR3 probing is commanding fr3_hand_tcp pose goals. "
            "Use this only for smoke tests; real probing should use a "
            "calibrated spatula_tip/tool TCP plus force feedback safety."
        )
        return

    print(
        f"WARNING: FR3 probing is commanding pose goals for {args.tip_link}. "
        "Verify the tool TCP calibration and add force feedback/contact aborts "
        "before real contact."
    )


class ProbeRunObservers:
    """Fan out stage events to optional CSV telemetry and trajectory plotting."""

    def __init__(self, telemetry=None, trajectory_plotter=None):
        self.telemetry = telemetry
        self.trajectory_plotter = trajectory_plotter

    def warmup(self):
        if self.telemetry is not None:
            self.telemetry.warmup()
        if self.trajectory_plotter is not None:
            self.trajectory_plotter.warmup()

    def capture(self, stage, event="sample", command_pose=None):
        if self.telemetry is not None:
            self.telemetry.capture(
                stage,
                event=event,
                command_pose=command_pose,
            )
        if self.trajectory_plotter is not None:
            self.trajectory_plotter.capture(
                stage,
                event=event,
                command_pose=command_pose,
            )

    def sleep(self, stage, duration_s):
        if self.telemetry is not None:
            self.telemetry.sleep(stage, duration_s)
            return

        time.sleep(duration_s)

    def close(self):
        if self.telemetry is not None:
            self.telemetry.close()
        if self.trajectory_plotter is not None:
            self.trajectory_plotter.close()


def move_stage(
    backend,
    stage,
    x_m,
    y_m,
    z_m,
    args,
    orientation=None,
    telemetry=None,
):
    use_orientation_constraint = (
        orientation is not None
        or args.pose_goal_mode in ("current_orientation", "pose")
    )
    if orientation is None:
        orientation = (args.qx, args.qy, args.qz, args.qw)

    command_pose = (
        x_m,
        y_m,
        z_m,
        orientation[0],
        orientation[1],
        orientation[2],
        orientation[3],
    )

    print(stage)
    if telemetry is not None:
        telemetry.capture(stage, event="start", command_pose=command_pose)

    backend.move_to_pose(
        x_m,
        y_m,
        z_m,
        qx=orientation[0],
        qy=orientation[1],
        qz=orientation[2],
        qw=orientation[3],
        use_orientation_constraint=use_orientation_constraint,
    )

    if telemetry is not None:
        telemetry.capture(stage, event="done", command_pose=command_pose)


def hold_stage(stage, duration_s, telemetry=None):
    print(stage)
    if telemetry is None:
        time.sleep(duration_s)
        return

    telemetry.sleep(stage, duration_s)


def run_approach_smoke(backend, args, telemetry=None):
    print("APPROACH")
    use_orientation_constraint = args.pose_goal_mode in (
        "current_orientation",
        "pose",
    )
    command_pose = (
        args.x,
        args.y,
        args.approach_z,
        args.qx,
        args.qy,
        args.qz,
        args.qw,
    )
    if telemetry is not None:
        telemetry.capture("APPROACH", event="start", command_pose=command_pose)

    backend.move_to_pose(
        args.x,
        args.y,
        args.approach_z,
        qx=args.qx,
        qy=args.qy,
        qz=args.qz,
        qw=args.qw,
        use_orientation_constraint=use_orientation_constraint,
    )
    if telemetry is not None:
        telemetry.capture("APPROACH", event="done", command_pose=command_pose)

    print("Reached FR3 probing approach pose.")


def run_trajectory_smoke(backend, args, telemetry=None):
    move_stage(
        backend,
        "APPROACH",
        args.x,
        args.y,
        args.approach_z,
        args,
        telemetry=telemetry,
    )
    move_stage(
        backend,
        "DOWN",
        args.x,
        args.y,
        args.smoke_down_z,
        args,
        telemetry=telemetry,
    )

    hold_stage("HOLD", args.hold_s, telemetry=telemetry)

    move_stage(
        backend,
        "RETRACT",
        args.x,
        args.y,
        args.approach_z,
        args,
        telemetry=telemetry,
    )

    print("Retracted to FR3 probing approach pose.")
    print("Completed FR3 approach/down/retract smoke sequence.")


def run_micro_compression(backend, args, telemetry=None):
    if args.compression_depth > MAX_VERTICAL_DEPTH_M:
        raise ValueError(
            "Refusing micro_compression with compression depth "
            f"{args.compression_depth:.3f} m; max is "
            f"{MAX_VERTICAL_DEPTH_M:.3f} m for this fake-hardware entry."
        )

    compression_z = args.approach_z - args.compression_depth

    print_tooling_warning(args)
    print(
        "Running FR3 micro_compression pose-stage sequence "
        f"at x={args.x:.3f}, y={args.y:.3f}, approach_z={args.approach_z:.3f}, "
        f"compression_z={compression_z:.3f}."
    )

    move_stage(
        backend,
        "APPROACH",
        args.x,
        args.y,
        args.approach_z,
        args,
        telemetry=telemetry,
    )
    move_stage(
        backend,
        "COMPRESS",
        args.x,
        args.y,
        compression_z,
        args,
        telemetry=telemetry,
    )

    hold_stage("HOLD", args.hold_s, telemetry=telemetry)

    move_stage(
        backend,
        "RETRACT",
        args.x,
        args.y,
        args.approach_z,
        args,
        telemetry=telemetry,
    )

    print("Retracted to FR3 probing approach pose.")
    print("Completed FR3 micro_compression pose-stage sequence.")


def run_horizontal_drag(backend, args, telemetry=None):
    if args.insert_depth > MAX_VERTICAL_DEPTH_M:
        raise ValueError(
            "Refusing horizontal_drag with insert depth "
            f"{args.insert_depth:.3f} m; max is "
            f"{MAX_VERTICAL_DEPTH_M:.3f} m for this fake-hardware entry."
        )

    insert_z = args.approach_z - args.insert_depth
    drag_x = args.x + args.drag_distance

    print_tooling_warning(args)
    print(
        "Running FR3 horizontal_drag pose-stage sequence "
        f"at x={args.x:.3f}, y={args.y:.3f}, approach_z={args.approach_z:.3f}, "
        f"insert_z={insert_z:.3f}, drag_x={drag_x:.3f}."
    )

    move_stage(
        backend,
        "APPROACH",
        args.x,
        args.y,
        args.approach_z,
        args,
        telemetry=telemetry,
    )
    move_stage(
        backend,
        "INSERT",
        args.x,
        args.y,
        insert_z,
        args,
        telemetry=telemetry,
    )
    move_stage(
        backend,
        "DRAG",
        drag_x,
        args.y,
        insert_z,
        args,
        telemetry=telemetry,
    )
    move_stage(
        backend,
        "RETRACT",
        drag_x,
        args.y,
        args.approach_z,
        args,
        telemetry=telemetry,
    )

    print("Retracted at FR3 horizontal_drag end pose.")
    print("Completed FR3 horizontal_drag pose-stage sequence.")


def run_lift_detach(backend, args, telemetry=None):
    if args.compression_depth > MAX_VERTICAL_DEPTH_M:
        raise ValueError(
            "Refusing lift_detach with compression depth "
            f"{args.compression_depth:.3f} m; max is "
            f"{MAX_VERTICAL_DEPTH_M:.3f} m for this fake-hardware entry."
        )

    compression_z = args.approach_z - args.compression_depth
    lift_z = compression_z + args.lift_height

    print_tooling_warning(args)
    print(
        "Running FR3 lift_detach pose-stage sequence "
        f"at x={args.x:.3f}, y={args.y:.3f}, approach_z={args.approach_z:.3f}, "
        f"compression_z={compression_z:.3f}, lift_z={lift_z:.3f}."
    )

    move_stage(
        backend,
        "APPROACH",
        args.x,
        args.y,
        args.approach_z,
        args,
        telemetry=telemetry,
    )
    move_stage(
        backend,
        "COMPRESS",
        args.x,
        args.y,
        compression_z,
        args,
        telemetry=telemetry,
    )

    hold_stage("HOLD", args.hold_s, telemetry=telemetry)

    move_stage(
        backend,
        "LIFT",
        args.x,
        args.y,
        lift_z,
        args,
        telemetry=telemetry,
    )

    print("Completed FR3 lift_detach pose-stage sequence.")


def run_inclined_insertion(backend, args, telemetry=None):
    if args.insert_depth > MAX_VERTICAL_DEPTH_M:
        raise ValueError(
            "Refusing inclined_insertion with insert depth "
            f"{args.insert_depth:.3f} m; max is "
            f"{MAX_VERTICAL_DEPTH_M:.3f} m for this fake-hardware entry."
        )
    if not MIN_INCLINED_TILT_DEG <= args.tilt_deg <= MAX_INCLINED_TILT_DEG:
        raise ValueError(
            "Refusing inclined_insertion with tilt angle "
            f"{args.tilt_deg:.1f} deg; allowed range is "
            f"{MIN_INCLINED_TILT_DEG:.1f}-"
            f"{MAX_INCLINED_TILT_DEG:.1f} deg for this entry."
        )

    tilt_rad = math.radians(args.tilt_deg)
    tilted_quat = tilted_orientation(args)

    # Match the MuJoCo primitive: insert along the tilted tool direction and
    # shift the approach slightly opposite that direction so the inserted tip
    # stays close to the requested probe center.
    insertion_dx = -math.sin(tilt_rad) * args.insert_depth
    insertion_dz = -math.cos(tilt_rad) * args.insert_depth
    approach_x = args.x - insertion_dx
    insert_x = args.x + insertion_dx
    insert_z = args.approach_z + insertion_dz

    print_tooling_warning(args)
    print(
        "Running FR3 inclined_insertion pose-stage sequence "
        f"with tip_link={args.tip_link}, tilt={args.tilt_deg:.1f} deg, "
        f"approach=({approach_x:.3f}, {args.y:.3f}, {args.approach_z:.3f}), "
        f"insert=({insert_x:.3f}, {args.y:.3f}, {insert_z:.3f})."
    )

    move_stage(
        backend,
        "APPROACH",
        approach_x,
        args.y,
        args.approach_z,
        args,
        telemetry=telemetry,
    )
    move_stage(
        backend,
        "TILT",
        approach_x,
        args.y,
        args.approach_z,
        args,
        orientation=tilted_quat,
        telemetry=telemetry,
    )
    move_stage(
        backend,
        "INSERT",
        insert_x,
        args.y,
        insert_z,
        args,
        orientation=tilted_quat,
        telemetry=telemetry,
    )

    hold_stage("HOLD", args.hold_s, telemetry=telemetry)

    move_stage(
        backend,
        "RETRACT",
        approach_x,
        args.y,
        args.approach_z,
        args,
        orientation=tilted_quat,
        telemetry=telemetry,
    )
    move_stage(
        backend,
        "UNTILT",
        approach_x,
        args.y,
        args.approach_z,
        args,
        telemetry=telemetry,
    )

    print("Completed FR3 inclined_insertion pose-stage sequence.")


def run_selected_primitive(backend, args, telemetry=None):
    primitive = selected_primitive(args)
    if primitive == "approach_smoke":
        run_approach_smoke(backend, args, telemetry=telemetry)
    elif primitive == "trajectory_smoke":
        run_trajectory_smoke(backend, args, telemetry=telemetry)
    elif primitive == "micro_compression":
        run_micro_compression(backend, args, telemetry=telemetry)
    elif primitive == "horizontal_drag":
        run_horizontal_drag(backend, args, telemetry=telemetry)
    elif primitive == "lift_detach":
        run_lift_detach(backend, args, telemetry=telemetry)
    elif primitive == "inclined_insertion":
        run_inclined_insertion(backend, args, telemetry=telemetry)
    else:
        raise ValueError(f"Unsupported FR3 primitive: {primitive}")


def main(
    description=None,
    default_tip_link=FR3_TIP_LINK,
    default_approach_z=0.45,
    default_use_fake_hardware=True,
    allow_contact_primitives_default=True,
):
    args = parse_probe_args(
        description=description,
        default_tip_link=default_tip_link,
        default_approach_z=default_approach_z,
        default_use_fake_hardware=default_use_fake_hardware,
        allow_contact_primitives_default=allow_contact_primitives_default,
    )
    try:
        primitive = validate_primitive_safety(args)
    except ValueError as exc:
        print(f"ERROR: {exc}")
        raise SystemExit(2) from exc

    import rclpy

    rclpy.init()

    backend = build_backend(args)
    prepare_goal_orientation(backend, args)
    telemetry_logger = None
    if args.telemetry_log_path:
        if not hasattr(backend, "node"):
            raise RuntimeError(
                "Telemetry logging requires a backend-owned ROS node. "
                "Use --backend-mode move_group_action for now."
            )
        telemetry_logger = ProbeTelemetryLogger.from_args(
            node=backend.node,
            args=args,
            primitive=primitive,
        )

    trajectory_plotter = None
    if args.trajectory_plot_path:
        if not hasattr(backend, "node"):
            raise RuntimeError(
                "Trajectory plotting requires a backend-owned ROS node. "
                "Use --backend-mode move_group_action for now."
            )
        trajectory_plotter = ProbeTrajectoryPlotter.from_args(
            node=backend.node,
            args=args,
            primitive=primitive,
        )

    telemetry = None
    if telemetry_logger is not None or trajectory_plotter is not None:
        telemetry = ProbeRunObservers(
            telemetry=telemetry_logger,
            trajectory_plotter=trajectory_plotter,
        )
        telemetry.warmup()

    try:
        run_selected_primitive(backend, args, telemetry=telemetry)
    finally:
        if telemetry is not None:
            telemetry.close()

    time.sleep(1.0)

    # MoveItPy can segfault during shutdown in one-shot scripts.
    os._exit(0)
