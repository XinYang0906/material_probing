import time

from probing.sensors.ft_sensor import (
    CsvForceTorqueLogger,
    TipPoseSample,
    WrenchSample,
)


class ProbeTelemetryLogger:
    """ROS2 adapter that samples tip pose and latest wrench into a CSV file."""

    def __init__(
        self,
        node,
        output_path,
        primitive,
        base_frame,
        tip_link,
        wrench_topic="",
        robot_state_topic="",
        sample_period_s=0.1,
    ):
        self.node = node
        self.primitive = primitive
        self.base_frame = base_frame
        self.tip_link = tip_link
        self.wrench_topic = wrench_topic or ""
        self.robot_state_topic = robot_state_topic or ""
        self.sample_period_s = float(sample_period_s)
        self.csv = CsvForceTorqueLogger(output_path)
        self.latest_wrench = None
        self.latest_robot_state_pose = None
        self.latest_robot_state_wrench = None
        self.row_count = 0
        self.wrench_message_count = 0
        self.robot_state_message_count = 0

        self._init_tf()
        self._init_wrench_subscription()
        self._init_robot_state_subscription()

        print(
            "Telemetry logging enabled: "
            f"path={output_path}, wrench_topic={self.wrench_topic or '<none>'}, "
            f"robot_state_topic={self.robot_state_topic or '<none>'}."
        )

    @classmethod
    def from_args(cls, node, args, primitive):
        if not args.telemetry_log_path:
            return None

        return cls(
            node=node,
            output_path=args.telemetry_log_path,
            primitive=primitive,
            base_frame=args.base_frame,
            tip_link=args.tip_link,
            wrench_topic=args.wrench_topic,
            robot_state_topic=args.robot_state_topic,
            sample_period_s=args.telemetry_sample_period_s,
        )

    def _init_tf(self):
        try:
            from tf2_ros import Buffer, TransformListener
        except ModuleNotFoundError:
            self.tf_buffer = None
            self.tf_listener = None
            return

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self.node,
            spin_thread=False,
        )

    def _init_wrench_subscription(self):
        if not self.wrench_topic:
            return

        from geometry_msgs.msg import WrenchStamped
        from rclpy.qos import qos_profile_sensor_data

        self.node.create_subscription(
            WrenchStamped,
            self.wrench_topic,
            self._on_wrench,
            qos_profile_sensor_data,
        )

    def _init_robot_state_subscription(self):
        if not self.robot_state_topic:
            return

        try:
            from franka_msgs.msg import FrankaRobotState
        except ModuleNotFoundError:
            print(
                "WARNING: franka_msgs is unavailable; robot_state telemetry "
                "subscription is disabled."
            )
            return

        from rclpy.qos import qos_profile_sensor_data

        self.node.create_subscription(
            FrankaRobotState,
            self.robot_state_topic,
            self._on_robot_state,
            qos_profile_sensor_data,
        )

    def _now_s(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp_s(header, fallback_s):
        stamp = getattr(header, "stamp", None)
        if stamp is None:
            return fallback_s
        if stamp.sec == 0 and stamp.nanosec == 0:
            return fallback_s
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _on_wrench(self, msg):
        now_s = self._now_s()
        self.wrench_message_count += 1
        self.latest_wrench = self._wrench_from_msg(
            msg,
            source_topic=self.wrench_topic,
            fallback_s=now_s,
        )

    def _on_robot_state(self, msg):
        now_s = self._now_s()
        self.robot_state_message_count += 1

        self.latest_robot_state_wrench = self._wrench_from_msg(
            msg.o_f_ext_hat_k,
            source_topic=self.robot_state_topic + ".o_f_ext_hat_k",
            fallback_s=now_s,
        )
        self.latest_robot_state_pose = self._pose_from_msg(
            msg.o_t_ee,
            child_frame_id="fr3_hand_tcp",
            source="robot_state.o_t_ee",
            fallback_s=now_s,
        )

    def _wrench_from_msg(self, msg, source_topic, fallback_s):
        return WrenchSample(
            stamp_s=self._stamp_s(msg.header, fallback_s),
            frame_id=msg.header.frame_id,
            force_x_n=msg.wrench.force.x,
            force_y_n=msg.wrench.force.y,
            force_z_n=msg.wrench.force.z,
            torque_x_nm=msg.wrench.torque.x,
            torque_y_nm=msg.wrench.torque.y,
            torque_z_nm=msg.wrench.torque.z,
            source_topic=source_topic,
        )

    def _pose_from_msg(self, msg, child_frame_id, source, fallback_s):
        return TipPoseSample(
            stamp_s=self._stamp_s(msg.header, fallback_s),
            frame_id=msg.header.frame_id,
            child_frame_id=child_frame_id,
            x_m=msg.pose.position.x,
            y_m=msg.pose.position.y,
            z_m=msg.pose.position.z,
            qx=msg.pose.orientation.x,
            qy=msg.pose.orientation.y,
            qz=msg.pose.orientation.z,
            qw=msg.pose.orientation.w,
            source=source,
        )

    def spin_once(self, timeout_sec=0.0):
        import rclpy

        rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def warmup(self, duration_s=0.25):
        end_time = time.monotonic() + duration_s
        while time.monotonic() < end_time:
            self.spin_once(timeout_sec=0.02)

    def latest_tip_pose(self, command_pose=None):
        now_s = self._now_s()
        pose = self._lookup_tf_pose(now_s)
        if pose is not None:
            return pose

        if self.latest_robot_state_pose is not None:
            return self.latest_robot_state_pose

        if command_pose is None:
            return None

        x_m, y_m, z_m, qx, qy, qz, qw = command_pose
        return TipPoseSample(
            stamp_s=now_s,
            frame_id=self.base_frame,
            child_frame_id=self.tip_link,
            x_m=x_m,
            y_m=y_m,
            z_m=z_m,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
            source="commanded_pose_fallback",
        )

    def _lookup_tf_pose(self, now_s):
        if self.tf_buffer is None:
            return None

        try:
            from rclpy.time import Time

            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tip_link,
                Time(),
            )
        except Exception:
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return TipPoseSample(
            stamp_s=self._stamp_s(transform.header, now_s),
            frame_id=transform.header.frame_id,
            child_frame_id=transform.child_frame_id,
            x_m=translation.x,
            y_m=translation.y,
            z_m=translation.z,
            qx=rotation.x,
            qy=rotation.y,
            qz=rotation.z,
            qw=rotation.w,
            source="tf",
        )

    def latest_wrench_sample(self):
        if self.latest_wrench is not None:
            return self.latest_wrench
        return self.latest_robot_state_wrench

    def capture(self, stage, event="sample", command_pose=None):
        self.spin_once(timeout_sec=0.0)
        self.csv.write_sample(
            sample_time_s=self._now_s(),
            primitive=self.primitive,
            stage=stage,
            event=event,
            tip_pose=self.latest_tip_pose(command_pose=command_pose),
            command_pose=command_pose,
            wrench=self.latest_wrench_sample(),
        )
        self.row_count += 1

    def sleep(self, stage, duration_s):
        end_time = time.monotonic() + duration_s
        self.capture(stage, event="start")
        while time.monotonic() < end_time:
            remaining = end_time - time.monotonic()
            self.capture(stage, event="sample")
            self.spin_once(timeout_sec=min(self.sample_period_s, max(remaining, 0.0)))
        self.capture(stage, event="done")

    def close(self):
        self.csv.close()
        print(
            "Telemetry logging closed: "
            f"rows={self.row_count}, "
            f"wrench_samples={self.wrench_message_count}, "
            f"robot_state_samples={self.robot_state_message_count}."
        )
        if self.wrench_topic and self.wrench_message_count == 0:
            print(
                "WARNING: No wrench samples were received. The CSV still "
                "contains stage, command, and tip-pose rows; wrench columns "
                "are empty for this run."
            )
