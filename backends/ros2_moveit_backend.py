class ROS2MoveItBackend:
    """Minimal ROS2/MoveIt backend for moving FR3 probing TCP pose goals."""

    def __init__(
        self,
        planning_group="fr3_arm",
        base_frame="fr3_link0",
        tip_link="spatula_tip",
        controller_names=("fr3_arm_controller",),
        node_name="probing_moveit_backend",
        move_group_action="/move_action",
        backend_mode="auto",
        pipeline_id="move_group",
        planner_id="RRTConnectkConfigDefault",
        planning_time=5.0,
        position_tolerance=0.01,
        orientation_tolerance=0.2,
        max_velocity_scaling_factor=0.1,
        max_acceleration_scaling_factor=0.1,
        action_timeout_s=30.0,
    ):
        self.planning_group = planning_group
        self.base_frame = base_frame
        self.tip_link = tip_link
        self.controller_names = list(controller_names or [])
        self.backend_mode = backend_mode
        self.pipeline_id = pipeline_id
        self.planner_id = planner_id
        self.planning_time = float(planning_time)
        self.position_tolerance = float(position_tolerance)
        self.orientation_tolerance = float(orientation_tolerance)
        self.max_velocity_scaling_factor = float(max_velocity_scaling_factor)
        self.max_acceleration_scaling_factor = float(max_acceleration_scaling_factor)
        self.action_timeout_s = float(action_timeout_s)

        if backend_mode not in ("auto", "moveit_py", "move_group_action"):
            raise ValueError(
                "backend_mode must be 'auto', 'moveit_py', or "
                f"'move_group_action', got {backend_mode!r}."
            )

        if backend_mode == "move_group_action":
            self._init_move_group_action(node_name, move_group_action)
            return

        try:
            from moveit.planning import MoveItPy
        except ModuleNotFoundError as exc:
            if exc.name != "moveit" or backend_mode == "moveit_py":
                raise
            self._init_move_group_action(node_name, move_group_action)
        else:
            self._mode = "moveit_py"
            self.moveit = MoveItPy(node_name=node_name)
            self.arm = self.moveit.get_planning_component(planning_group)

    def _init_move_group_action(self, node_name, move_group_action):
        import rclpy
        from rclpy.action import ActionClient
        from moveit_msgs.action import MoveGroup

        self._mode = "move_group_action"
        self._move_group_action = move_group_action
        self.node = rclpy.create_node(node_name, use_global_arguments=False)
        self._move_group_client = ActionClient(
            self.node,
            MoveGroup,
            move_group_action,
        )
        if not self._move_group_client.wait_for_server(
            timeout_sec=self.action_timeout_s
        ):
            raise RuntimeError(
                f"MoveIt action server {move_group_action!r} is not available."
            )

    def move_to_pose(
        self,
        x_m,
        y_m,
        z_m,
        qx=0.0,
        qy=0.0,
        qz=0.0,
        qw=1.0,
        use_orientation_constraint=True,
        execute=True,
    ):
        if self._mode == "moveit_py":
            return self._move_to_pose_moveitpy(
                x_m,
                y_m,
                z_m,
                qx,
                qy,
                qz,
                qw,
                use_orientation_constraint,
                execute,
            )

        return self._move_to_pose_action(
            x_m,
            y_m,
            z_m,
            qx,
            qy,
            qz,
            qw,
            use_orientation_constraint,
            execute,
        )

    def _move_to_pose_moveitpy(
        self,
        x_m,
        y_m,
        z_m,
        qx,
        qy,
        qz,
        qw,
        use_orientation_constraint,
        execute,
    ):
        from geometry_msgs.msg import PoseStamped

        if not use_orientation_constraint:
            raise NotImplementedError(
                "Position-only goals are implemented for backend_mode="
                "move_group_action. Use --backend-mode move_group_action."
            )

        pose_goal = PoseStamped()
        pose_goal.header.frame_id = self.base_frame

        pose_goal.pose.orientation.x = float(qx)
        pose_goal.pose.orientation.y = float(qy)
        pose_goal.pose.orientation.z = float(qz)
        pose_goal.pose.orientation.w = float(qw)

        pose_goal.pose.position.x = float(x_m)
        pose_goal.pose.position.y = float(y_m)
        pose_goal.pose.position.z = float(z_m)

        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(
            pose_stamped_msg=pose_goal,
            pose_link=self.tip_link,
        )

        plan_result = self.arm.plan()
        if not plan_result:
            raise RuntimeError(
                "MoveIt failed to plan to pose "
                f"({x_m:.3f}, {y_m:.3f}, {z_m:.3f}) in {self.base_frame}."
            )

        if execute:
            self.moveit.execute(
                plan_result.trajectory,
                controllers=self.controller_names,
            )

        return plan_result

    def _move_to_pose_action(
        self,
        x_m,
        y_m,
        z_m,
        qx,
        qy,
        qz,
        qw,
        use_orientation_constraint,
        execute,
    ):
        import rclpy
        from geometry_msgs.msg import Pose
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import (
            Constraints,
            MoveItErrorCodes,
            OrientationConstraint,
            PositionConstraint,
        )
        from shape_msgs.msg import SolidPrimitive

        pose_goal = Pose()
        pose_goal.orientation.x = float(qx)
        pose_goal.orientation.y = float(qy)
        pose_goal.orientation.z = float(qz)
        pose_goal.orientation.w = float(qw)
        pose_goal.position.x = float(x_m)
        pose_goal.position.y = float(y_m)
        pose_goal.position.z = float(z_m)

        position_region = Pose()
        position_region.orientation.w = 1.0
        position_region.position = pose_goal.position

        position_bounds = SolidPrimitive()
        position_bounds.type = SolidPrimitive.SPHERE
        position_bounds.dimensions = [self.position_tolerance]

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.base_frame
        position_constraint.link_name = self.tip_link
        position_constraint.constraint_region.primitives = [position_bounds]
        position_constraint.constraint_region.primitive_poses = [position_region]
        position_constraint.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.name = "fr3_probe_position_goal"
        goal_constraints.position_constraints = [position_constraint]

        if use_orientation_constraint:
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = self.base_frame
            orientation_constraint.link_name = self.tip_link
            orientation_constraint.orientation = pose_goal.orientation
            orientation_constraint.absolute_x_axis_tolerance = (
                self.orientation_tolerance
            )
            orientation_constraint.absolute_y_axis_tolerance = (
                self.orientation_tolerance
            )
            orientation_constraint.absolute_z_axis_tolerance = (
                self.orientation_tolerance
            )
            orientation_constraint.parameterization = (
                OrientationConstraint.ROTATION_VECTOR
            )
            orientation_constraint.weight = 1.0
            goal_constraints.name = "fr3_probe_pose_goal"
            goal_constraints.orientation_constraints = [orientation_constraint]

        goal_msg = MoveGroup.Goal()
        goal_msg.request.workspace_parameters.header.frame_id = self.base_frame
        goal_msg.request.workspace_parameters.min_corner.x = -2.0
        goal_msg.request.workspace_parameters.min_corner.y = -2.0
        goal_msg.request.workspace_parameters.min_corner.z = -2.0
        goal_msg.request.workspace_parameters.max_corner.x = 2.0
        goal_msg.request.workspace_parameters.max_corner.y = 2.0
        goal_msg.request.workspace_parameters.max_corner.z = 2.0
        goal_msg.request.start_state.is_diff = True
        goal_msg.request.goal_constraints = [goal_constraints]
        goal_msg.request.pipeline_id = self.pipeline_id
        goal_msg.request.planner_id = self.planner_id
        goal_msg.request.group_name = self.planning_group
        goal_msg.request.num_planning_attempts = 1
        goal_msg.request.allowed_planning_time = self.planning_time
        goal_msg.request.max_velocity_scaling_factor = (
            self.max_velocity_scaling_factor
        )
        goal_msg.request.max_acceleration_scaling_factor = (
            self.max_acceleration_scaling_factor
        )
        goal_msg.planning_options.plan_only = not execute
        goal_msg.planning_options.replan = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True

        send_goal_future = self._move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(
            self.node,
            send_goal_future,
            timeout_sec=self.action_timeout_s,
        )
        if not send_goal_future.done():
            raise TimeoutError(
                f"Timed out sending goal to {self._move_group_action!r}."
            )

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("MoveIt rejected the pose goal.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.node,
            result_future,
            timeout_sec=self.action_timeout_s,
        )
        if not result_future.done():
            raise TimeoutError(
                f"Timed out waiting for {self._move_group_action!r} result."
            )

        result = result_future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            message = result.error_code.message or "no detailed MoveIt message"
            raise RuntimeError(
                "MoveIt failed to plan/execute pose "
                f"({x_m:.3f}, {y_m:.3f}, {z_m:.3f}) in {self.base_frame}: "
                f"{result.error_code.val} ({message})."
            )

        return result
