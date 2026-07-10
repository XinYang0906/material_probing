from probing.ros2_probe_runner import main


if __name__ == "__main__":
    main(
        description=(
            "Run one FR3 probing primitive through an already running "
            "franka_fr3_moveit_config MoveIt pipeline."
        )
    )
