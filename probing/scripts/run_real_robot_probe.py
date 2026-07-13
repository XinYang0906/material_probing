from probing.ros2_probe_runner import main


if __name__ == "__main__":
    main(
        description=(
            "Compatibility entry for FR3 probing against an already running "
            "MoveIt pipeline. Defaults are real-robot preflight settings: "
            "high, non-contact approach_smoke only. Contact-style primitives "
            "must be explicitly unlocked after the safety checklist."
        ),
        default_approach_z=0.55,
        default_use_fake_hardware=False,
        allow_contact_primitives_default=False,
    )
