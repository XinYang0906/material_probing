import numpy as np

from probing.backends.mujoco_backend import rotation_y


def build_plan(backend, config):
    backend.reset_robot()

    home_q = backend.home_q()
    initial_tip = backend.tip_position()
    initial_rotation = backend.tip_rotation()
    sample_top = backend.sample_top()

    tilt_angle = np.deg2rad(config.angle_deg)
    tilted_rotation = rotation_y(tilt_angle) @ initial_rotation

    approach_position = initial_tip.copy()
    approach_position[2] = sample_top + config.approach_gap_m

    # Insertion follows the tilted tool direction, not the world Z axis.
    insertion_direction = tilted_rotation[:, 2].copy()
    if insertion_direction[2] > 0:
        insertion_direction = -insertion_direction

    # Start slightly opposite the insertion direction so that the final
    # inserted tip remains near the requested probe center.
    horizontal_shift = config.insert_distance_m * insertion_direction[:2]

    insert_position = (
        np.array([
            config.probe_center_x_m,
            config.probe_center_y_m,
            approach_position[2],
        ])
        + config.insert_distance_m * insertion_direction
    )

    approach_position[0] = config.probe_center_x_m - horizontal_shift[0]
    approach_position[1] = config.probe_center_y_m - horizontal_shift[1]

    approach_q = backend.solve_pose_ik(
        home_q, approach_position, initial_rotation
    )
    tilted_q = backend.solve_pose_ik(
        approach_q, approach_position, tilted_rotation
    )
    insert_q = backend.solve_pose_ik(
        tilted_q, insert_position, tilted_rotation
    )

    return {
        "sample_top": sample_top,
        "summary": {
            "approach_position": approach_position,
            "insert_position": insert_position,
            "insertion_direction": insertion_direction,
            "angle_deg": config.angle_deg,
        },
        "stages": [
            ("APPROACH", home_q, approach_q, config.approach_duration_s),
            ("TILT", approach_q, tilted_q, config.tilt_duration_s),
            ("INSERT", tilted_q, insert_q, config.insert_duration_s),
            ("HOLD", insert_q, insert_q, config.hold_duration_s),
            ("RETRACT", insert_q, tilted_q, config.retract_duration_s),
            ("UNTILT", tilted_q, approach_q, config.untilt_duration_s),
            ("COMPLETE", approach_q, approach_q, config.complete_duration_s),
        ],
    }
