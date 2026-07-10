def build_plan(backend, config):
    backend.reset_robot()

    home_q = backend.home_q()
    initial_tip = backend.tip_position()
    sample_top = backend.sample_top()

    approach_position = initial_tip.copy()
    approach_position[0] = config.probe_center_x_m
    approach_position[1] = config.probe_center_y_m
    approach_position[2] = sample_top + config.approach_gap_m

    compression_position = approach_position.copy()
    compression_position[2] = sample_top - config.compression_depth_m

    approach_q = backend.solve_position_ik(home_q, approach_position)
    compression_q = backend.solve_position_ik(approach_q, compression_position)

    return {
        "sample_top": sample_top,
        "summary": {
            "approach_position": approach_position,
            "compression_position": compression_position,
        },
        "stages": [
            ("APPROACH", home_q, approach_q, config.approach_duration_s),
            (
                "COMPRESS",
                approach_q,
                compression_q,
                config.compression_duration_s,
            ),
            ("HOLD", compression_q, compression_q, config.hold_duration_s),
            (
                "RETRACT",
                compression_q,
                approach_q,
                config.retract_duration_s,
            ),
            ("COMPLETE", approach_q, approach_q, config.complete_duration_s),
        ],
    }
