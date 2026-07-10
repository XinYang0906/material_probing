def build_plan(backend, config):
    backend.reset_robot()

    home_q = backend.home_q()
    initial_tip = backend.tip_position()
    sample_top = backend.sample_top()

    approach_position = initial_tip.copy()
    approach_position[0] = config.probe_center_x_m
    approach_position[1] = config.probe_center_y_m
    approach_position[2] = sample_top + config.approach_gap_m

    insert_position = approach_position.copy()
    insert_position[2] = sample_top - config.insert_depth_m

    drag_position = insert_position.copy()
    drag_position[0] += config.drag_distance_m

    retract_position = drag_position.copy()
    retract_position[2] = sample_top + config.approach_gap_m

    approach_q = backend.solve_position_ik(home_q, approach_position)
    insert_q = backend.solve_position_ik(approach_q, insert_position)
    drag_q = backend.solve_position_ik(insert_q, drag_position)
    retract_q = backend.solve_position_ik(drag_q, retract_position)

    return {
        "sample_top": sample_top,
        "summary": {
            "approach_position": approach_position,
            "insert_position": insert_position,
            "drag_position": drag_position,
            "retract_position": retract_position,
        },
        "stages": [
            ("APPROACH", home_q, approach_q, config.approach_duration_s),
            ("INSERT", approach_q, insert_q, config.insert_duration_s),
            ("DRAG", insert_q, drag_q, config.drag_duration_s),
            ("RETRACT", drag_q, retract_q, config.retract_duration_s),
            ("COMPLETE", retract_q, retract_q, config.complete_duration_s),
        ],
    }
