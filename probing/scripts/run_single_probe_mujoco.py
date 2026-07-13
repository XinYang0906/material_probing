"""No.41"""


import argparse
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from probing.backends.mujoco_backend import MujocoBackend, interpolate
from probing.config import ProbeConfig
from probing.primitives import (
    horizontal_drag,
    inclined_insertion,
    lift_detach,
    micro_compression,
)


PRIMITIVES = {
    "micro_compression": micro_compression.build_plan,
    "horizontal_drag": horizontal_drag.build_plan,
    "inclined_insertion": inclined_insertion.build_plan,
    "lift_detach": lift_detach.build_plan,
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run one probing primitive in MuJoCo."
    )
    parser.add_argument(
        "--primitive",
        choices=sorted(PRIMITIVES),
        default="micro_compression",
    )
    parser.add_argument("--tool", default="flat")
    parser.add_argument("--angle", type=float, default=15.0)
    parser.add_argument("--probe-x", type=float, default=0.500)
    parser.add_argument("--probe-y", type=float, default=0.000)
    parser.add_argument("--approach-gap", type=float, default=0.020)
    parser.add_argument("--compression-depth", type=float, default=0.0001)
    parser.add_argument("--insert-depth", type=float, default=0.0001)
    parser.add_argument("--insert-distance", type=float, default=0.0208)
    parser.add_argument("--drag-distance", type=float, default=-0.030)
    parser.add_argument("--lift-height", type=float, default=0.040)
    parser.add_argument("--max-force", type=float, default=15.0)
    parser.add_argument(
        "--model-path",
        default=None,
        help="Optional path to a MuJoCo scene XML.",
    )
    return parser.parse_args()


def make_config(args):
    config = ProbeConfig(
        primitive=args.primitive,
        tool_geometry=args.tool,
        angle_deg=args.angle,
        probe_center_x_m=args.probe_x,
        probe_center_y_m=args.probe_y,
        approach_gap_m=args.approach_gap,
        compression_depth_m=args.compression_depth,
        insert_depth_m=args.insert_depth,
        insert_distance_m=args.insert_distance,
        drag_distance_m=args.drag_distance,
        lift_height_m=args.lift_height,
        max_force_n=args.max_force,
    )
    if args.model_path:
        config.model_path = Path(args.model_path)
    return config


def output_path(config):
    filename = f"{config.primitive}_{config.tool_geometry}.csv"
    return config.output_dir / filename


def print_plan_summary(plan):
    print("Sample top:", round(plan["sample_top"], 4), "m")
    for name, value in plan["summary"].items():
        if hasattr(value, "round"):
            print(f"{name}:", np.round(value, 4))
        else:
            print(f"{name}:", value)


def run_plan(backend, config, plan):
    records = []
    force_limit_reached = False
    last_print_time = 0.0

    backend.reset_robot()
    sample_top = plan["sample_top"]
    stages = plan["stages"]

    with mujoco.viewer.launch_passive(backend.model, backend.data) as viewer:
        for stage_name, start_q, target_q, duration_s in stages:
            stage_start = time.time()

            while viewer.is_running():
                elapsed = time.time() - stage_start
                progress = elapsed / duration_s if duration_s > 0 else 1.0

                if force_limit_reached:
                    command_q = backend.data.qpos[:7].copy()
                    current_stage = "FORCE_LIMIT_STOP"
                else:
                    command_q = interpolate(start_q, target_q, progress)
                    current_stage = stage_name

                backend.data.ctrl[:7] = command_q
                backend.keep_gripper_closed()

                mujoco.mj_step(backend.model, backend.data)
                viewer.sync()

                contact_force, contact_count = backend.sample_contact_force()
                if (
                    contact_force > config.max_force_n
                    and not force_limit_reached
                ):
                    force_limit_reached = True
                    print(
                        "Force limit reached:",
                        round(contact_force, 2),
                        "N. Stopping motion.",
                    )

                tip_position = backend.tip_position()
                gap_mm = (tip_position[2] - sample_top) * 1000

                records.append(
                    [
                        backend.data.time,
                        config.primitive,
                        current_stage,
                        config.tool_geometry,
                        config.angle_deg,
                        tip_position[0],
                        tip_position[1],
                        tip_position[2],
                        gap_mm,
                        contact_count,
                        contact_force,
                    ]
                )

                if backend.data.time - last_print_time >= 0.5:
                    print(
                        "stage:",
                        current_stage,
                        "| tip:",
                        np.round(tip_position, 4),
                        "| gap:",
                        round(gap_mm, 2),
                        "mm",
                        "| contacts:",
                        contact_count,
                        "| force:",
                        round(contact_force, 2),
                        "N",
                    )
                    last_print_time = backend.data.time

                if progress >= 1.0 or force_limit_reached:
                    break

                time.sleep(backend.model.opt.timestep)

            if force_limit_reached:
                break

    return records


def main():
    args = parse_args()
    config = make_config(args)

    backend = MujocoBackend(config)
    plan = PRIMITIVES[config.primitive](backend, config)

    print("Primitive:", config.primitive)
    print("Tool geometry:", config.tool_geometry)
    print("Angle:", config.angle_deg, "deg")
    print_plan_summary(plan)

    records = run_plan(backend, config, plan)
    csv_path = output_path(config)
    backend.save_records(records, csv_path)
    print("Saved data to:", csv_path)


if __name__ == "__main__":
    main()
