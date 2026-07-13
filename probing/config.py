from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path("D:/2026livsurf_project")
DEFAULT_MODEL_PATH = (
    PROJECT_ROOT
    / "probing/models/franka_emika_panda/probing_scene.xml"
)


@dataclass
class ProbeConfig:
    # Main task parameters. These are the first values to change later.
    primitive: str = "micro_compression"
    tool_geometry: str = "flat"
    angle_deg: float = 15.0

    # Geometry and trajectory parameters.
    probe_center_x_m: float = 0.500
    probe_center_y_m: float = 0.000
    approach_gap_m: float = 0.020
    compression_depth_m: float = 0.0001
    insert_depth_m: float = 0.0001
    insert_distance_m: float = 0.0208
    drag_distance_m: float = -0.030
    lift_height_m: float = 0.040

    # Timing parameters.
    approach_duration_s: float = 3.0
    compression_duration_s: float = 3.0
    insert_duration_s: float = 3.0
    drag_duration_s: float = 4.0
    lift_duration_s: float = 4.0
    tilt_duration_s: float = 2.0
    hold_duration_s: float = 2.0
    retract_duration_s: float = 3.0
    untilt_duration_s: float = 2.0
    complete_duration_s: float = 1.0

    # Force parameters. Target force is intentionally left for later.
    max_force_n: float = 15.0

    # MuJoCo scene parameters.
    model_path: Path = DEFAULT_MODEL_PATH
    output_dir: Path = PROJECT_ROOT / "probing/data"

    # Panda hand parameters.
    gripper_position_m: float = 0.006
    gripper_control: float = 38.0
