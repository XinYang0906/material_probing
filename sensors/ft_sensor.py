import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol


@dataclass(frozen=True)
class WrenchSample:
    stamp_s: float
    frame_id: str
    force_x_n: float
    force_y_n: float
    force_z_n: float
    torque_x_nm: float
    torque_y_nm: float
    torque_z_nm: float
    source_topic: str = ""

    @property
    def force_norm_n(self):
        return (
            self.force_x_n**2
            + self.force_y_n**2
            + self.force_z_n**2
        ) ** 0.5


@dataclass(frozen=True)
class TipPoseSample:
    stamp_s: float
    frame_id: str
    child_frame_id: str
    x_m: float
    y_m: float
    z_m: float
    qx: float
    qy: float
    qz: float
    qw: float
    source: str


class ForceTorqueSensor(Protocol):
    """Interface for future wrist/tool F/T sources."""

    def read_wrench(self) -> WrenchSample:
        raise NotImplementedError


class CsvForceTorqueLogger:
    """CSV sink for probing telemetry captured during pose/contact stages."""

    FIELDNAMES = (
        "time_s",
        "primitive",
        "stage",
        "event",
        "pose_source",
        "tip_frame_id",
        "tip_child_frame_id",
        "tip_x_m",
        "tip_y_m",
        "tip_z_m",
        "tip_qx",
        "tip_qy",
        "tip_qz",
        "tip_qw",
        "command_x_m",
        "command_y_m",
        "command_z_m",
        "command_qx",
        "command_qy",
        "command_qz",
        "command_qw",
        "wrench_source_topic",
        "wrench_stamp_s",
        "frame_id",
        "force_x_n",
        "force_y_n",
        "force_z_n",
        "force_norm_n",
        "torque_x_nm",
        "torque_y_nm",
        "torque_z_nm",
    )

    def __init__(self, output_path):
        self.output_path = Path(output_path)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.output_path.open("w", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=self.FIELDNAMES)
        self._writer.writeheader()

    def write(self, stage, sample: WrenchSample):
        self.write_sample(stage=stage, wrench=sample)

    def write_sample(
        self,
        stage,
        sample_time_s=None,
        primitive="",
        event="sample",
        tip_pose: TipPoseSample | None = None,
        command_pose=None,
        wrench: WrenchSample | None = None,
    ):
        command_pose = command_pose or (None, None, None, None, None, None, None)

        self._writer.writerow(
            {
                "time_s": sample_time_s,
                "primitive": primitive,
                "stage": stage,
                "event": event,
                "pose_source": tip_pose.source if tip_pose else "",
                "tip_frame_id": tip_pose.frame_id if tip_pose else "",
                "tip_child_frame_id": tip_pose.child_frame_id if tip_pose else "",
                "tip_x_m": tip_pose.x_m if tip_pose else None,
                "tip_y_m": tip_pose.y_m if tip_pose else None,
                "tip_z_m": tip_pose.z_m if tip_pose else None,
                "tip_qx": tip_pose.qx if tip_pose else None,
                "tip_qy": tip_pose.qy if tip_pose else None,
                "tip_qz": tip_pose.qz if tip_pose else None,
                "tip_qw": tip_pose.qw if tip_pose else None,
                "command_x_m": command_pose[0],
                "command_y_m": command_pose[1],
                "command_z_m": command_pose[2],
                "command_qx": command_pose[3],
                "command_qy": command_pose[4],
                "command_qz": command_pose[5],
                "command_qw": command_pose[6],
                "wrench_source_topic": wrench.source_topic if wrench else "",
                "wrench_stamp_s": wrench.stamp_s if wrench else None,
                "frame_id": wrench.frame_id if wrench else "",
                "force_x_n": wrench.force_x_n if wrench else None,
                "force_y_n": wrench.force_y_n if wrench else None,
                "force_z_n": wrench.force_z_n if wrench else None,
                "force_norm_n": wrench.force_norm_n if wrench else None,
                "torque_x_nm": wrench.torque_x_nm if wrench else None,
                "torque_y_nm": wrench.torque_y_nm if wrench else None,
                "torque_z_nm": wrench.torque_z_nm if wrench else None,
            }
        )
        self._file.flush()

    def close(self):
        self._file.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        self.close()
