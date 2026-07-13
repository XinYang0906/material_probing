from dataclasses import dataclass
from typing import Protocol, Sequence

from probing.sensors.ft_sensor import ForceTorqueSensor


@dataclass(frozen=True)
class CartesianContactStageConfig:
    """Constant-speed Cartesian stage request for future contact control."""

    stage_name: str
    direction_xyz: tuple[float, float, float]
    speed_m_s: float
    max_distance_m: float
    max_force_n: float
    sample_period_s: float = 0.002


@dataclass(frozen=True)
class CartesianContactStageResult:
    stage_name: str
    distance_m: float
    peak_force_n: float
    stopped_by: str


class CartesianServoBackend(Protocol):
    """Backend expected by a future constant-speed Cartesian controller."""

    def send_twist(
        self,
        linear_xyz_m_s: Sequence[float],
        angular_xyz_rad_s: Sequence[float],
    ):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError


class ConstantSpeedContactController:
    """Interface sketch for contact stages; not implemented for hardware yet."""

    def __init__(
        self,
        servo_backend: CartesianServoBackend,
        ft_sensor: ForceTorqueSensor,
    ):
        self.servo_backend = servo_backend
        self.ft_sensor = ft_sensor

    def execute_stage(
        self,
        config: CartesianContactStageConfig,
    ) -> CartesianContactStageResult:
        raise NotImplementedError(
            "Constant-speed Cartesian/contact control is planned but not "
            "implemented. Keep using MoveIt pose-stage fake-hardware tests."
        )
