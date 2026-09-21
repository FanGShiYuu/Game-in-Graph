"""Vehicle state, path-following kinematics, and fixed-parameter IDM."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Optional


@dataclass(frozen=True)
class VehicleState:
    """State used by the preview.

    ``progress`` is arc length from the route origin. Cartesian pose is derived
    from the scenario route, so path geometry and longitudinal control remain
    separate.
    """

    vehicle_id: int
    kind: str
    route_id: str
    progress: float
    speed: float
    acceleration: float = 0.0
    spawn_step: int = 0
    delay: float = 0.0

    def advance(
        self,
        acceleration: float,
        dt: float,
        speed_limit: float,
        min_acceleration: float,
        max_acceleration: float,
    ) -> "VehicleState":
        """Advance one source-aligned semi-implicit longitudinal step."""

        bounded_acc = min(max(acceleration, min_acceleration), max_acceleration)
        new_speed = min(speed_limit, max(0.0, self.speed + bounded_acc * dt))
        new_progress = self.progress + new_speed * dt
        delay_increment = max(0.0, 1.0 - new_speed / speed_limit) * dt
        return replace(
            self,
            progress=new_progress,
            speed=new_speed,
            acceleration=bounded_acc,
            delay=self.delay + delay_increment,
        )


@dataclass(frozen=True)
class IDMParameters:
    minimum_spacing: float = 2.0
    time_headway: float = 2.0
    maximum_acceleration: float = 2.0
    comfortable_acceleration: float = 2.0
    desired_speed: float = 10.0
    maximum_deceleration: float = -4.0
    exponent: float = 5.0


def idm_acceleration(
    state: VehicleState,
    front: Optional[VehicleState],
    gap: Optional[float],
    parameters: IDMParameters = IDMParameters(),
) -> float:
    """Compute the fixed-parameter IDM acceleration used for HDV realization."""

    if front is None or gap is None:
        return parameters.maximum_acceleration * (
            1.0 - (state.speed / parameters.desired_speed) ** parameters.exponent
        )
    safe_gap = max(gap, 1e-6)
    delta_v = state.speed - front.speed
    dynamic = state.speed * parameters.time_headway
    dynamic += state.speed * delta_v / (
        2.0
        * math.sqrt(
            parameters.maximum_acceleration * parameters.comfortable_acceleration
        )
    )
    desired_gap = parameters.minimum_spacing + max(0.0, dynamic)
    acceleration = parameters.maximum_acceleration * (
        1.0
        - (state.speed / parameters.desired_speed) ** parameters.exponent
        - (desired_gap / safe_gap) ** 2
    )
    return min(
        parameters.maximum_acceleration,
        max(parameters.maximum_deceleration, acceleration),
    )
