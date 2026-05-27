from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True, slots=True)
class VisionTarget:
    has_target: bool = False
    tx_degrees: float = 0.0
    ty_degrees: float = 0.0
    tag_id: int | None = None
    timestamp_seconds: float = 0.0
    is_stale: bool = True
    pipeline_latency_ms: float = 0.0
    capture_latency_ms: float = 0.0

    @property
    def is_valid(self) -> bool:
        return self.has_target and not self.is_stale


@dataclass(frozen=True, slots=True)
class AxisTrackerOutput:
    output: float
    error_degrees: float
    within_tolerance: bool


@dataclass(frozen=True, slots=True)
class TrackerOutput:
    has_target: bool
    x_axis: AxisTrackerOutput
    y_axis: AxisTrackerOutput

    @property
    def within_tolerance(self) -> bool:
        return self.x_axis.within_tolerance and self.y_axis.within_tolerance


def _stopped_axis_output() -> AxisTrackerOutput:
    return AxisTrackerOutput(output=0.0, error_degrees=0.0, within_tolerance=False)


class _AxisAimController:
    def __init__(
        self,
        *,
        k_p: float,
        k_d: float,
        tolerance_degrees: float,
        minimum_command: float,
        maximum_command: float,
    ) -> None:
        self._k_p = k_p
        self._k_d = k_d
        self._tolerance_degrees = tolerance_degrees
        self._minimum_command = minimum_command
        self._maximum_command = maximum_command
        self._previous_error: float | None = None

    def reset(self) -> None:
        self._previous_error = None

    def calculate(self, error_degrees: float) -> AxisTrackerOutput:
        within_tolerance = abs(error_degrees) <= self._tolerance_degrees

        if within_tolerance:
            self._previous_error = error_degrees
            return AxisTrackerOutput(
                output=0.0,
                error_degrees=error_degrees,
                within_tolerance=True,
            )

        derivative_term = 0.0
        if self._previous_error is not None:
            derivative_term = self._k_d * (error_degrees - self._previous_error)

        output = (self._k_p * error_degrees) + derivative_term

        if abs(output) < self._minimum_command:
            output = math.copysign(self._minimum_command, error_degrees)

        output = max(-self._maximum_command, min(self._maximum_command, output))
        self._previous_error = error_degrees

        return AxisTrackerOutput(
            output=output,
            error_degrees=error_degrees,
            within_tolerance=False,
        )


class AprilTagAimController:
    def __init__(
        self,
        *,
        x_k_p: float,
        x_k_d: float,
        x_tolerance_degrees: float,
        x_minimum_command: float,
        x_maximum_command: float,
        y_k_p: float,
        y_k_d: float,
        y_tolerance_degrees: float,
        y_minimum_command: float,
        y_maximum_command: float,
    ) -> None:
        self._x_controller = _AxisAimController(
            k_p=x_k_p,
            k_d=x_k_d,
            tolerance_degrees=x_tolerance_degrees,
            minimum_command=x_minimum_command,
            maximum_command=x_maximum_command,
        )
        self._y_controller = _AxisAimController(
            k_p=y_k_p,
            k_d=y_k_d,
            tolerance_degrees=y_tolerance_degrees,
            minimum_command=y_minimum_command,
            maximum_command=y_maximum_command,
        )

    def reset(self) -> None:
        self._x_controller.reset()
        self._y_controller.reset()

    def calculate(self, target: VisionTarget) -> TrackerOutput:
        if not target.is_valid:
            self.reset()
            return TrackerOutput(
                has_target=False,
                x_axis=_stopped_axis_output(),
                y_axis=_stopped_axis_output(),
            )

        return TrackerOutput(
            has_target=True,
            x_axis=self._x_controller.calculate(target.tx_degrees),
            y_axis=self._y_controller.calculate(target.ty_degrees),
        )
