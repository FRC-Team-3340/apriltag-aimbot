from __future__ import annotations

from typing import Protocol


class AimingMechanism(Protocol):
    def apply_tracking_command(self, x_output: float, y_output: float) -> None:
        ...

    def stop(self) -> None:
        ...

    def get_x_position_degrees(self) -> float:
        ...

    def get_y_position_degrees(self) -> float:
        ...
