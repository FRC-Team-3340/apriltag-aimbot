from __future__ import annotations

import commands2

from interfaces.aiming_axis import AimingMechanism


class StopTurretCommand(commands2.Command):
    def __init__(
        self,
        aiming_mechanism: AimingMechanism,
        requirements: tuple[commands2.Subsystem, ...],
    ) -> None:
        super().__init__()
        self._aiming_mechanism = aiming_mechanism
        self.addRequirements(*requirements)
        self.setName("StopTurretCommand")

    def initialize(self) -> None:
        self._aiming_mechanism.stop()

    def isFinished(self) -> bool:
        return True
