from __future__ import annotations

import rev
from wpimath.controller import SimpleMotorFeedforwardRadians

from config import MotorConfig


class TurretAxis:
    def __init__(self, cfg: MotorConfig):
        self._cfg = cfg
        self._motor = rev.SparkMax(cfg.can_id, rev.SparkMax.MotorType.kBrushless)
        self._encoder = self._motor.getEncoder()
        self._pid = self._motor.getClosedLoopController()
        self._ff = SimpleMotorFeedforwardRadians(kS=cfg.ks, kV=0.0)
        self._target: float = 0.0

    def configure(self) -> None:
        config = rev.SparkMaxConfig()
        config.inverted(self._cfg.inverted)
        config.encoder.positionConversionFactor(self._cfg.deg_per_rotation)
        config.closedLoop.pid(p=self._cfg.p, i=0.0, d=self._cfg.d)
        config.closedLoop.allowedClosedLoopError(self._cfg.allowed_error)
        config.closedLoop.positionWrappingEnabled(True)
        config.closedLoop.positionWrappingInputRange(
            minInput=self._cfg.wrap_min, maxInput=self._cfg.wrap_max
        )
        self._motor.configure(
            config,
            rev.ResetMode.kNoResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

    def initialize(self) -> None:
        self._target = self._encoder.getPosition()

    def track(self, offset: float) -> None:
        current = self._encoder.getPosition()
        self._target = current + offset
        direction = 1.0 if self._target > current else -1.0
        ff_voltage = self._ff.calculate(0.0) * direction
        self._pid.setReference(
            self._target,
            rev.SparkMax.ControlType.kPosition,
            arbFeedforward=ff_voltage,
        )


class Turret:
    def __init__(self, x_cfg: MotorConfig, y_cfg: MotorConfig):
        self.x = TurretAxis(x_cfg)
        self.y = TurretAxis(y_cfg)

    def configure(self) -> None:
        self.x.configure()
        self.y.configure()

    def initialize(self) -> None:
        self.x.initialize()
        self.y.initialize()

    def track(self, tx: float, ty: float) -> None:
        self.x.track(tx)
        self.y.track(ty)