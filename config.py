from dataclasses import dataclass


@dataclass
class MotorConfig:
    can_id: int
    deg_per_rotation: float
    p: float
    d: float
    inverted: bool
    wrap_min: float = 0.0
    wrap_max: float = 0.0
    ks: float = 0.0
    allowed_error: float = 0.25
    wrap_enabled: bool = True

X_AXIS = MotorConfig(
    can_id=1,
    deg_per_rotation=360.0 / 64.0,
    p=0.01,
    d=0.001,
    inverted=False,
    wrap_min=0.0,
    wrap_max=85.0,
    ks=0.07,
)

Y_AXIS = MotorConfig(
    can_id=2,
    deg_per_rotation=360.0 / 14.2,
    p=0.005,
    d=0.0005,
    inverted=True,
    wrap_min=-75.0,
    wrap_max=75.0,
    ks=0.07,
    wrap_enabled=False,
)