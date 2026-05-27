from __future__ import annotations

from dataclasses import dataclass

import commands2
import rev
import wpilib

import constants


@dataclass(frozen=True, slots=True)
class _AxisConfig:
    name: str
    motor_can_id: int
    motor_inverted: bool
    current_limit_amps: int
    open_loop_ramp_seconds: float
    startup_angle_degrees: float
    gear_ratio: float
    min_angle_degrees: float
    max_angle_degrees: float
    max_open_loop_output: float


class _SparkAxis:
    def __init__(self, axis_config: _AxisConfig) -> None:
        self._axis_config = axis_config
        self._position_factor = 360.0 / axis_config.gear_ratio
        self._velocity_factor = 6.0 / axis_config.gear_ratio
        self._last_command = 0.0

        self._motor = rev.SparkMax(
            axis_config.motor_can_id,
            rev.SparkLowLevel.MotorType.kBrushless,
        )

        motor_config = rev.SparkMaxConfig()
        motor_config.inverted(axis_config.motor_inverted)
        motor_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        motor_config.smartCurrentLimit(axis_config.current_limit_amps)
        motor_config.openLoopRampRate(axis_config.open_loop_ramp_seconds)
        motor_config.encoder.positionConversionFactor(self._position_factor)
        motor_config.encoder.velocityConversionFactor(self._velocity_factor)
        self._motor.configure(
            motor_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kNoPersistParameters,
        )

        self._encoder = self._motor.getEncoder()
        self._encoder.setPosition(axis_config.startup_angle_degrees)

    def get_position_degrees(self) -> float:
        return self._encoder.getPosition()

    def get_velocity_degrees_per_second(self) -> float:
        return self._encoder.getVelocity()

    def set_current_angle_degrees(self, angle_degrees: float) -> None:
        self._encoder.setPosition(angle_degrees)

    def at_lower_limit(self) -> bool:
        return self.get_position_degrees() <= self._axis_config.min_angle_degrees

    def at_upper_limit(self) -> bool:
        return self.get_position_degrees() >= self._axis_config.max_angle_degrees

    def apply_output(self, output: float) -> None:
        clamped_output = max(
            -self._axis_config.max_open_loop_output,
            min(self._axis_config.max_open_loop_output, output),
        )

        if self.at_lower_limit() and clamped_output < 0.0:
            clamped_output = 0.0
        if self.at_upper_limit() and clamped_output > 0.0:
            clamped_output = 0.0

        self._last_command = clamped_output
        self._motor.set(clamped_output)

    def stop(self) -> None:
        self._last_command = 0.0
        self._motor.stopMotor()

    def publish_telemetry(self) -> None:
        prefix = f"Turret/{self._axis_config.name}"
        wpilib.SmartDashboard.putNumber(
            f"{prefix}/AngleDegrees", self.get_position_degrees()
        )
        wpilib.SmartDashboard.putNumber(
            f"{prefix}/VelocityDegreesPerSecond",
            self.get_velocity_degrees_per_second(),
        )
        wpilib.SmartDashboard.putNumber(f"{prefix}/Command", self._last_command)
        wpilib.SmartDashboard.putNumber(
            f"{prefix}/AppliedOutput", self._motor.getAppliedOutput()
        )
        wpilib.SmartDashboard.putNumber(
            f"{prefix}/OutputCurrent", self._motor.getOutputCurrent()
        )
        wpilib.SmartDashboard.putBoolean(f"{prefix}/LowerLimit", self.at_lower_limit())
        wpilib.SmartDashboard.putBoolean(f"{prefix}/UpperLimit", self.at_upper_limit())


class TurretSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()
        commands2.CommandScheduler.getInstance().registerSubsystem(self)

        self._x_axis = _SparkAxis(
            _AxisConfig(
                name="X",
                motor_can_id=constants.Turret.X_MOTOR_CAN_ID,
                motor_inverted=constants.Turret.X_MOTOR_INVERTED,
                current_limit_amps=constants.Turret.X_CURRENT_LIMIT_AMPS,
                open_loop_ramp_seconds=constants.Turret.X_OPEN_LOOP_RAMP_SECONDS,
                startup_angle_degrees=constants.Turret.X_STARTUP_ANGLE_DEGREES,
                gear_ratio=constants.Turret.X_GEAR_RATIO,
                min_angle_degrees=constants.Turret.X_MIN_ANGLE_DEGREES,
                max_angle_degrees=constants.Turret.X_MAX_ANGLE_DEGREES,
                max_open_loop_output=constants.Turret.X_MAX_OPEN_LOOP_OUTPUT,
            )
        )

        self._y_axis = _SparkAxis(
            _AxisConfig(
                name="Y",
                motor_can_id=constants.Turret.Y_MOTOR_CAN_ID,
                motor_inverted=constants.Turret.Y_MOTOR_INVERTED,
                current_limit_amps=constants.Turret.Y_CURRENT_LIMIT_AMPS,
                open_loop_ramp_seconds=constants.Turret.Y_OPEN_LOOP_RAMP_SECONDS,
                startup_angle_degrees=constants.Turret.Y_STARTUP_ANGLE_DEGREES,
                gear_ratio=constants.Turret.Y_GEAR_RATIO,
                min_angle_degrees=constants.Turret.Y_MIN_ANGLE_DEGREES,
                max_angle_degrees=constants.Turret.Y_MAX_ANGLE_DEGREES,
                max_open_loop_output=constants.Turret.Y_MAX_OPEN_LOOP_OUTPUT,
            )
        )

    def get_x_position_degrees(self) -> float:
        return self._x_axis.get_position_degrees()

    def get_y_position_degrees(self) -> float:
        return self._y_axis.get_position_degrees()

    def set_x_angle_degrees(self, angle_degrees: float) -> None:
        self._x_axis.set_current_angle_degrees(angle_degrees)

    def set_y_angle_degrees(self, angle_degrees: float) -> None:
        self._y_axis.set_current_angle_degrees(angle_degrees)

    def apply_tracking_command(self, x_output: float, y_output: float) -> None:
        self._x_axis.apply_output(x_output)
        self._y_axis.apply_output(y_output)

    def stop(self) -> None:
        self._x_axis.stop()
        self._y_axis.stop()

    def periodic(self) -> None:
        self._x_axis.publish_telemetry()
        self._y_axis.publish_telemetry()
