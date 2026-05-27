from __future__ import annotations

import commands2
from commands2.button import Trigger
import wpilib

import constants
from commands.track_apriltag import TrackAprilTagCommand
from subsystems.limelight_vision import LimelightVisionSubsystem
from subsystems.turret import TurretSubsystem
from tracking.apriltag_tracker import AprilTagAimController


class RobotContainer:
    def __init__(self) -> None:
        self.driver_controller = wpilib.Joystick(
            constants.OperatorInterface.DRIVER_CONTROLLER_PORT
        )

        self.turret = TurretSubsystem()
        self.vision = LimelightVisionSubsystem()
        self.aim_controller = AprilTagAimController(
            x_k_p=constants.AimController.X_KP,
            x_k_d=constants.AimController.X_KD,
            x_tolerance_degrees=constants.AimController.X_ERROR_TOLERANCE_DEGREES,
            x_minimum_command=constants.AimController.X_MIN_COMMAND,
            x_maximum_command=constants.AimController.X_MAX_COMMAND,
            y_k_p=constants.AimController.Y_KP,
            y_k_d=constants.AimController.Y_KD,
            y_tolerance_degrees=constants.AimController.Y_ERROR_TOLERANCE_DEGREES,
            y_minimum_command=constants.AimController.Y_MIN_COMMAND,
            y_maximum_command=constants.AimController.Y_MAX_COMMAND,
        )

        self.track_apriltag_command = TrackAprilTagCommand(
            aiming_mechanism=self.turret,
            vision=self.vision,
            tracker=self.aim_controller,
            requirements=(self.turret, self.vision),
        )

        self._track_trigger = Trigger(
            lambda: self.driver_controller.getRawButton(
                constants.OperatorInterface.TRACK_BUTTON
            )
        )
        self._configure_bindings()

        wpilib.SmartDashboard.putString("Aiming/Mode", "PanTilt")
        wpilib.SmartDashboard.putNumber(
            "Aiming/TrackButton", constants.OperatorInterface.TRACK_BUTTON
        )
        wpilib.SmartDashboard.putNumber("Aiming/XMotorCanId", constants.Turret.X_MOTOR_CAN_ID)
        wpilib.SmartDashboard.putNumber("Aiming/YMotorCanId", constants.Turret.Y_MOTOR_CAN_ID)

    def _configure_bindings(self) -> None:
        self._track_trigger.whileTrue(self.track_apriltag_command)

    def get_autonomous_command(self) -> commands2.Command | None:
        return None

    def stop_all_motion(self) -> None:
        self.aim_controller.reset()
        self.turret.stop()
