from __future__ import annotations

import commands2
import wpilib

from interfaces.aiming_axis import AimingMechanism
from subsystems.limelight_vision import LimelightVisionSubsystem
from tracking.apriltag_tracker import AprilTagAimController


class TrackAprilTagCommand(commands2.Command):
    def __init__(
        self,
        aiming_mechanism: AimingMechanism,
        vision: LimelightVisionSubsystem,
        tracker: AprilTagAimController,
        requirements: tuple[commands2.Subsystem, ...],
    ) -> None:
        super().__init__()
        self._aiming_mechanism = aiming_mechanism
        self._vision = vision
        self._tracker = tracker
        self.addRequirements(*requirements)
        self.setName("TrackAprilTagCommand")

    def initialize(self) -> None:
        self._tracker.reset()

    def execute(self) -> None:
        target = self._vision.get_latest_target()
        tracker_output = self._tracker.calculate(target)

        if tracker_output.has_target:
            self._aiming_mechanism.apply_tracking_command(
                tracker_output.x_axis.output,
                tracker_output.y_axis.output,
            )
        else:
            self._aiming_mechanism.stop()

        wpilib.SmartDashboard.putNumber(
            "Tracker/X/ErrorDegrees", tracker_output.x_axis.error_degrees
        )
        wpilib.SmartDashboard.putNumber(
            "Tracker/Y/ErrorDegrees", tracker_output.y_axis.error_degrees
        )
        wpilib.SmartDashboard.putNumber(
            "Tracker/X/CommandedOutput", tracker_output.x_axis.output
        )
        wpilib.SmartDashboard.putNumber(
            "Tracker/Y/CommandedOutput", tracker_output.y_axis.output
        )
        wpilib.SmartDashboard.putBoolean("Tracker/HasTarget", tracker_output.has_target)
        wpilib.SmartDashboard.putBoolean(
            "Tracker/X/WithinTolerance", tracker_output.x_axis.within_tolerance
        )
        wpilib.SmartDashboard.putBoolean(
            "Tracker/Y/WithinTolerance", tracker_output.y_axis.within_tolerance
        )
        wpilib.SmartDashboard.putBoolean(
            "Tracker/WithinTolerance", tracker_output.within_tolerance
        )

    def end(self, interrupted: bool) -> None:
        self._tracker.reset()
        self._aiming_mechanism.stop()
        wpilib.SmartDashboard.putBoolean("Tracker/Interrupted", interrupted)

    def isFinished(self) -> bool:
        return False
