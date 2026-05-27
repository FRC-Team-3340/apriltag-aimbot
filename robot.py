from __future__ import annotations

import typing

import commands2
import wpilib

from robotcontainer import RobotContainer


class AprilTagAimbotRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        self.container = RobotContainer()
        self.autonomous_command: typing.Optional[commands2.Command] = None
        wpilib.SmartDashboard.putData(
            "CommandScheduler", commands2.CommandScheduler.getInstance()
        )

    def disabledInit(self) -> None:
        self.container.stop_all_motion()

    def autonomousInit(self) -> None:
        self.autonomous_command = self.container.get_autonomous_command()
        if self.autonomous_command is not None:
            self.autonomous_command.schedule()

    def teleopInit(self) -> None:
        if self.autonomous_command is not None:
            self.autonomous_command.cancel()
            self.autonomous_command = None

    def testInit(self) -> None:
        commands2.CommandScheduler.getInstance().cancelAll()
        self.container.stop_all_motion()


if __name__ == "__main__":
    wpilib.run(AprilTagAimbotRobot)
