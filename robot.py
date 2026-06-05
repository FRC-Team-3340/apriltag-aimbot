from __future__ import annotations

import wpilib
import ntcore

from config import X_AXIS, Y_AXIS
from subsystems.turret import Turret


class TurretRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.turret = Turret(X_AXIS, Y_AXIS)
        self.turret.configure()

        self.limelight = ntcore.NetworkTableInstance.getDefault().getTable("limelight")

    def teleopInit(self):
        self.turret.initialize()

    def teleopPeriodic(self):
        tv = self.limelight.getEntry("tv").getDouble(0.0)

        if tv > 0.0:
            tx = self.limelight.getEntry("tx").getDouble(0.0)
            ty = self.limelight.getEntry("ty").getDouble(0.0)
            self.turret.track(ty, tx) # inverted because of how the limelight is mounted


if __name__ == "__main__":
    wpilib.run(TurretRobot)
