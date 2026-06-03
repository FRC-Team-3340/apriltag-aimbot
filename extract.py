from __future__ import annotations
import wpilib
    
import rev
from wpimath.controller import SimpleMotorFeedforwardRadians
import ntcore

class SingleMotorTurret(wpilib.TimedRobot):
    def robotInit(self):
        self.x_motor = rev.SparkMax(1, rev.SparkMax.MotorType.kBrushless)
        self.y_motor = rev.SparkMax(2, rev.SparkMax.MotorType.kBrushless)

        self.x_encoder = self.x_motor.getEncoder()
        self.y_encoder = self.y_motor.getEncoder()

        self.x_pid_controller = self.x_motor.getClosedLoopController()
        self.y_pid_controller = self.y_motor.getClosedLoopController()

        x_deg_per_rotation = 360.0 / 64.0
        y_deg_per_rotation = 360.0 / 14.2

        x_config = rev.SparkMaxConfig()
        y_config = rev.SparkMaxConfig()

        x_config.inverted(True)
        x_config.closedLoop.pid(p=0.01, i=0.0, d=0.001)
        x_config.closedLoop.allowedClosedLoopError(0.25)
        x_config.closedLoop.positionWrappingEnabled(True)
        x_config.closedLoop.positionWrappingInputRange(minInput=0.0, maxInput=85.0)
        x_config.encoder.positionConversionFactor(x_deg_per_rotation)

        y_config.inverted(True)
        y_config.closedLoop.pid(p=0.01, i=0.0, d=0.001)
        y_config.closedLoop.allowedClosedLoopError(0.25)
        y_config.closedLoop.positionWrappingEnabled(True)
        y_config.closedLoop.positionWrappingInputRange(minInput=0.0, maxInput=85.0)
        y_config.encoder.positionConversionFactor(y_deg_per_rotation)

        self.x_motor.configure(x_config, rev.ResetMode.kNoResetSafeParameters, rev.PersistMode.kPersistParameters)
        self.y_motor.configure(y_config, rev.ResetMode.kNoResetSafeParameters, rev.PersistMode.kPersistParameters)
        
        self.feedforward = SimpleMotorFeedforwardRadians(kS=0.07, kV=0.0)

        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self.limelight_table = self.nt_instance.getTable("limelight")

        self.x_virtual_target_angle = 0.0
        self.y_virtual_target_angle = 0.0

    def teleopInit(self):
        self.x_virtual_target_angle = self.x_encoder.getPosition()
        self.y_virtual_target_angle = self.y_encoder.getPosition()

    def teleopPeriodic(self) -> None:
        
        tx = self.limelight_table.getEntry("tx").getDouble(0.0)
        ty = self.limelight_table.getEntry("ty").getDouble(0.0)
        tv = self.limelight_table.getEntry("tv").getDouble(0.0)

        x_current_motor_angle = self.x_encoder.getPosition()
        y_current_motor_angle = self.y_encoder.getPosition()   

        if tv > 0.0:
            self.x_virtual_target_angle = x_current_motor_angle + tx
            self.y_virtual_target_angle = y_current_motor_angle + ty
        
        x_direction = 1.0 if self.x_virtual_target_angle > x_current_motor_angle else -1.0
        x_ff_voltage = self.feedforward.calculate(0.0) * x_direction
        y_direction = 1.0 if self.y_virtual_target_angle > y_current_motor_angle else -1.0
        y_ff_voltage = self.feedforward.calculate(0.0) * y_direction

        self.x_pid_controller.setReference(
            self.x_virtual_target_angle,
            rev.SparkMax.ControlType.kPosition,
            arbFeedforward=x_ff_voltage)
    

        self.y_pid_controller.setReference(
            self.y_virtual_target_angle,
            rev.SparkMax.ControlType.kPosition,
            arbFeedforward=y_ff_voltage)

if __name__ == "__main__":
    wpilib.run(SingleMotorTurret)
