# redundant declaration of classes
# given the roboRIO's memory constraints, it might be better 
# to declare this within the classes you already have
# e.g. in robotcontainer.py
# instead of a separate class


class OperatorInterface:
    DRIVER_CONTROLLER_PORT = 0
    TRACK_BUTTON = 1


class Turret:
    X_MOTOR_CAN_ID = 2
    X_MOTOR_INVERTED = False
    X_CURRENT_LIMIT_AMPS = 30
    X_OPEN_LOOP_RAMP_SECONDS = 0.15
    X_STARTUP_ANGLE_DEGREES = 0.0
    X_GEAR_RATIO = 48.0
    X_MIN_ANGLE_DEGREES = -85.0
    X_MAX_ANGLE_DEGREES = 85.0
    X_MAX_OPEN_LOOP_OUTPUT = 0.35

    Y_MOTOR_CAN_ID = 1
    Y_MOTOR_INVERTED = False
    Y_CURRENT_LIMIT_AMPS = 30
    Y_OPEN_LOOP_RAMP_SECONDS = 0.15
    Y_STARTUP_ANGLE_DEGREES = 0.0
    Y_GEAR_RATIO = 64.0
    Y_MIN_ANGLE_DEGREES = -85.0
    Y_MAX_ANGLE_DEGREES = 85.0
    Y_MAX_OPEN_LOOP_OUTPUT = 0.35


class AimController:
    X_KP = 0.02
    X_KD = 0.0
    X_ERROR_TOLERANCE_DEGREES = 1.5
    X_MIN_COMMAND = 0.07
    X_MAX_COMMAND = 0.35

    Y_KP = 0.02
    Y_KD = 0.0
    Y_ERROR_TOLERANCE_DEGREES = 1.5
    Y_MIN_COMMAND = 0.07
    Y_MAX_COMMAND = 0.35

# overriding the limelight class from limelightlib, maybe?
class Limelight:
    TABLE_NAME = "limelight"
    PIPELINE = 0
    TARGET_VALID_KEY = "tv"
    HORIZONTAL_OFFSET_KEY = "tx"
    VERTICAL_OFFSET_KEY = "ty"
    TARGET_ID_KEY = "tid"
    PIPELINE_LATENCY_KEY = "tl"
    CAPTURE_LATENCY_KEY = "cl"
    HEARTBEAT_KEY = "hb"
    STALE_TIMEOUT_SECONDS = 0.25


# if you want to have a separate constants file
# you could use a dictionary instead of a class
# as dictionary keys are hashable (and may run a bit faster)

# xoxo, ryan (mit '29)