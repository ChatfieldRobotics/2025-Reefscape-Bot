from rev import SparkMaxConfig
from rev import ClosedLoopConfig
from rev import SparkBaseConfig

from constants import ModuleConstants

import math


class MaxSwerveModule:
    drivingConfig: SparkMaxConfig = SparkMaxConfig()
    turningConfig: SparkMaxConfig = SparkMaxConfig()

    # Use module constants to calculate conversion factors and feed forward gain
    drivingFactor: float = (
        ModuleConstants.kWheelDiameterMeters
        * math.pi
        / ModuleConstants.kDrivingMotorReduction
    )
    turningFactor: float = 2 * math.pi
    drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps

    drivingConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(50)
    drivingConfig.encoder.positionConversionFactor(
        drivingFactor
    ).velocityConversionFactor(drivingFactor / 60.0)
    drivingConfig.closedLoop.setFeedbackSensor(
        ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
    ).pid(0.04, 0, 0).velocityFF(drivingVelocityFeedForward).outputRange(-1, 1)

    turningConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(20)
    turningConfig.absoluteEncoder.inverted(True).positionConversionFactor(
        turningFactor
    ).velocityConversionFactor(turningFactor / 60.0)
    turningConfig.closedLoop.setFeedbackSensor(
        ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder
    ).pid(1, 0, 0).outputRange(-1, 1).positionWrappingEnabled(
        True
    ).positionWrappingInputRange(
        0, turningFactor
    )
