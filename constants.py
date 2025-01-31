# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

"""
The constants module is a convenience place for teams to hold robot-wide
numerical or boolean constants. Don't use this for any other purpose!
"""

import math

from wpimath import units
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfile

from wpimath.controller import PIDController, ProfiledPIDControllerRadians
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import (
    TrajectoryConfig,
    TrajectoryGenerator,
    TrapezoidProfileRadians,
)
from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)


class NeoMotorConstants:
    kFreeSpeedRpm = 5676


class DriveConstants:
    # Driving Parameters - Note that these are not the maximum capable speeds of
    # the robot, rather the allowed maximum speeds
    kMaxSpeedMetersPerSecond = 4.8
    kMaxAngularSpeed = math.tau  # radians per second

    # Chassis configuration
    kTrackWidth = units.inchesToMeters(24.5)
    # Distance between centers of right and left wheels on robot
    kWheelBase = units.inchesToMeters(24.5)

    # Distance between front and back wheels on robot
    kDriveKinematics = SwerveDrive4Kinematics(
        Translation2d(kWheelBase / 2, kTrackWidth / 2),
        Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
    )

    # Angular offsets of the modules relative to the chassis in radians
    kFrontLeftChassisAngularOffset = -math.pi / 2
    kFrontRightChassisAngularOffset = 0
    kBackLeftChassisAngularOffset = math.pi
    kBackRightChassisAngularOffset = math.pi / 2

    # SPARK MAX CAN IDs
    kFrontLeftDrivingCanId = 2
    kRearLeftDrivingCanId = 4
    kFrontRightDrivingCanId = 8
    kRearRightDrivingCanId = 6

    kFrontLeftTurningCanId = 1
    kRearLeftTurningCanId = 3
    kFrontRightTurningCanId = 7
    kRearRightTurningCanId = 5

    kGyroReversed = False


class ModuleConstants:
    # The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    # This changes the drive speed of the module (a pinion gear with more teeth will result in a
    # robot that drives faster).
    kDrivingMotorPinionTeeth = 14

    # Calculations required for driving motor conversion factors and feed forward
    kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60
    kWheelDiameterMeters = 0.0762
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi
    # 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
    kDriveWheelFreeSpeedRps = (
        kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters
    ) / kDrivingMotorReduction


class OIConstants:
    kDriverControllerPort = 0
    kDriveDeadband = 0.05


class AutoConstants:
    kMaxSpeedMetersPerSecond = 3.0
    kMaxAccelerationMetersPerSecondSquared = 3.0
    kMaxAngularSpeedRadiansPerSecond = 4.0
    kMaxAngularSpeedRadiansPerSecondSquared = 2.5

    config = TrajectoryConfig(
        kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared
    )
    config.setKinematics(DriveConstants.kDriveKinematics)

    exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        Pose2d(0, 0, Rotation2d(0)),
        [Translation2d(1, 1), Translation2d(2, -1)],
        Pose2d(3, 0, Rotation2d(0)),
        config,
    )

    kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
    )

    kPXController = PIDController(0.25, 0.025, 0.05)
    kPYController = PIDController(0.25, 0.025, 0.05)

    kPThetaController = ProfiledPIDControllerRadians(
        2.0, 0.05, 0.002, kThetaControllerConstraints
    )
    kPThetaController.enableContinuousInput(-math.pi, math.pi)

    PIDController = HolonomicDriveController(
        kPXController, kPYController, kPThetaController
    )
