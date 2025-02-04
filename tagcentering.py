import commands2
import pathplannerlib.controller
import wpilib
from commands2 import Command
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import Trajectory, TrajectoryGenerator
from wpilib import Field2d
import math

from constants import AutoConstants, DriveConstants
from subsystems.drivesubsystem import DriveSubsystem

from photonlibpy import PhotonCamera
from pathplannerlib.path import PathPlannerPath, PathConstraints, GoalEndState, Waypoint
from pathplannerlib.commands import FollowPathCommand
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants
from wpimath.geometry import Pose2d, Rotation2d


class TagCentering(Command):
    def __init__(
        self,
        drive_subsystem,
        kinematics: SwerveDrive4Kinematics,
        default_command: commands2.RunCommand,
        camera: PhotonCamera
    ):
        super().__init__()

        self.subsystem: DriveSubsystem = drive_subsystem
        self.kinematics = kinematics
        self.default_command = default_command
        self.camera = camera

        self.subsystem.setDefaultCommand(None)

        self.field = Field2d()
        wpilib.SmartDashboard.putData("Field", self.field)

    def initialize(self):
        self.result = self.camera.getLatestResult()

        if not self.result.hasTargets():
            self.end(interrupted=True)
            return 

        target = self.result.getBestTarget()

        pose = target.getBestCameraToTarget()

        rotation = -pose.rotation().angle + math.pi
        
        wpilib.SmartDashboard.putNumber("Angle: ", rotation)

        waypoints: list[Waypoint] = PathPlannerPath.waypointsFromPoses(
            self.subsystem.getPose(),
            Pose2d(Translation2d(pose.x, pose.y), Rotation2d(rotation))
        )

        constraints = PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared, AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared)

        path = PathPlannerPath(
            waypoints,
            constraints,
            None,
            GoalEndState(0.0, Rotation2d(rotation))
        )

        path.preventFlipping = True

        self.followPathCommand: FollowPathCommand = FollowPathCommand(      
            path,
            self.subsystem.getPose,
            self.subsystem.getRobotRelativeSpeeds,
            self.subsystem.driveRobotRelative,
            PPHolonomicDriveController(
                PIDConstants(1.0, 0, 0),
                PIDConstants(1.0, 0, 0)
            ),
            RobotConfig.fromGUISettings(),
            lambda : False,
            self.subsystem
        ).schedule()

    def execute(self):
        pass

    def isFinished(self):
        return self.followPathCommand.isFinished()

    def end(self, interrupted: bool):
        if not interrupted:
            self.followPathCommand.isFinished()

        self.subsystem.setDefaultCommand(self.default_command)
