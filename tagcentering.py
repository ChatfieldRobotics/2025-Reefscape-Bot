import commands2
import wpilib
from commands2 import Command
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import Trajectory, TrajectoryGenerator
from wpilib import Field2d
import math

from constants import AutoConstants, DriveConstants
from subsystems.drivesubsystem import DriveSubsystem

class TagCentering(Command):
    def __init__(
        self,
        drive_subsystem,
        kinematics: SwerveDrive4Kinematics,
        default_command: commands2.RunCommand,
    ):
        super().__init__()

        self.subsystem: DriveSubsystem = drive_subsystem
        self.kinematics = kinematics
        self.default_command = default_command

        self.subsystem.setDefaultCommand(None)

        self.field = Field2d()
        wpilib.SmartDashboard.putData("Field", self.field)

    def initialize(self):
        start_pose = Pose2d(0.0, 0.0, Rotation2d(0))
        end_pose = Pose2d(0.0, 0.0, Rotation2d(math.pi / 2))
        interior_waypoints = [Translation2d(1.0, 1.0)]

        self.trajectory: Trajectory = TrajectoryGenerator.generateTrajectory(
            start_pose, interior_waypoints, end_pose, AutoConstants.config
        )

        self.field.getObject("Trajectory").setTrajectory(self.trajectory)

        self.swerveControllerCommand = commands2.SwerveControllerCommand(
            self.trajectory,
            self.subsystem.getPose,
            DriveConstants.kDriveKinematics,
            AutoConstants.PIDController,
            self.subsystem.setModuleStates,
            (self.subsystem,),
        )

        self.subsystem.resetOdometry(self.trajectory.initialPose())

    def execute(self):
        self.swerveControllerCommand.schedule()

    def isFinished(self):
        return self.swerveControllerCommand.isFinished()

    def end(self, interrupted: bool):
        self.swerveControllerCommand.cancel()

        self.subsystem.stop()

        self.subsystem.setDefaultCommand(self.default_command)
