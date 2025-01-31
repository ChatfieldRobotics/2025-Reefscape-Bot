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

from photonlibpy import PhotonCamera

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

        start_pose = Pose2d(0.0, 0.0, 0.0)
        end_pose = Pose2d(pose.x, pose.y, rotation)
        interior_waypoints = []

        self.trajectory: Trajectory = TrajectoryGenerator.generateTrajectory(
            start_pose, interior_waypoints, end_pose, AutoConstants.config
        )

        self.field.getObject("Trajectory").setTrajectory(self.trajectory)

        self.subsystem.resetOdometry(self.trajectory.initialPose())

        self.swerveControllerCommand = commands2.SwerveControllerCommand(
            self.trajectory,
            self.subsystem.getPose,
            DriveConstants.kDriveKinematics,
            AutoConstants.PIDController,
            self.subsystem.setModuleStates,
            (self.subsystem,),
        )

        self.swerveControllerCommand.schedule()

    def execute(self):
        pass

    def isFinished(self):
        return self.swerveControllerCommand.isFinished()

    def end(self, interrupted: bool):
        if not interrupted:
            self.swerveControllerCommand.cancel()

        self.subsystem.setDefaultCommand(self.default_command)
