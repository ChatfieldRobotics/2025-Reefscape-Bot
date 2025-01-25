import math

import commands2
import wpimath
import wpilib

from commands2 import cmd
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

from constants import AutoConstants, DriveConstants, OIConstants
from subsystems.drivesubsystem import DriveSubsystem
from tagcentering import TagCentering

from commands2 import Command, RunCommand, InstantCommand
from commands2.button import JoystickButton


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems
        self.robotDrive = DriveSubsystem()

        # The driver's controller
        self.driverController = wpilib.XboxController(OIConstants.kDriverControllerPort)

        self.defaultDriveCommand = commands2.RunCommand(
            lambda: self.robotDrive.drive(
                -wpimath.applyDeadband(
                    self.driverController.getLeftY() * 0.3, OIConstants.kDriveDeadband
                ),
                -wpimath.applyDeadband(
                    self.driverController.getLeftX() * 0.3, OIConstants.kDriveDeadband
                ),
                -wpimath.applyDeadband(
                    self.driverController.getRightX() * 0.3, OIConstants.kDriveDeadband
                ),
                True,
            ),
            self.robotDrive,
        )

        # Configure the button bindings
        self.configureButtonBindings()

        # Configure default commands
        self.robotDrive.setDefaultCommand(
            # The left stick controls translation of the robot.
            # Turning is controlled by the X axis of the right stick.
            self.defaultDriveCommand
        )

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        setX = JoystickButton(
            self.driverController, wpilib.XboxController.Button.kA
        ).whileTrue(RunCommand(self.robotDrive.setX))
        zeroHeading = JoystickButton(
            self.driverController, wpilib.XboxController.Button.kB
        ).toggleOnTrue(RunCommand(self.robotDrive.zeroHeading))
        triggerTest = JoystickButton(
            self.driverController, wpilib.XboxController.Button.kY
        ).toggleOnTrue(
            TagCentering(
                self.robotDrive,
                DriveConstants.kDriveKinematics,
                self.defaultDriveCommand,
            )
        )

    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        swerveControllerCommand = commands2.SwerveControllerCommand(
            AutoConstants.exampleTrajectory,
            self.robotDrive.getPose,  # Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            # Position controllers
            PIDController,
            self.robotDrive.setModuleStates,
            (self.robotDrive,),
        )

        # Reset odometry to the starting pose of the trajectory.
        self.robotDrive.resetOdometry(AutoConstants.exampleTrajectory.initialPose())

        # Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(
            cmd.run(
                lambda: self.robotDrive.drive(0, 0, 0, False, False),
                self.robotDrive,
            )
        )
