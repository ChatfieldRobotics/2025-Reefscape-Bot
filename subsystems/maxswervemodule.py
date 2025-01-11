from rev import (
    ClosedLoopConfig,
    ClosedLoopSlot,
    SparkBase,
    SparkMax,
    SparkAbsoluteEncoder,
    SparkClosedLoopController,
    SparkRelativeEncoder,
)
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

from configs import MaxSwerveModule


class MAXSwerveModule:
    def __init__(
        self, drivingCANId: int, turningCANId: int, chassisAngularOffset: float
    ) -> None:
        """Constructs a MAXSwerveModule and configures the driving and turning motor,
        encoder, and PID controller. This configuration is specific to the REV
        MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
        Encoder.
        """

        self.drivingSparkMax: SparkMax = SparkMax(
            drivingCANId, SparkMax.MotorType.kBrushless
        )
        self.turningSparkMax: SparkMax = SparkMax(
            turningCANId, SparkMax.MotorType.kBrushless
        )

        self.drivingEncoder: SparkRelativeEncoder = self.drivingSparkMax.getEncoder()
        self.turningEncoder: SparkAbsoluteEncoder = (
            self.turningSparkMax.getAbsoluteEncoder()
        )

        self.drivingClosedLoopController: SparkClosedLoopController = (
            self.drivingSparkMax.getClosedLoopController()
        )
        self.turningClosedLoopController: SparkClosedLoopController = (
            self.turningSparkMax.getClosedLoopController()
        )

        # Apply the respective configurations to the SPARKS. Reset parameters before
        # applying the configuration to bring the SPARK to a known good state. Persist
        # the settings to the SPARK to avoid losing them on a power cycle.
        self.drivingSparkMax.configure(
            MaxSwerveModule.drivingConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters,
        )
        self.turningSparkMax.configure(
            MaxSwerveModule.turningConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters,
        )

        self.chassisAngularOffset: float = chassisAngularOffset
        self.desiredState: SwerveModuleState = SwerveModuleState(0.0, Rotation2d())
        self.desiredState.angle = Rotation2d(self.turningEncoder.getPosition())
        self.drivingEncoder.setPosition(0.0)

    def getState(self) -> SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        # Apply chassis angular offset to the encoder position to get the position
        # relative to the chassis.
        return SwerveModuleState(
            self.drivingEncoder.getVelocity(),
            Rotation2d(self.turningEncoder.getPosition() - self.chassisAngularOffset),
        )

    def getPosition(self) -> SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        # Apply chassis angular offset to the encoder position to get the position
        # relative to the chassis.
        return SwerveModulePosition(
            self.drivingEncoder.getPosition(),
            Rotation2d(self.turningEncoder.getPosition() - self.chassisAngularOffset),
        )

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.

        """
        # Apply chassis angular offset to the desired state.
        correctedDesiredState = SwerveModuleState()
        correctedDesiredState.speed = desiredState.speed
        correctedDesiredState.angle = desiredState.angle + Rotation2d(
            self.chassisAngularOffset
        )

        # Optimize the reference state to avoid spinning further than 90 degrees.
        optimizedDesiredState = SwerveModuleState.optimize(
            correctedDesiredState, Rotation2d(self.turningEncoder.getPosition())
        )

        # Command driving and turning SPARKS MAX towards their respective setpoints.
        self.drivingClosedLoopController.setReference(
            optimizedDesiredState.speed, SparkMax.ControlType.kVelocity
        )
        self.turningPIDController.setReference(
            optimizedDesiredState.angle.radians(), SparkMax.ControlType.kPosition
        )

        self.desiredState = desiredState

    def resetEncoders(self) -> None:
        """
        Zeroes all the SwerveModule encoders.
        """
        self.drivingEncoder.setPosition(0)
