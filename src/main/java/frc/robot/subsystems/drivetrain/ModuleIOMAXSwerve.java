package frc.robot.subsystems.drivetrain;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class ModuleIOMAXSwerve implements ModuleIO {
    private final SparkMax driveSparkMax;
    private final SparkMax turnSparkMax;

    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder turnEncoder;

    private final SparkClosedLoopController drivePIDController;
    private final SparkClosedLoopController turnPIDController;

    public ModuleIOMAXSwerve(int index) {
        switch (index) {
            case 0:
                driveSparkMax = new SparkMax(DriveConstants.kFrontLeftDriveCanId, MotorType.kBrushless);
                turnSparkMax = new SparkMax(DriveConstants.kFrontLeftTurningCanId, MotorType.kBrushless);
                break;
            case 1:
                driveSparkMax = new SparkMax(DriveConstants.kFrontRightDriveCanId, MotorType.kBrushless);
                turnSparkMax = new SparkMax(DriveConstants.kFrontRightTurningCanId, MotorType.kBrushless);
                break;
            case 2:
                driveSparkMax = new SparkMax(DriveConstants.kBackLeftDriveCanId, MotorType.kBrushless);
                turnSparkMax = new SparkMax(DriveConstants.kBackLeftTurningCanId, MotorType.kBrushless);
                break;
            case 3:
                driveSparkMax = new SparkMax(DriveConstants.kBackRightDriveCanId, MotorType.kBrushless);
                turnSparkMax = new SparkMax(DriveConstants.kBackRightTurningCanId, MotorType.kBrushless);
                break;
            default:
                throw new RuntimeException("Invalid module index: " + index);
        }

        SparkMaxConfig driveConfig = new SparkMaxConfig();
        SparkMaxConfig turnConfig = new SparkMaxConfig();

        driveConfig
            .idleMode(ModuleConstants.kDrivingMotorIdleMode)
            .smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
        turnConfig
            .idleMode(ModuleConstants.kTurningMotorIdleMode)
            .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
        driveConfig
            .encoder
            .positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
            .velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
        turnConfig
            .encoder
            .positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
            .velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor)
            .inverted(ModuleConstants.kTurningEncoderInverted);
        driveConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(
                ModuleConstants.kDrivingP,
                ModuleConstants.kDrivingI,
                ModuleConstants.kDrivingD,
                ModuleConstants.kDrivingFF
            )
            .outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);
        turnConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pidf(
                ModuleConstants.kTurningP,
                ModuleConstants.kTurningI,
                ModuleConstants.kTurningD,
                ModuleConstants.kTurningFF
            )
            .outputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(ModuleConstants.kTurningEncoderPositionPIDMinInput, ModuleConstants.kTurningEncoderPositionPIDMaxInput);

        driveSparkMax.setCANTimeout(250);
        turnSparkMax.setCANTimeout(250);

        driveEncoder = driveSparkMax.getEncoder();
        turnEncoder = turnSparkMax.getAbsoluteEncoder();
        
        drivePIDController = driveSparkMax.getClosedLoopController();
        turnPIDController = turnSparkMax.getClosedLoopController();

        driveSparkMax.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turnSparkMax.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.position = new SwerveModulePosition(driveEncoder.getPosition(),
                new Rotation2d(turnEncoder.getPosition()));
        inputs.state = new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turnEncoder.getPosition()));
    }

    @Override
    public void setState(SwerveModuleState state) {
        drivePIDController.setReference(state.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
        turnPIDController.setReference(state.angle.getRadians(), SparkMax.ControlType.kPosition);
    }

    @Override
    public void resetEncoders() {
        driveEncoder.setPosition(0);
    }

}
