package frc.robot.subsystems.drivetrain;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class ModuleIOMAXSwerve implements ModuleIO {
    private final CANSparkMax driveSparkMax;
    private final CANSparkMax turnSparkMax;

    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder turnEncoder;

    private final SparkPIDController drivePIDController;
    private final SparkPIDController turnPIDController;

    public ModuleIOMAXSwerve(int index) {
        switch (index) {
            case 0:
                driveSparkMax = new CANSparkMax(DriveConstants.kFrontLeftDriveCanId, MotorType.kBrushless);
                turnSparkMax = new CANSparkMax(DriveConstants.kFrontLeftTurningCanId, MotorType.kBrushless);
                break;
            case 1:
                driveSparkMax = new CANSparkMax(DriveConstants.kFrontRightDriveCanId, MotorType.kBrushless);
                turnSparkMax = new CANSparkMax(DriveConstants.kFrontRightTurningCanId, MotorType.kBrushless);
                break;
            case 2:
                driveSparkMax = new CANSparkMax(DriveConstants.kBackLeftDriveCanId, MotorType.kBrushless);
                turnSparkMax = new CANSparkMax(DriveConstants.kBackLeftTurningCanId, MotorType.kBrushless);
                break;
            case 3:
                driveSparkMax = new CANSparkMax(DriveConstants.kBackRightDriveCanId, MotorType.kBrushless);
                turnSparkMax = new CANSparkMax(DriveConstants.kBackRightTurningCanId, MotorType.kBrushless);
                break;
            default:
                throw new RuntimeException("Invalid module index: " + index);
        }

        driveSparkMax.restoreFactoryDefaults();
        turnSparkMax.restoreFactoryDefaults();

        driveSparkMax.setCANTimeout(250);
        turnSparkMax.setCANTimeout(250);

        driveEncoder = driveSparkMax.getEncoder();
        turnEncoder = turnSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
        drivePIDController = driveSparkMax.getPIDController();
        turnPIDController = turnSparkMax.getPIDController();
        drivePIDController.setFeedbackDevice(driveEncoder);
        turnPIDController.setFeedbackDevice(turnEncoder);

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
        turnEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
        turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

        turnEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);
        turnPIDController.setPositionPIDWrappingEnabled(true);
        turnPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);
        turnPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

        drivePIDController.setP(ModuleConstants.kDrivingP);
        drivePIDController.setI(ModuleConstants.kDrivingI);
        drivePIDController.setD(ModuleConstants.kDrivingD);
        drivePIDController.setFF(ModuleConstants.kDrivingFF);
        drivePIDController.setOutputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);

        turnPIDController.setP(ModuleConstants.kTurningP);
        turnPIDController.setI(ModuleConstants.kTurningI);
        turnPIDController.setD(ModuleConstants.kTurningD);
        turnPIDController.setFF(ModuleConstants.kTurningFF);
        turnPIDController.setOutputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput);

        driveSparkMax.burnFlash();
        turnSparkMax.burnFlash();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.position = new SwerveModulePosition(driveEncoder.getPosition(),
                new Rotation2d(turnEncoder.getPosition()));
        inputs.state = new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turnEncoder.getPosition()));
    }

    @Override
    public void setState(SwerveModuleState state) {
        drivePIDController.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        turnPIDController.setReference(state.angle.getRadians(), CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void resetEncoders() {
        driveEncoder.setPosition(0);
    }

}
