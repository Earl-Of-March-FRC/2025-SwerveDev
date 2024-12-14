package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
    DriveConstants.kDriveKinematics, 
    new Rotation2d(),
    new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
    }
  );

  public Drivetrain(ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO, GyroIO gyroIO) {
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);
    this.gyroIO = gyroIO;
  }

  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    for (Module module : modules) {
      module.periodic();
    }

    ChassisSpeeds measuredChassisSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(
      modules[0].getState(), modules[1].getState(), modules[2].getState(), modules[3].getState()
    );

    // Simulation only
    if (gyroIO instanceof GyroIOSim) {
      ((GyroIOSim) gyroIO).updateAngularVelocity(measuredChassisSpeeds.omegaRadiansPerSecond);
    }

    Pose2d pose = odometry.update(
      gyroInputs.angle,
      new SwerveModulePosition[] {
        modules[0].getPosition(),
        modules[1].getPosition(),
        modules[2].getPosition(),
        modules[3].getPosition()
      }
    );

    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/Measured", new SwerveModuleState[] {});
      Logger.recordOutput("Drive/Odometry", new Pose2d());
      Logger.recordOutput("Drive/ChassisSpeeds/Setpoint", new ChassisSpeeds());
      Logger.recordOutput("Drive/ChassisSpeeds/Measured", new ChassisSpeeds());
    } else {
      Logger.recordOutput("Drive/Odometry", pose);
      Logger.recordOutput("Drive/ChassisSpeeds/Measured", measuredChassisSpeeds);
    }
  }

  public void runVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds fieldOrientedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, gyroInputs.angle
    );
    Logger.recordOutput("Drive/ChassisSpeeds/Setpoint", fieldOrientedSpeeds);

    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(fieldOrientedSpeeds);

    for (int i = 0; i < 4; i++) {
      states[i].speedMetersPerSecond *= DriveConstants.kMaxSpeedMetersPerSecond;
    }

    Logger.recordOutput("SwerveStates/Setpoints", states);

    for (int i = 0; i < 4; i++) {
      modules[i].runState(states[i]);
    }

    // TODO: Fix this, we shoudln't need this line with @AutoLogOutput
    Logger.recordOutput("SwerveStates/Measured", getStates());
  }

  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getStates() {
    return new SwerveModuleState[] {
        modules[0].getState(),
        modules[1].getState(),
        modules[2].getState(),
        modules[3].getState()
    };
  }
}
