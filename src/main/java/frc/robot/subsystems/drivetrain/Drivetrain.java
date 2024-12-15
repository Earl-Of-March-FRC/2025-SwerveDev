package frc.robot.subsystems.drivetrain;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
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

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      throw new RuntimeException("Failed to load robot config", e);
    }

    AutoBuilder.configure(
      this::getPose,
      this::resetOdometry,
      this::getRelativeChassisSpeeds, 
      (speeds, feedforwards) -> runVelocityRobotRelative(speeds),
      new PPHolonomicDriveController(
        new PIDConstants(AutoConstants.kPXController, AutoConstants.kIXController, AutoConstants.kDXController),
        new PIDConstants(AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController)
      ),
      config,
      () -> {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
      },
      this
    );
  }

  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    for (Module module : modules) {
      module.periodic();
    }

    ChassisSpeeds measuredChassisSpeeds = getRelativeChassisSpeeds();
    // Simulation only
    if (gyroIO instanceof GyroIOSim) {
      ((GyroIOSim) gyroIO).updateAngularVelocity(measuredChassisSpeeds.omegaRadiansPerSecond);
    }

    Pose2d pose = odometry.update(
      gyroInputs.angle,
      getModulePositions()
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
    runVelocity(speeds, true);
  }

  public void runVelocityRobotRelative(ChassisSpeeds speeds) {
    runVelocity(speeds, false);
  }

  public void runVelocity(ChassisSpeeds speeds, Boolean fieldRelative) {
    Rotation2d compensatedAngle = gyroInputs.angle.plus(new Rotation2d(gyroInputs.rate*0.05));
    if (fieldRelative) speeds.toFieldRelativeSpeeds(compensatedAngle);
    
    Logger.recordOutput("Drive/ChassisSpeeds/Setpoint", speeds);

    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);

    // for (int i = 0; i < 4; i++) {
    //   states[i].speedMetersPerSecond *= DriveConstants.kMaxSpeedMetersPerSecond;
    // }

    Logger.recordOutput("SwerveStates/Setpoints", states);

    for (int i = 0; i < 4; i++) {
      modules[i].runState(states[i]);
    }

    // TODO: Fix this, we shoudln't need this line with @AutoLogOutput
    Logger.recordOutput("SwerveStates/Measured", getModuleStates());
  }

  public void runModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < 4; i++) {
      modules[i].runState(states[i]);
    }
  }

  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        modules[0].getState(),
        modules[1].getState(),
        modules[2].getState(),
        modules[3].getState()
    };
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        modules[0].getPosition(),
        modules[1].getPosition(),
        modules[2].getPosition(),
        modules[3].getPosition()
    };
  }

  @AutoLogOutput(key = "Drive/Odometry")
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Rotation2d rotation, SwerveModulePosition[] modulePositions, Pose2d pose) {
    odometry.resetPosition(rotation, modulePositions, pose);
  }

  public void resetOdometry(Pose2d pose) {
    resetOdometry(new Rotation2d(), getModulePositions(), pose);
  }

  public void resetOdometry(Rotation2d rotation, Pose2d pose) {
    resetOdometry(rotation, getModulePositions(), pose);
  }

  public void resetOdometry() {
    resetOdometry(new Rotation2d(0), getModulePositions(), new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
  }

  public ChassisSpeeds getRelativeChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
      modules[0].getState(), modules[1].getState(), modules[2].getState(), modules[3].getState()
    );
  }
}
