package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

  private final Module[] modules = new Module[2]; // FL, FR, BL, BR
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private Rotation2d[] prevAngle = new Rotation2d[] {
      new Rotation2d(),
      new Rotation2d()
  };

  Translation2d baseXVec[] = { new Translation2d(1, 0), new Translation2d(1, 0) };
  Translation2d baseYVec[] = { new Translation2d(0, 1), new Translation2d(0, 1) };
  Translation2d baseRotVec[] = { new Translation2d(-1, 1), new Translation2d(1, -1) };

  public Drivetrain(ModuleIO flModuleIO, ModuleIO brModuleIO, GyroIO gyroIO) {
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(brModuleIO, 3);
    this.gyroIO = gyroIO;
  }

  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    for (Module module : modules) {
      module.periodic();
    }

    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
    }
  }

  public void runVelocity(ChassisSpeeds speeds) {
    Translation2d frontLeftVec = baseXVec[0].times(speeds.vxMetersPerSecond)
        .plus(baseYVec[0].times(speeds.vyMetersPerSecond)).rotateBy(gyroInputs.angle.times(-1));
    frontLeftVec = frontLeftVec.plus(baseRotVec[0].times(speeds.omegaRadiansPerSecond));
    Translation2d backRightVec = baseXVec[1].times(speeds.vxMetersPerSecond)
        .plus(baseYVec[1].times(speeds.vyMetersPerSecond)).rotateBy(gyroInputs.angle.times(-1));
    backRightVec = backRightVec.plus(baseRotVec[1].times(speeds.omegaRadiansPerSecond));

    SwerveModuleState[] desiredStates = new SwerveModuleState[] {
        new SwerveModuleState(frontLeftVec.getNorm(), frontLeftVec.getAngle()),
        new SwerveModuleState(backRightVec.getNorm(), backRightVec.getAngle())
    };
    Double maxVel = Math.max(desiredStates[0].speedMetersPerSecond, desiredStates[1].speedMetersPerSecond);

    for (int i = 0; i < desiredStates.length; i++) {
      SwerveModuleState state = desiredStates[i];

      if (maxVel > DriveConstants.kMaxSpeedMetersPerSecond) {
        state.speedMetersPerSecond /= maxVel;
      }

      if (state.speedMetersPerSecond <= 0.05) {
        state.speedMetersPerSecond = 0;
        state.angle = prevAngle[i];
      }

      prevAngle[i] = state.angle;
      modules[i].runState(state);
    }

    Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] { desiredStates[0], new SwerveModuleState(),
        new SwerveModuleState(), desiredStates[1] });
  }

  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = modules[0].getState();
    states[1] = new SwerveModuleState();
    states[2] = new SwerveModuleState();
    states[3] = modules[1].getState();
    return states;
  }
}
