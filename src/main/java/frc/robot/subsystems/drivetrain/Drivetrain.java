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

  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private Rotation2d[] prevAngle = new Rotation2d[] {
      new Rotation2d(),
      new Rotation2d()
  };

  public Drivetrain(ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);
  }

  @Override
  public void periodic() {
    for (Module module : modules) {
      module.periodic();
    }

    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
    }
  }

  public void runVelocity(ChassisSpeeds speeds) {
    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

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
