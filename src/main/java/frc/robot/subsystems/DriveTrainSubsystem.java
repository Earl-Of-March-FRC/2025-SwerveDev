package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrainSubsystem extends SubsystemBase {

  private MAXSwerveModule m_swerveModule;
  
  public DriveTrainSubsystem() {
    m_swerveModule = new MAXSwerveModule(DriveConstants.kDriveCanId, DriveConstants.kTurningCanId, DriveConstants.kAngularOffset);
  }

  public void setSwerveState(SwerveModuleState state) {
    SwerveModuleState desiredState = new SwerveModuleState();
    desiredState.speedMetersPerSecond = state.speedMetersPerSecond * 0.2;
    desiredState.angle = desiredState.angle;
    m_swerveModule.setDesiredState(state);
  }

  public void resetEncoders() {
    m_swerveModule.resetEncoders();
  }

  public SwerveModuleState getState() {
    return m_swerveModule.getState();
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
