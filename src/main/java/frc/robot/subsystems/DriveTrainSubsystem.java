package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrainSubsystem extends SubsystemBase {

  private MAXSwerveModule m_swerveModuleFrontLeft;
  private MAXSwerveModule m_swerveModuleBackRight;

  private SwerveModuleState[] allStates = new SwerveModuleState[] {
    new SwerveModuleState(),
    new SwerveModuleState(),
    new SwerveModuleState(),
    new SwerveModuleState()
  };

  private StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault().getStructArrayTopic("SwerveStates", SwerveModuleState.struct).publish();
  
  public DriveTrainSubsystem() {
    m_swerveModuleFrontLeft = new MAXSwerveModule(DriveConstants.kFrontLeftDriveCanId, DriveConstants.kFrontLeftTurningCanId, DriveConstants.kFrontLeftChassisAngularOffset);
    m_swerveModuleBackRight= new MAXSwerveModule(DriveConstants.kBackRightDriveCanId, DriveConstants.kBackRightTurningCanId, DriveConstants.kBackRightChassisAngularOffset);
  }

  public void setSwerveState(SwerveModuleState stateFrontLeft, SwerveModuleState stateBackRight) {
    // SwerveModuleState desiredState = new SwerveModuleState();
    // desiredState.speedMetersPerSecond = stateFrontLeft.speedMetersPerSecond * 0.2; // speed lowered to 20% for testing
    // desiredState.angle = desiredState.angle;

    allStates = new SwerveModuleState[] {
      stateFrontLeft,
      new SwerveModuleState(),
      new SwerveModuleState(),
      stateBackRight
    };

    m_swerveModuleFrontLeft.setDesiredState(stateFrontLeft);
    m_swerveModuleBackRight.setDesiredState(stateBackRight);

  }

  public void resetEncoders() {
    m_swerveModuleFrontLeft.resetEncoders();
    m_swerveModuleBackRight.resetEncoders();
  }

  public SwerveModuleState getFrontLeftState() {
    return m_swerveModuleFrontLeft.getState();
  }

  public SwerveModuleState getBackRightState() {
    return m_swerveModuleBackRight.getState();
  }

  @Override
  public void periodic() {
    publisher.set(allStates);
  }

  @Override
  public void simulationPeriodic() {}
}
