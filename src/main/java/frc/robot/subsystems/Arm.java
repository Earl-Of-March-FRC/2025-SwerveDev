package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private WPI_VictorSPX extension1 = new WPI_VictorSPX(ArmConstants.extension1);

  private WPI_TalonSRX incline1 = new WPI_TalonSRX(ArmConstants.incline1);
  private WPI_TalonSRX incline2 = new WPI_TalonSRX(ArmConstants.incline2);

  private final Encoder encoder = new Encoder(0, 1);

  public Arm() {
    incline2.setSelectedSensorPosition(25000);
    incline1.setNeutralMode(NeutralMode.Brake);
    incline2.setNeutralMode(NeutralMode.Brake);
    extension1.setNeutralMode(NeutralMode.Brake);
  }

  public double getInclineAngle() {
    return incline2.getSelectedSensorPosition() / (25000 / ArmConstants.armMaxAngle);
  }

  public double getExtensionInches() {
    return 37.0
        + (encoder.getRaw() / (-4100.0 / 51.0)); // change 25220 to num ticks per full extension
  }

  public void setEncoderValueIncline(double ticks) {
    incline1.set(ControlMode.Position, ticks);
    incline2.set(ControlMode.Position, ticks);
  }

  public void resetEncoders() {
    incline1.setSelectedSensorPosition(0);
    incline2.setSelectedSensorPosition(25000);
    encoder.reset();
  }

  public void setVoltageExtension(double voltage) {
    extension1.setVoltage(voltage);
  }

  public void armExtension(double speed) {
    extension1.set(-speed);
  }

  public double getRateExtension() {
    return encoder.getRate();
  }

  public double getRateIncline() {
    return incline2.getSelectedSensorVelocity();
  }

  public void armExtensionBrake() {
    extension1.setNeutralMode(NeutralMode.Brake);
    incline1.setNeutralMode(NeutralMode.Brake);
    incline2.setNeutralMode(NeutralMode.Brake);
  }

  public void armIncline(double speed) {
    incline1.set(speed);
    incline2.set(-speed);
  }

  public void extendStop() {
    extension1.set(0);
  }

  public void inclineStop() {
    incline1.set(0);
    incline2.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Incline 1", incline1.getSelectedSensorPosition());
    SmartDashboard.putNumber("Incline Angle", getInclineAngle());

    SmartDashboard.putNumber("Encoder Extension", getExtensionInches());
  }
}
