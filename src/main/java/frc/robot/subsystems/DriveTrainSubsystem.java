package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ScaleFactorConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class DriveTrainSubsystem extends SubsystemBase {
  private CANSparkMax frontLeft = new CANSparkMax(DrivetrainConstants.frontLeftTalonPort, MotorType.kBrushless);
  private CANSparkMax frontRight = new CANSparkMax(DrivetrainConstants.frontRightTalonPort, MotorType.kBrushless);
  private CANSparkMax backLeft = new CANSparkMax(DrivetrainConstants.backLeftTalonPort, MotorType.kBrushless);
  private CANSparkMax backRight = new CANSparkMax(DrivetrainConstants.backRightTalonPort, MotorType.kBrushless);

  public MecanumDrive mecDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);

  public DriveTrainSubsystem() {
    frontLeft.setInverted(true);
    backLeft.setInverted(true);
  }

  public void setMecanumPermanent(double x, double y, double rx) {
    mecDrive.driveCartesian(x, y, rx);
  }

  public void setMecanum(double x, double y, double rx) {
    if (Math.abs(x) < ScaleFactorConstants.driveDeadzone) x = 0;
    if (Math.abs(y) < ScaleFactorConstants.driveDeadzone) y = 0;
    if (Math.abs(rx) < ScaleFactorConstants.rotateDeadzone) rx = 0;

    mecDrive.driveCartesian(x, y, rx);

    SmartDashboard.putNumber("x", x);
    SmartDashboard.putNumber("y", y);
    SmartDashboard.putNumber("rx", rx);
  }


  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
