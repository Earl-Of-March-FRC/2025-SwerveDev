package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveDriveCmd;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.ModuleIOMAXSwerve;
import frc.robot.subsystems.drivetrain.ModuleIOSim;

public class RobotContainer {

  final Drivetrain driveSub;

  public final XboxController controller = new XboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    if (RobotBase.isReal()) {
      driveSub = new Drivetrain(
          new ModuleIOMAXSwerve(0),
          new ModuleIOMAXSwerve(3)
      );
    } else {
      driveSub = new Drivetrain(
          new ModuleIOSim(),
          new ModuleIOSim()
      );
    }

    driveSub.setDefaultCommand(
        new SwerveDriveCmd(
            driveSub,
            () -> MathUtil.applyDeadband(-controller.getRawAxis(OIConstants.kDriverControllerYAxis),
                OIConstants.kDriveDeadband),
            () -> MathUtil.applyDeadband(-controller.getRawAxis(OIConstants.kDriverControllerXAxis),
                OIConstants.kDriveDeadband),
            () -> MathUtil.applyDeadband(-controller.getRawAxis(OIConstants.kDriverControllerRotAxis),
                OIConstants.kDriveDeadband)));

    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
