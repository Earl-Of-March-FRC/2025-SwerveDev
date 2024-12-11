package frc.robot;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveDriveCmd;
import frc.robot.subsystems.DriveTrainSubsystem;

public class RobotContainer {

  final DriveTrainSubsystem driveSubsystem = new DriveTrainSubsystem();

  public final XboxController controller = new XboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {

    driveSubsystem.setDefaultCommand(
        new SwerveDriveCmd(
            driveSubsystem,
            () -> MathUtil.applyDeadband(-controller.getRawAxis(OIConstants.kDriverControllerXAxis), OIConstants.kDriveDeadband),
            () -> MathUtil.applyDeadband(controller.getRawAxis(OIConstants.kDriverControllerYAxis), OIConstants.kDriveDeadband),
            () -> MathUtil.applyDeadband(controller.getRawAxis(OIConstants.kDriverControllerRotAxis), OIConstants.kDriveDeadband)
        )
    );            
    
    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    // switch(cycle){
    //   case 1:
    //     new Leave(driveSubsystem);
    //   case 2:
    //     new ScoreFloorLeave(armMotors, driveSubsystem, claw);
    //   default:
    //     break;
    // }
    // return null;
    return null;
  }
}
