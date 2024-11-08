package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Drivetrain.MecanumDriveCmd;
import frc.robot.subsystems.DriveTrainSubsystem;

public class RobotContainer {

  final DriveTrainSubsystem driveSubsystem = new DriveTrainSubsystem();

  public final XboxController controller = new XboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {

    driveSubsystem.setDefaultCommand(
        new MecanumDriveCmd(
            driveSubsystem,
            () -> controller.getRawAxis(OperatorConstants.sideAxis),
            () -> controller.getRawAxis(OperatorConstants.forwardAxis),
            () -> controller.getRawAxis(OperatorConstants.rotationAxis),
            () -> 1d,
            () -> false));

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
