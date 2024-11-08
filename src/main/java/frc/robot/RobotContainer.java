package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriverStationConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Drivetrain.MecanumDriveCmd;
import frc.robot.subsystems.DriveTrainSubsystem;

public class RobotContainer {

  final DriveTrainSubsystem driveSubsystem = new DriveTrainSubsystem();

  public final Joystick controller = new Joystick(OperatorConstants.kDriverControllerPort);

  public final Joystick driverStation =
      new Joystick(DriverStationConstants.DriverStationController);

  public JoystickButton triggerButton = new JoystickButton(controller, 1);

  public RobotContainer() {

    driveSubsystem.setDefaultCommand(
        new MecanumDriveCmd(
            driveSubsystem,
            () -> controller.getRawAxis(OperatorConstants.sideAxis),
            () -> controller.getRawAxis(OperatorConstants.forwardAxis),
            () -> controller.getRawAxis(OperatorConstants.rotationAxis),
            () -> controller.getRawAxis(OperatorConstants.scaleAxis),
            () -> triggerButton.getAsBoolean()));

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
