package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriverStationConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commandgroups.AutoArm.ArmExtensionAndInclineLow;
import frc.robot.commandgroups.AutoArm.ArmExtensionAndInclineTop;
import frc.robot.commandgroups.AutoCmds.LeaveClose;
import frc.robot.commandgroups.AutoCmds.LeaveFar;
import frc.robot.commandgroups.AutoCmds.ScoreTopLeaveClose;
import frc.robot.commandgroups.AutoCmds.ScoreTopLeaveFar;
import frc.robot.commandgroups.TeleopArm.ArmDefaultPosition;
import frc.robot.commandgroups.TeleopArm.ArmPlaceMid;
import frc.robot.commandgroups.TeleopArm.ArmPlaceLow;
import frc.robot.commandgroups.TeleopArm.ArmPlaceTop;
import frc.robot.commands.Arm.ArmControl;
import frc.robot.commands.Arm.ArmRetract;
import frc.robot.commands.Drivetrain.MecanumDriveCmd;
import frc.robot.commands.ClawControl;
import frc.robot.commands.GyroReset;
// import frc.robot.commandgroups.AutoCmds.ScoreFloorAndBalance;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

  final DrivetrainSubsystem driveSubsystem = new DrivetrainSubsystem();
  public final Arm armMotors = new Arm();
  public final Claw claw = new Claw();


  public final VisionSubsystem vision = new VisionSubsystem();
  public final LEDSubsystem led = new LEDSubsystem();

  public final Joystick controller = new Joystick(OperatorConstants.kDriverControllerPort);

  public final Joystick driverStation =
      new Joystick(DriverStationConstants.DriverStationController);

  public RobotContainer() {

    driveSubsystem.setDefaultCommand(
        new MecanumDriveCmd(
            driveSubsystem,
            () -> controller.getRawAxis(OperatorConstants.sideAxis),
            () -> controller.getRawAxis(OperatorConstants.forwardAxis),
            () -> controller.getRawAxis(OperatorConstants.rotationAxis),
            () -> controller.getRawAxis(OperatorConstants.scaleAxis)));

    armMotors.setDefaultCommand(
        new ArmControl(
            armMotors,
            () -> driverStation.getRawAxis(DriverStationConstants.ArmExtendAxis),
            () -> driverStation.getRawAxis(DriverStationConstants.ArmInclineAxis)));

    configureBindings();
  }

  private void configureBindings() {

    new JoystickButton(driverStation, DriverStationConstants.ClawOpenButton)
        .whileTrue(new ClawControl(claw, -1));
    new JoystickButton(driverStation, DriverStationConstants.ClawCloseButton)
        .whileTrue(new ClawControl(claw, 1));

    // new JoystickButton(controller, 8).toggleOnTrue(new ScaleButtonCmd());

    // new JoystickButton(driverStation, 2).onTrue(new ArmRetract(armMotors, 38.0));

    // top position
    new JoystickButton(driverStation, 4).onTrue(new ArmPlaceTop(armMotors, led));

    // middle position
    new JoystickButton(driverStation, 2).onTrue(new ArmPlaceMid(armMotors, led));

    // Low position
    new JoystickButton(driverStation, 1).onTrue(new ArmPlaceLow(armMotors, led));

    // default position
    new JoystickButton(driverStation, 7).onTrue(new ArmDefaultPosition(armMotors, led));

    new JoystickButton(controller, 7).onTrue(new GyroReset(driveSubsystem));


    // new JoystickButton(driverStation, 2).onTrue(new ScoreTopLeaveClose(driveSubsystem, armMotors,
    // claw));

    // new JoystickButton(controller, 1).onTrue(new VerticalPID(driveSubsystem, -10));
    // new JoystickButton(controller, 10).onTrue(new ResetCalibrateGyro(driveSubsystem));
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
    return new ScoreTopLeaveClose(driveSubsystem, armMotors, claw);
  }
}
