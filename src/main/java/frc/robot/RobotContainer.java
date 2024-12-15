package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.GoToAmpCmd;
import frc.robot.commands.GoToHumanCmd;
import frc.robot.commands.GoToSpeakerCCmd;
import frc.robot.commands.SwerveDriveCmd;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.GyroIOADXRS450;
import frc.robot.subsystems.drivetrain.GyroIOSim;
import frc.robot.subsystems.drivetrain.ModuleIOMAXSwerve;
import frc.robot.subsystems.drivetrain.ModuleIOSim;

public class RobotContainer {

  final Drivetrain driveSub;
  final GyroIO gyroIO;

  public final XboxController controller = new XboxController(OIConstants.kDriverControllerPort);
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    if (RobotBase.isReal()) {
      gyroIO = new GyroIOADXRS450();
      driveSub = new Drivetrain(
          new ModuleIOMAXSwerve(0),
          new ModuleIOMAXSwerve(1),
          new ModuleIOMAXSwerve(2),
          new ModuleIOMAXSwerve(3),
          gyroIO);
    } else {
      gyroIO = new GyroIOSim();
      driveSub = new Drivetrain(
          new ModuleIOSim(),
          new ModuleIOSim(),
          new ModuleIOSim(),
          new ModuleIOSim(),
          gyroIO);
    }

    autoChooser = new LoggedDashboardChooser<Command>("Auto Routine", AutoBuilder.buildAutoChooser());
    SmartDashboard.putData("Auto Routine", autoChooser.getSendableChooser());

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
    new JoystickButton(controller, 2).whileTrue(
      new GoToHumanCmd()
    );
    new JoystickButton(controller, 3).whileTrue(
      new GoToSpeakerCCmd()
    );
    new JoystickButton(controller, 4).whileTrue(
      new GoToAmpCmd()
    );
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
