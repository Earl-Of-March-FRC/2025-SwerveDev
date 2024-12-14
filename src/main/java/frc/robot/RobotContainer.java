package frc.robot;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
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
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.575, 7.022, new Rotation2d()),
      List.of(
        new Translation2d(5.406, 4.178),
        new Translation2d(10.094, 1.986)
      ),
      new Pose2d(15.31, 0.914, new Rotation2d()),
      trajectoryConfig
    );

    PIDController xController = new PIDController(AutoConstants.kPXController, AutoConstants.kIXController, AutoConstants.kDXController);
    PIDController yController = new PIDController(AutoConstants.kPYController, AutoConstants.kIYController, AutoConstants.kDYController);
    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController,
      AutoConstants.kThetaControllerConstraints
    );
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      trajectory,
      driveSub::getPose,
      DriveConstants.kDriveKinematics,
      xController,
      yController,
      thetaController,
      driveSub::runModuleStates,
      driveSub
    );

    return new SequentialCommandGroup(
      new InstantCommand(() -> driveSub.resetOdometry(new Rotation2d(0), trajectory.getInitialPose())),
      swerveControllerCommand
    );
  }
}
