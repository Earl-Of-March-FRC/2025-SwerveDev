package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  private RobotContainer m_robotContainer;

  // SendableChooser<Command> auto = new SendableChooser<Command>();

  @Override
  public void robotInit() {
    // CameraServer.startAutomaticCapture();
    m_robotContainer = new RobotContainer();

    // auto.setDefaultOption("ScoreTopLeaveFar", new ScoreTopLeaveFar(
    //   m_robotContainer.driveSubsystem,
    //   m_robotContainer.armMotors,
    //   m_robotContainer.claw, 35, 80));

    // auto.addOption("ScoreTopBalance", new ScoreTopBalance(
    //   m_robotContainer.driveSubsystem,
    //   m_robotContainer.armMotors,
    //   m_robotContainer.claw,
    //   35,
    //   80));

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {

    // m_robotContainer.armMotors.resetEncoders();
    // m_robotContainer.driveSubsystem.resetEncoders();
    // m_robotContainer.driveSubsystem.resetGyro();
    // m_robotContainer.driveSubsystem.calibrateGyro();

  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // m_robotContainer.armMotors.resetEncoders();
    // m_robotContainer.driveSubsystem.resetEncoders();
    // m_robotContainer.driveSubsystem.resetGyro();
    // m_robotContainer.driveSubsystem.calibrateGyro();

    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
