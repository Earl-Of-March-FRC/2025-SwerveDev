package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import java.util.function.Supplier;

public class MecanumDriveCmd extends Command {

  private DriveTrainSubsystem driveSubsystem;
  private Supplier<Double> forwardFunction, sideFunction, rotateFunction, scaleFactor;
  SlewRateLimiter filter = new SlewRateLimiter(0.5);

  private Supplier<Boolean> limit;

  public MecanumDriveCmd(
      DriveTrainSubsystem driveSubsystem,
      Supplier<Double> ff,
      Supplier<Double> sf,
      Supplier<Double> rf,
      Supplier<Double> scaling,
      Supplier<Boolean> limit) {

    this.driveSubsystem = driveSubsystem;
    this.forwardFunction = ff;
    this.sideFunction = sf;
    this.rotateFunction = rf;
    this.scaleFactor = scaling;
    this.limit = limit;

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    double speedForward, speedSide, speedRotate;
    speedForward =
        (DrivetrainConstants.maxDriveSpeed * (-scaleFactor.get() + 1) / 2) * forwardFunction.get();

    speedSide =
        -1
            * (DrivetrainConstants.maxDriveSpeed * (-scaleFactor.get() + 1) / 2)
            * sideFunction.get();

    speedRotate =
        -1
            * (DrivetrainConstants.maxTurnSpeed * (-scaleFactor.get() + 1) / 2)
            * rotateFunction.get();

    if (limit.get()) {
      speedForward = 0;
      speedRotate = 0;
    }

    // driveSubsystem.setMecanum(speedSide, speedForward, speedRotate);
    driveSubsystem.setMecanum(
        filter.calculate(speedSide), filter.calculate(speedForward), filter.calculate(speedRotate));

    // double speedForward;

    // double sideFunction =
    //     (DrivetrainConstants.maxDriveSpeed * (-scaleFactor.get() + 1) / 2) *
    // this.sideFunction.get();
    //     SmartDashboard.putNumber("Side speed", speedForward);
    // driveSubsystem.setMecanum(sideFunction, speedForward, rotateFunction.get(),
    // scaleFactor.get());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
