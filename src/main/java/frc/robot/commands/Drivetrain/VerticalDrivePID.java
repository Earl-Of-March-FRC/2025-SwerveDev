// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class VerticalDrivePID extends PIDCommand {
  /** Creates a new DrivePID. */
  public VerticalDrivePID(DrivetrainSubsystem drive, double setpoint) {
    super(
        // The controller that the command will use
        new PIDController(
            DrivetrainConstants.kPVerticalDrive,
            DrivetrainConstants.kIVerticalDrive,
            DrivetrainConstants.kDVerticalDrive),
        // This should return the measurement
        () -> drive.getDistance(),
        // This should return the setpoint (can also be a constant)
        () -> setpoint,
        // This uses the output
        output -> {
          drive.setMecanum(output, 0, 0);
        });
    getController().setTolerance(5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
