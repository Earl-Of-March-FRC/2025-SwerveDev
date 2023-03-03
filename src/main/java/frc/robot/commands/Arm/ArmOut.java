// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmOut extends CommandBase {

  Arm armMotors;
  /** Creates a new ArmOut. */
  public ArmOut(Arm armMotors) {
    this.armMotors = armMotors;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // armMotors.armOut();
    armMotors.armExtension(0.4);
  }

  @Override
  public void end(boolean interrupted) {
    armMotors.extendStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
