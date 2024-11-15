// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainSubsystem;

public class SwerveDriveCmd extends Command {

  private DriveTrainSubsystem driveSubsystem;
  private Supplier<Double> velocityFunction;
  private Supplier<Rotation2d> rotationFunction;

  public SwerveDriveCmd(DriveTrainSubsystem dSub, Supplier<Double> vFunc, Supplier<Rotation2d> rFunc) {
    driveSubsystem = dSub;
    velocityFunction = vFunc;
    rotationFunction = rFunc;

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveModuleState desiredState = new SwerveModuleState(velocityFunction.get(), rotationFunction.get());
    driveSubsystem.setSwerveState(desiredState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
