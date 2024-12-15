// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class SwerveDriveCmd extends Command {

  private Drivetrain driveSub;
  private Supplier<Double> xSupplier;
  private Supplier<Double> ySupplier;
  private Supplier<Double> omegaSupplier;

  Rotation2d prevAngle[] = new Rotation2d[] {
      new Rotation2d(0),
      new Rotation2d(0)
  };

  public SwerveDriveCmd(Drivetrain dSub, Supplier<Double> xSup, Supplier<Double> ySup, Supplier<Double> wSup) {
    driveSub = dSub;
    xSupplier = xSup;
    ySupplier = ySup;
    omegaSupplier = wSup;

    addRequirements(driveSub);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    Double xVel = xSupplier.get()*DriveConstants.kMaxSpeedMetersPerSecond;
    Double yVel = ySupplier.get()*DriveConstants.kMaxSpeedMetersPerSecond;
    Double omega = omegaSupplier.get()*DriveConstants.kMaxAngularSpeed;
    Logger.recordOutput("Drive/Inputs/xVel", xVel);
    Logger.recordOutput("Drive/Inputs/yVel", yVel);
    Logger.recordOutput("Drive/Inputs/omega", omega);

    driveSub.runVelocity(new ChassisSpeeds(xVel, yVel, omega));
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
