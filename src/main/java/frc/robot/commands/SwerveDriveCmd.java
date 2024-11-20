// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class SwerveDriveCmd extends Command {

  private DriveTrainSubsystem driveSubsystem;
  private Supplier<Double> velocityFunction;
  private Supplier<Rotation2d> rotationFunction;

  SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.kDirectionSlewRate);
  SlewRateLimiter magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);

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
    double angle = rotationFunction.get().getRadians();


    //THe wheels are subtracting by the offset constant!!

    Rotation2d frontLeftRotation = new Rotation2d(rotLimiter.calculate(Math.atan2
    (Math.sin(angle - DriveConstants.kFrontLeftChassisAngularOffset), Math.cos(angle - DriveConstants.kFrontLeftChassisAngularOffset))));
    Rotation2d frontRightRotation = new Rotation2d(rotLimiter.calculate(Math.atan2
    (Math.sin(angle - DriveConstants.kFrontRightChassisAngularOffset), Math.cos(angle - DriveConstants.kFrontRightChassisAngularOffset))));
    Rotation2d backLeftRotation = new Rotation2d(rotLimiter.calculate(Math.atan2
    (Math.sin(angle - DriveConstants.kBackLeftChassisAngularOffset), Math.cos(angle - DriveConstants.kBackLeftChassisAngularOffset))));
    Rotation2d backRightRotation = new Rotation2d(rotLimiter.calculate(Math.atan2
    (Math.sin(angle - DriveConstants.kBackRightChassisAngularOffset), Math.cos(angle - DriveConstants.kBackRightChassisAngularOffset))));

    SwerveModuleState desiredStateFrontLeft = new SwerveModuleState(magLimiter.calculate(velocityFunction.get()), new Rotation2d(rotLimiter.calculate(rotationFunction.get().getRadians())));
    SwerveModuleState desiredStateBackRight = new SwerveModuleState(magLimiter.calculate(velocityFunction.get()), new Rotation2d(rotLimiter.calculate(rotationFunction.get().getRadians())));
    driveSubsystem.setSwerveState(desiredStateFrontLeft, desiredStateBackRight);
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
