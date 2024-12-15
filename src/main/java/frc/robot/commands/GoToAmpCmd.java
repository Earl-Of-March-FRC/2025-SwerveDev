// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;

public class GoToAmpCmd extends SequentialCommandGroup {
  public GoToAmpCmd() {
    PathPlannerPath intoAmpPath = PathPlannerPath.fromPathFile("Into Amp");
    PathConstraints pathfindingConstraints = new PathConstraints(
      AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared,
      AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared
    );

    addCommands(
      AutoBuilder.pathfindThenFollowPath(intoAmpPath, pathfindingConstraints)
    );
  }
}
