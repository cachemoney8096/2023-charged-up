// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Cal;
import frc.robot.subsystems.drive.DriveSubsystem;

/** Drives onto the charge station and balances */
public class AutoChargeStationSequence extends SequentialCommandGroup {
  private PathPlannerTrajectory traj =
      PathPlanner.loadPath(
          "EngageOnlyTraj",
          new PathConstraints(
              Cal.SwerveSubsystem.MAX_LINEAR_SPEED_METERS_PER_SEC,
              Cal.SwerveSubsystem.MAX_LINEAR_ACCELERATION_METERS_PER_SEC_SQ));

  public AutoChargeStationSequence(boolean isFirstPath, DriveSubsystem drive) {
    addCommands(
        drive.followTrajectoryCommand(traj, isFirstPath), new AutoChargeStationBalance(drive));
  }

  public PathPlannerTrajectory getTrajectory() {
    return this.traj;
  }
}
