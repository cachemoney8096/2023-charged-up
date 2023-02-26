// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Cal;
import frc.robot.subsystems.drive.DriveSubsystem;

/** Drives onto the charge station and balances */
public class AutoChargeStationSequence extends SequentialCommandGroup {
  private PathPlannerTrajectory traj =
      PathPlanner.loadPath(
          "EngageOnlyTraj",
          new PathConstraints(
              0.75,
              1.5));

  public AutoChargeStationSequence(boolean isFirstPath, DriveSubsystem drive) {
    // addCommands(
    //     new InstantCommand(() -> {drive.setModuleDrivePidf(
    //       Cal.SwerveModule.DRIVING_PITCHED_P,
    //       Cal.SwerveModule.DRIVING_PITCHED_I,
    //       Cal.SwerveModule.DRIVING_PITCHED_D,
    //        Cal.SwerveModule.DRIVING_PITCHED_FF);}),
    //     drive.followTrajectoryCommand(traj, isFirstPath),
    //     new InstantCommand(() -> {drive.setModuleDrivePidf(
    //       Cal.SwerveModule.DRIVING_P,
    //       Cal.SwerveModule.DRIVING_I,
    //       Cal.SwerveModule.DRIVING_D,
    //        Cal.SwerveModule.DRIVING_FF);}));
    addCommands(
      new RunCommand(() -> {drive.drive(0.3, 0, 0, false);}, drive).until(
        () -> {return drive.getPose().getX() > 2.21; }
      )
    );
  }

  public PathPlannerTrajectory getTrajectory() {
    return this.traj;
  }
}
