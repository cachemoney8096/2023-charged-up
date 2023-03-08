// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveSubsystem;

/** Drives onto the charge station and balances */
public class AutoChargeStationSequence extends SequentialCommandGroup {

  private double startXMeters = 0.0;
  private final double NORM_SPEED_UP_CHARGE_STATION;

  public AutoChargeStationSequence(boolean red, DriveSubsystem drive, double distanceMeters) {
    if (red) {
      NORM_SPEED_UP_CHARGE_STATION = -0.4;
    }
    else {
      NORM_SPEED_UP_CHARGE_STATION = 0.4;
    }
    addCommands(
        new InstantCommand(() -> {startXMeters = drive.getPose().getX();}),
        new RunCommand(
                () -> {
                  /** Time remaining in current match period (auto or teleop) in seconds */
                  double matchTime = DriverStation.getMatchTime();
                  if (matchTime > 0.3) {
                    drive.drive(NORM_SPEED_UP_CHARGE_STATION, 0, 0, true);
                  } else {
                    drive.drive(0.0, 0, 0, false);
                  }
                },
                drive)
            .until(
                () -> {
                  if (red) {
                    return (drive.getPose().getX() - startXMeters) < distanceMeters;
                  }
                  else {
                    return (drive.getPose().getX() - startXMeters) > distanceMeters;
                  }
                }),
        new AutoChargeStationBalance(drive));
  }
}
