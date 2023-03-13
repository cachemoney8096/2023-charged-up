// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.components;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Cal;
import frc.robot.subsystems.drive.DriveSubsystem;

/** Drives onto the charge station and balances */
public class AutoChargeStationSequence extends SequentialCommandGroup {

  private double startXMeters = 0.0;
  private static final double NORM_SPEED_UP_CHARGE_STATION = 0.4;

  public AutoChargeStationSequence(DriveSubsystem drive, double distanceMeters) {
    addCommands(
        new InstantCommand(
            () -> {
              startXMeters = drive.getPose().getX();
            }),
        new RunCommand(
                () -> {
                  /** Time remaining in current match period (auto or teleop) in seconds */
                  double matchTime = DriverStation.getMatchTime();
                  if (matchTime > Cal.AutoBalance.SET_X_TIME_LEFT_SECONDS) {
                    drive.drive(NORM_SPEED_UP_CHARGE_STATION, 0, 0, true);
                  } else {
                    drive.setX();
                  }
                },
                drive)
            .until(
                () -> {
                  return (drive.getPose().getX() - startXMeters) > distanceMeters;
                }),
        new AutoChargeStationBalance(drive));
  }
}
