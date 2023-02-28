// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveSubsystem;

/** Drives onto the charge station and balances */
public class AutoChargeStationSequence extends SequentialCommandGroup {

  private static final double NORM_SPEED_UP_CHARGE_STATION = 0.3;
  private static final double DISTANCE_UP_CHARGE_STATION_METERS = 2.21;

  public AutoChargeStationSequence(boolean isFirstPath, DriveSubsystem drive) {
    addCommands(
      new RunCommand(() -> {drive.drive(NORM_SPEED_UP_CHARGE_STATION, 0, 0, false);}, drive).until(
        () -> {return drive.getPose().getX() >DISTANCE_UP_CHARGE_STATION_METERS; }
      )
    );
  }
}
