// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Calibrations;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoChargeStationSequence extends CommandBase {
  private final DriveSubsystem drive;
  private final double MOVING_X = Calibrations.PLACEHOLDER_DOUBLE;
  private final double NOT_MOVING_IN_Y = 0;
  private final double NOT_ROTATING = 0;
  private final boolean FIELD_RELATIVE = false;

  public AutoChargeStationSequence(DriveSubsystem drive){
        this.drive = drive;
  }

  @Override
  public void initialize(){
    Timer timer = new Timer();
    timer.start();
    while (timer.get() < Calibrations.DRIVE_TIME_AUTO_SECONDS) {
        drive.drive(MOVING_X, NOT_MOVING_IN_Y, NOT_ROTATING, FIELD_RELATIVE);
    }
    drive.drive(0, NOT_MOVING_IN_Y, NOT_ROTATING, FIELD_RELATIVE);
  }

  @Override
  public void execute() {
    AutoChargeStationBalance balanceAuto = new AutoChargeStationBalance(drive);
    while (Timer.getMatchTime() > 1){
        balanceAuto.execute();
    }
  }
}
