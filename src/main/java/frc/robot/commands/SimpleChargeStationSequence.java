package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveSubsystem;

public class SimpleChargeStationSequence extends SequentialCommandGroup {
  public SimpleChargeStationSequence(DriveSubsystem drive) {
    addCommands(
      new RunCommand(() -> drive.drive(0.5, 0, 0, false), drive).withTimeout(1),
      new AutoChargeStationBalance(drive)
    );
  }
}
