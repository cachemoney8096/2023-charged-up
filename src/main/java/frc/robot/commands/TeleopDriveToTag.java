package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.LightCode;
import frc.robot.subsystems.TagLimelightV2;
import frc.robot.subsystems.drive.DriveSubsystem;

public class TeleopDriveToTag extends SequentialCommandGroup {

  public TeleopDriveToTag(TagLimelightV2 limelight, DriveSubsystem drive, Lights lights) {
    addCommands(
        new LookForTag(limelight, drive, lights),
        new SwerveFollowerWrapper(drive).withTimeout(2.0).asProxy(),
        new InstantCommand(
            () -> {
              lights.setLight(LightCode.READY_TO_SCORE);
            }));
  }
}
