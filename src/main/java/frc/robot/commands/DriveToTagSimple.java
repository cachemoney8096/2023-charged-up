package frc.robot.commands;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TagLimelightV2;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.Optional;

public class DriveToTagSimple extends CommandBase {

  private DriveSubsystem drive;
  private TagLimelightV2 tagLimelight;
  private boolean targetLocked = false;
  private Command followTrajectoryCommand = null;

  public DriveToTagSimple(TagLimelightV2 limelight, DriveSubsystem driveSubsystem) {
    // Note: does not require the drive subsystem itself! It will schedule a command that will do
    // the driving.
    drive = driveSubsystem;
    tagLimelight = limelight;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (!targetLocked) {
      Optional<Transform2d> robotToScoringLocation = tagLimelight.checkForTag();
      if (!robotToScoringLocation.isPresent()) {
        robotToScoringLocation = Optional.of(new Transform2d());
      }
      targetLocked = true;
      followTrajectoryCommand = drive.transformToPath(robotToScoringLocation.get());
      followTrajectoryCommand.schedule();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (targetLocked) {
      followTrajectoryCommand.cancel();
    }
  }

  @Override
  public boolean isFinished() {
    if (followTrajectoryCommand == null) {
      return false;
    }
    return followTrajectoryCommand.isFinished();
  }
}
