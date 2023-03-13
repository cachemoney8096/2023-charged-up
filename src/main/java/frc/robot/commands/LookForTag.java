package frc.robot.commands;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.LightCode;
import frc.robot.subsystems.TagLimelightV2;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.Optional;

public class LookForTag extends CommandBase {
  private DriveSubsystem drive;
  private TagLimelightV2 tagLimelight;
  private Lights lights;
  private boolean targetLocked = false;

  public LookForTag(
      TagLimelightV2 limelight, DriveSubsystem driveSubsystem, Lights lightsSubsysten) {
    // Note: does not require the drive subsystem itself! It just sets the final point in the drive.
    drive = driveSubsystem;
    tagLimelight = limelight;
    lights = lightsSubsysten;
  }

  @Override
  public void initialize() {
    lights.toggleCode(LightCode.NO_TAG);
  }

  @Override
  public void execute() {
    if (!targetLocked) {
      Optional<Transform2d> robotToScoringLocation = tagLimelight.checkForTag();
      if (!robotToScoringLocation.isPresent()) {
        robotToScoringLocation = Optional.empty();
        return;
      }
      double latencySeconds = tagLimelight.getLatencySeconds();
      targetLocked = true;
      drive.setLimelightTargetFromTransform(robotToScoringLocation.get(), latencySeconds);
      lights.toggleCode(LightCode.WORKING);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return targetLocked;
  }
}
