package frc.robot.commands;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TagLimelightV2;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.Optional;

import com.pathplanner.lib.PathPlannerTrajectory;

public class DriveToTagSimple extends CommandBase {

  public interface TrajectorySetterInterface {
    void setTrajectory(PathPlannerTrajectory path);
}

  private DriveSubsystem drive;
  private TagLimelightV2 tagLimelight;
  private boolean targetLocked = false;
  private TrajectorySetterInterface trajectorySetterFunc;

  public DriveToTagSimple(TagLimelightV2 limelight, DriveSubsystem driveSubsystem, TrajectorySetterInterface trajectorySetter) {
    // Note: does not require the drive subsystem itself! It will schedule a command that will do
    // the driving.
    drive = driveSubsystem;
    tagLimelight = limelight;
    trajectorySetterFunc = trajectorySetter;
  }

  @Override
  public void initialize() { }

  @Override
  public void execute() {
    if (!targetLocked) {
      Optional<Transform2d> robotToScoringLocation = tagLimelight.checkForTag();
      if (!robotToScoringLocation.isPresent()) {
        robotToScoringLocation = Optional.of(new Transform2d());
      }
      targetLocked = true;
      trajectorySetterFunc.setTrajectory(drive.transformToPath(robotToScoringLocation.get()));
    }
  }

  @Override
  public void end(boolean interrupted) { }

  @Override
  public boolean isFinished() {
    return targetLocked;
  }
}
