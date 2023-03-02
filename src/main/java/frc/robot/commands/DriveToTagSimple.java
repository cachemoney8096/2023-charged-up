package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.ScoringLocationUtil;
import frc.robot.utils.LimelightHelpers.LimelightResults;
import frc.robot.utils.LimelightHelpers.LimelightTarget_Fiducial;

public class DriveToTagSimple extends CommandBase {

  private ScoringLocationUtil scoreLoc;
  private DriveSubsystem drive;
  private boolean targetLocked = false;
  private Command followTrajectoryCommand;

  public DriveToTagSimple(ScoringLocationUtil scoreLocationUtil, DriveSubsystem driveSubsystem) {
    // Note: does not require the drive subsystem itself! It will schedule a command that will do the driving.
    scoreLoc = scoreLocationUtil;
    drive = driveSubsystem;
  }

  private static Transform2d getBotFromTarget(Pose3d botPoseTargetSpace) {
    /**
     * Target space:
     * 3d Cartesian Coordinate System with (0,0,0) at the center of the target.
     * <p>X+ → Pointing to the right of the target (If you are looking at the target)
     * <p>Y+ → Pointing downward
     * <p>Z+ → Pointing out of the target (orthogonal to target’s plane).
     */

    /**
     * We convert to 2d target space:
     * X+ -> Out of the target
     * Y+ -> Pointing to the right of the target (If you are looking at the target)
     * This means positive yaw is based on Z+ being up
     */
    Translation2d translation =
        new Translation2d(
          botPoseTargetSpace.getZ(),
          botPoseTargetSpace.getX());
    Rotation2d rot = Rotation2d.fromDegrees(-botPoseTargetSpace.getRotation().getY());
    return new Transform2d(translation, rot);
  }

  private Transform2d getRobotToScoringLocation(Pose3d botPoseTargetSpace) {
    Transform2d botFromTarget = getBotFromTarget(botPoseTargetSpace);
        Transform2d scoringLocationFromTag = scoreLoc.scoringLocationFromTag();
    return botFromTarget.inverse().plus(scoringLocationFromTag);
  }

  private static boolean validScoringTag(double tagId) {
    long tagIdRounded = Math.round(tagId);
    if (tagIdRounded == 1 || tagIdRounded == 2 || tagIdRounded == 3 || tagIdRounded == 6 || tagIdRounded == 7 || tagIdRounded == 8) {
      return true;
    }
    else {
      return false;
    }
  }

  private int chooseTag(LimelightTarget_Fiducial[] targets) {
    int numTags = targets.length;
    double minDistMeters = Double.MAX_VALUE;
    int bestTag = 0;
    for (int tagIndex = 0; tagIndex < numTags; tagIndex++) {
      LimelightTarget_Fiducial target = targets[tagIndex];
      if (!validScoringTag(target.fiducialID)) {
        continue;
      }
      double targetDistance = target.getRobotPose_TargetSpace2D().getTranslation().getNorm();
      if (targetDistance < minDistMeters) {
        minDistMeters = targetDistance;
        bestTag = tagIndex;
      }
    }
    return bestTag;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (!targetLocked) {
      LimelightResults results = LimelightHelpers.getLatestResults("");
      LimelightTarget_Fiducial[] targets = results.targetingResults.targets_Fiducials;
      int numAprilTags = targets.length;
      if (numAprilTags < 2) {
        return;
      }
      targetLocked = true;
      int bestTagIndex = chooseTag(targets);
      LimelightTarget_Fiducial bestTarget = targets[bestTagIndex];
      Transform2d robotToScoringLocation = getRobotToScoringLocation(bestTarget.getRobotPose_TargetSpace());
      followTrajectoryCommand = drive.transformToPath(robotToScoringLocation);
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
    return targetLocked;
  }
}
