package frc.robot.subsystems;

import java.util.Optional;

import javax.xml.crypto.dsig.Transform;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.ScoringLocationUtil;
import frc.robot.utils.LimelightHelpers.LimelightResults;
import frc.robot.utils.LimelightHelpers.LimelightTarget_Fiducial;

public class TagLimelightV2 extends SubsystemBase {
  private ScoringLocationUtil scoreLoc;

  private Optional<Transform2d> robotToScoringLocation = Optional.empty();

  public TagLimelightV2(ScoringLocationUtil scoringLocUtil) {
    scoreLoc = scoringLocUtil;
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

  private static boolean validScoringTag(double tagId) {
    long tagIdRounded = Math.round(tagId);
    if (tagIdRounded == 1 || tagIdRounded == 2 || tagIdRounded == 3 || tagIdRounded == 6 || tagIdRounded == 7 || tagIdRounded == 8) {
      return true;
    }
    else {
      return false;
    }
  }

  public static int chooseTag(LimelightTarget_Fiducial[] targets) {
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

  private Transform2d getRobotToScoringLocation(Pose3d botPoseTargetSpace) {
    Transform2d botFromTarget = getBotFromTarget(botPoseTargetSpace);
        Transform2d scoringLocationFromTag = scoreLoc.scoringLocationFromTag();
    return botFromTarget.inverse().plus(scoringLocationFromTag);
  }

  public Optional<Transform2d> getRobotToScoringLocation() {
    return robotToScoringLocation;
  }

  @Override
  public void periodic() {
    LimelightResults results = LimelightHelpers.getLatestResults("");
    LimelightTarget_Fiducial[] targets = results.targetingResults.targets_Fiducials;
    int numAprilTags = targets.length;
    if (numAprilTags < 2) {
      robotToScoringLocation = Optional.empty();
      return;
    }
    int bestTagIndex = chooseTag(targets);
    LimelightTarget_Fiducial bestTarget = targets[bestTagIndex];
    robotToScoringLocation = Optional.of(getRobotToScoringLocation(bestTarget.getRobotPose_TargetSpace()));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty("Translation X (m)",
    () -> {
      if (robotToScoringLocation.isPresent()) {
        return String.valueOf(robotToScoringLocation.get().getX());
      }
      else {
        return "empty";
      }
      }, null);
      builder.addStringProperty("Translation Y (m)",
      () -> {
        if (robotToScoringLocation.isPresent()) {
          return String.valueOf(robotToScoringLocation.get().getY());
        }
        else {
          return "empty";
        }
        }, null);
        builder.addStringProperty("Rotation (deg)",
        () -> {
          if (robotToScoringLocation.isPresent()) {
            return String.valueOf(robotToScoringLocation.get().getRotation().getDegrees());
          }
          else {
            return "empty";
          }
          }, null);
  }
}
