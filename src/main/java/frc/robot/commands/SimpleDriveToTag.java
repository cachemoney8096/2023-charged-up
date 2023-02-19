package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Cal;
import frc.robot.subsystems.TagLimelight;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.ScoringLocationUtil;

public class SimpleDriveToTag extends CommandBase {
  private TagLimelight limelight;
  private DriveSubsystem drive;
  private ScoringLocationUtil scoreLoc;

  public SimpleDriveToTag(
      TagLimelight limelight, DriveSubsystem drive, ScoringLocationUtil scoreLoc) {
    addRequirements(limelight, drive);
    this.limelight = limelight;
    this.drive = drive;
    this.scoreLoc = scoreLoc;
  }

  @Override
  public void execute() {
    if (limelight.isValidTarget()) {
      Pose2d robotPose = drive.getPose();
      double robotAngleRad = robotPose.getRotation().getRadians();

      Translation2d tagPosFromRobot = limelight.getTargetTranslation();

      Translation2d tagPosFromNormalizedRobot =
          new Translation2d(
              Math.cos(robotAngleRad) * tagPosFromRobot.getX()
                  - Math.sin(robotAngleRad) * tagPosFromRobot.getY(),
              Math.sin(robotAngleRad) * tagPosFromRobot.getX()
                  + Math.cos(robotAngleRad) * tagPosFromRobot.getY());

      double horizTranslationFromTag;
      if (scoreLoc.getScoreCol() == ScoringLocationUtil.ScoreCol.RIGHT) {
        horizTranslationFromTag = Cal.HORIZONTAL_DISTANCE_TAG_TO_RIGHT_CONE_METERS;
      } else if (scoreLoc.getScoreCol() == ScoringLocationUtil.ScoreCol.LEFT) {
        horizTranslationFromTag = -Cal.HORIZONTAL_DISTANCE_TAG_TO_RIGHT_CONE_METERS;
      } else {
        horizTranslationFromTag = 0;
      }

      double backTranslationFromTag;
      if (scoreLoc.getScoreHeight() == ScoringLocationUtil.ScoreHeight.LOW){
        backTranslationFromTag = Cal.DISTANCE_BACK_FROM_TAG_LOW_METERS;
      } else {
        backTranslationFromTag = Cal.DISTANCE_BACK_FROM_TAG_MID_HIGH_METERS;
      }

      Translation2d locationToDriveTo =
          new Translation2d(
              tagPosFromNormalizedRobot.getX() + backTranslationFromTag,
              tagPosFromNormalizedRobot.getY() + horizTranslationFromTag);

      double angleToRotate = -robotPose.getRotation().getDegrees();

      PathPlannerTrajectory robotToTag =
          PathPlanner.generatePath(
              new PathConstraints(
                  Cal.SwerveSubsystem.MAX_LINEAR_SPEED_METERS_PER_SEC,
                  Cal.SwerveSubsystem.MAX_LINEAR_ACCELERATION_METERS_PER_SEC_SQ),
              new PathPoint(robotPose.getTranslation(), robotPose.getRotation()),
              new PathPoint(
                  locationToDriveTo,
                  Rotation2d.fromDegrees(180),
                  Rotation2d.fromDegrees(angleToRotate)));
      drive.followTrajectoryCommand(robotToTag, false);
    }
  }
}
