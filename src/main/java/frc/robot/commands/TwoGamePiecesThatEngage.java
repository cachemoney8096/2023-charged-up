package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Cal;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lift.LiftPosition;
import frc.robot.subsystems.Lights.LightCode;
import frc.robot.subsystems.TagLimelight;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.ScoringLocationUtil;
import frc.robot.utils.ScoringLocationUtil.ScoreCol;
import frc.robot.utils.ScoringLocationUtil.ScoreHeight;
import java.util.HashMap;

/**
 * This does the following: Robot starts at right postion at left grid Score the game piece at the
 * high right position on the left grid Drive and retreive another game piece from the center Bring
 * it back to the left grid left side, and score at the high left position Go to charing station and
 * do the charge station balance sequence
 */
public class TwoGamePiecesThatEngage extends SequentialCommandGroup {
  private PathPlannerTrajectory trajInit =
      PathPlanner.loadPath(
          "InitScoreAndGetGamePiece",
          new PathConstraints(
              Cal.SwerveSubsystem.MAX_LINEAR_SPEED_METERS_PER_SEC,
              Cal.SwerveSubsystem.MAX_LINEAR_ACCELERATION_METERS_PER_SEC_SQ));

  private PathPlannerTrajectory trajCharge =
      PathPlanner.loadPath(
          "ScoringLocToChargeStation",
          new PathConstraints(
              Cal.SwerveSubsystem.MAX_LINEAR_SPEED_METERS_PER_SEC,
              Cal.SwerveSubsystem.MAX_LINEAR_ACCELERATION_METERS_PER_SEC_SQ));

  private HashMap<String, Command> eventMap = new HashMap<>();

  public TwoGamePiecesThatEngage(
      Lift lift,
      Intake intake,
      DriveSubsystem drive,
      Lights lights,
      TagLimelight tagLimelight,
      ScoringLocationUtil scoringLocationUtil) {
    addRequirements(lift, intake, drive, tagLimelight);
    /** Events include: open intake and close intake before and after obtaining game piece */
    eventMap.put(
        "deployIntake",
        new InstantCommand(
            () -> {
              intake.setDesiredDeployed(true);
            },
            intake));
    eventMap.put(
        "closeIntake",
        new InstantCommand(
            () -> {
              intake.setDesiredDeployed(false);
            },
            intake));
    /**
     * TODO: configure limelight to "take over" driving process after certain point AFTER obtaining
     * game piece to be more accurate with distance
     * if limelight fails to find valid target/arpil tag, then turn on NO_TAG light
     */

    /** Initialize sequential commands that run for the "15 second autonomous phase" */
    addCommands(
        new InstantCommand(() -> scoringLocationUtil.setScoreCol(ScoreCol.RIGHT)),
        new InstantCommand(() -> scoringLocationUtil.setScoreHeight(ScoreHeight.HIGH)),
        new InstantCommand(() -> lift.ManualPrepScoreSequence(lights), lift),
        new WaitUntilCommand(() -> lift.atPosition(LiftPosition.PRE_SCORE_HIGH_CONE)),
        new InstantCommand(lift::startScore, lift),
        new WaitUntilCommand(() -> lift.atPosition(LiftPosition.SCORE_HIGH_CONE)),
        new finishScore(lift, lights),
        new InstantCommand(() -> scoringLocationUtil.setScoreCol(ScoreCol.LEFT)),
        new WaitUntilCommand(() -> lift.atPosition(LiftPosition.STARTING)),
        new FollowPathWithEvents(
            drive.followTrajectoryCommand(trajInit, true), trajInit.getMarkers(), eventMap),
        /** TODO: Limelight code goes here */
        new InstantCommand(()->{if(tagLimelight.getValidTarget() == 0){lights.toggleCode(LightCode.NO_TAG);}}),
        new InstantCommand(() -> lift.ManualPrepScoreSequence(lights), lift),
        new WaitUntilCommand(() -> lift.atPosition(LiftPosition.PRE_SCORE_HIGH_CONE)),
        new InstantCommand(lift::startScore, lift),
        new WaitUntilCommand(() -> lift.atPosition(LiftPosition.SCORE_HIGH_CONE)),
        new finishScore(lift, lights),
        new WaitUntilCommand(() -> lift.atPosition(LiftPosition.STARTING)),
        drive.followTrajectoryCommand(
            trajCharge, false), // this does not accept the FollowPathWithEvents
        new AutoChargeStationBalance(drive));
  }
}
