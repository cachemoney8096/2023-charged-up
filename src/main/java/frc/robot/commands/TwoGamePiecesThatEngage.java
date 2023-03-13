package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Cal;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.LiftPosition;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.LightCode;
import frc.robot.subsystems.TagLimelightV2;
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

  private static final double DISTANCE_UP_CHARGE_STATION_METERS = 1.6;

  public TwoGamePiecesThatEngage(
      boolean red,
      Lift lift,
      Intake intake,
      DriveSubsystem drive,
      Lights lights,
      TagLimelightV2 tagLimelight,
      ScoringLocationUtil scoringLocationUtil) {
    if (red) {
      // default to blue, only change for red
      trajInit =
          PathPlanner.loadPath(
              "InitScoreAndGetGamePieceRed",
              new PathConstraints(
                  Cal.SwerveSubsystem.MAX_LINEAR_SPEED_METERS_PER_SEC,
                  Cal.SwerveSubsystem.MAX_LINEAR_ACCELERATION_METERS_PER_SEC_SQ));

      trajInit =
          PathPlannerTrajectory.transformTrajectoryForAlliance(
              trajInit, DriverStation.Alliance.Red);

      trajCharge =
          PathPlanner.loadPath(
              "ScoringLocToChargeStationRed",
              new PathConstraints(
                  Cal.SwerveSubsystem.MAX_LINEAR_SPEED_METERS_PER_SEC,
                  Cal.SwerveSubsystem.MAX_LINEAR_ACCELERATION_METERS_PER_SEC_SQ));

      trajCharge =
          PathPlannerTrajectory.transformTrajectoryForAlliance(
              trajCharge, DriverStation.Alliance.Red);
    }

    addRequirements(lift, intake, drive, tagLimelight);
    /** Events include: open intake and close intake before and after obtaining game piece */
    eventMap.put(
        "deployIntake",
        new IntakeSequence(intake, lift, lights)
            .withTimeout(2.0)
            .finallyDo(
                (boolean interrupted) -> {
                  lift.home();
                  lift.closeGrabber();
                  intake.setDesiredDeployed(false);
                  intake.setDesiredClamped(false);
                  intake.stopIntakingGamePiece();
                }));
    eventMap.put("limelight", new LookForTag(tagLimelight, drive, lights).withTimeout(2.0));

    /** Initialize sequential commands that run for the "15 second autonomous phase" */
    addCommands(
        new InstantCommand(
            () -> scoringLocationUtil.setScoreCol(red ? ScoreCol.LEFT : ScoreCol.RIGHT)),
        new InstantCommand(() -> scoringLocationUtil.setScoreHeight(ScoreHeight.HIGH)),
        new InstantCommand(() -> lift.ManualPrepScoreSequence(lights), lift),
        new WaitUntilCommand(() -> lift.atPosition(LiftPosition.PRE_SCORE_HIGH_CONE))
            .withTimeout(Cal.Lift.START_TO_PRESCORE_HIGH_SEC),
        new InstantCommand(
            () -> {
              lights.toggleCode(LightCode.READY_TO_SCORE);
            }),
        new InstantCommand(lift::startScore, lift),
        new WaitUntilCommand(() -> lift.atPosition(LiftPosition.SCORE_HIGH_CONE))
            .withTimeout(Cal.Lift.PRESCORE_TO_SCORE_SEC),
        new finishScore(lift, lights),
        new InstantCommand(
            () -> scoringLocationUtil.setScoreCol(red ? ScoreCol.RIGHT : ScoreCol.LEFT)),
        new WaitCommand(Cal.Lift.SCORE_TO_START_FAST_SEC), // going to start
        new FollowPathWithEvents(
            drive.followTrajectoryCommand(trajInit, true), trajInit.getMarkers(), eventMap),
        new SwerveFollowerWrapper(red, drive)
            .withTimeout(2.0)
            .finallyDo(
                (boolean interrupted) -> {
                  if (interrupted) {
                    drive.drive(0, 0, 0, true);
                  }
                }),
        new InstantCommand(
            () -> {
              lights.toggleCode(LightCode.READY_TO_SCORE);
            }),
        new InstantCommand(() -> lift.ManualPrepScoreSequence(lights), lift),
        new WaitUntilCommand(() -> lift.atPosition(LiftPosition.PRE_SCORE_HIGH_CONE))
            .withTimeout(Cal.Lift.START_TO_PRESCORE_HIGH_SEC),
        new InstantCommand(lift::startScore, lift),
        new WaitUntilCommand(() -> lift.atPosition(LiftPosition.SCORE_HIGH_CONE))
            .withTimeout(Cal.Lift.PRESCORE_TO_SCORE_SEC),
        new finishScore(lift, lights),
        new WaitCommand(Cal.Lift.SCORE_TO_START_FAST_SEC),
        drive.followTrajectoryCommand(trajCharge, false).withTimeout(2.0),
        new InstantCommand(
            () -> {
              drive.resetYaw();
            }),
        new AutoChargeStationSequence(drive, DISTANCE_UP_CHARGE_STATION_METERS));
  }
}
