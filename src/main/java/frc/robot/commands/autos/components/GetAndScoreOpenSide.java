package frc.robot.commands.autos.components;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Cal;
import frc.robot.commands.IntakeSequence;
import frc.robot.commands.LookForTag;
import frc.robot.commands.SwerveFollowerWrapper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.TagLimelightV2;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.ScoringLocationUtil;
import frc.robot.utils.ScoringLocationUtil.ScoreCol;
import frc.robot.utils.ScoringLocationUtil.ScoreHeight;
import java.util.HashMap;

/**
 * Assuming the robot is starting from cone scoring position nearest the loading zone, this auto
 * component drives to the nearest midfield game object, picks it up, and scores it on the next
 * nearest cone position.
 */
public class GetAndScoreOpenSide extends SequentialCommandGroup {
  private PathPlannerTrajectory trajInit =
      PathPlanner.loadPath(
          "InitScoreAndGetGamePiece",
          new PathConstraints(
              Cal.SwerveSubsystem.MAX_LINEAR_SPEED_METERS_PER_SEC,
              Cal.SwerveSubsystem.MAX_LINEAR_ACCELERATION_METERS_PER_SEC_SQ));

  private HashMap<String, Command> eventMap = new HashMap<>();

  public GetAndScoreOpenSide(
      boolean red,
      boolean fast,
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
    }

    addRequirements(lift, intake, drive, tagLimelight);
    /** Events include: open intake and close intake before and after obtaining game piece */
    eventMap.put(
        "deployIntake",
        new IntakeSequence(intake, lift, lights)
            .finallyDo(
                (boolean interrupted) -> {
                  lift.home();
                  lift.closeGrabber();
                  intake.setDesiredDeployed(false);
                  intake.setDesiredClamped(false);
                  intake.stopIntakingGamePiece();
                }));

    eventMap.put("retractIntake", new InstantCommand(() -> {}, lift, intake));

    /** Initialize sequential commands that run for the "15 second autonomous phase" */
    addCommands(
        new InstantCommand(() -> scoringLocationUtil.setScoreHeight(ScoreHeight.HIGH)),
        new InstantCommand(
            () -> scoringLocationUtil.setScoreCol(red ? ScoreCol.RIGHT : ScoreCol.LEFT)),
        new FollowPathWithEvents(
            drive.followTrajectoryCommand(trajInit, true), trajInit.getMarkers(), eventMap),
        new InstantCommand(drive::setNoMove, drive),
        new LookForTag(tagLimelight, drive, lights).withTimeout(0.05),
        new SwerveFollowerWrapper(drive).withTimeout(2.0),
        drive.stopDrivingCommand(),
        new WaitCommand(0.02),
        new ConditionalCommand(
            new ScoreThisGamePiece(fast, lift, lights),
            new InstantCommand(),
            () -> {
              double yawDeg = drive.getHeadingDegrees();
              boolean shouldScoreByYaw = Math.abs(yawDeg) < 5.0;
              if (!shouldScoreByYaw) {
                System.out.println("Not squared up, don't score");
              }
              if (!drive.generatedPath) {
                System.out.println("Didn't drive to target");
              }
              return shouldScoreByYaw && drive.generatedPath;
            }));
  }
}
