package frc.robot.commands.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Cal;
import frc.robot.commands.IntakeSequence;
import frc.robot.commands.autos.components.GetAndScoreOpenSide;
import frc.robot.commands.autos.components.ScoreThisGamePiece;
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
 * This auto assumes a start at the cone scoring position nearest the loading zone. It scores a
 * cone, grabs another, and scores that one too.
 */
public class TwoFiveOpenSide extends SequentialCommandGroup {
  private PathPlannerTrajectory headstart =
      PathPlanner.loadPath(
          "HeadStart",
          new PathConstraints(
              Cal.SwerveSubsystem.MAX_LINEAR_SPEED_METERS_PER_SEC,
              Cal.SwerveSubsystem.MAX_LINEAR_ACCELERATION_METERS_PER_SEC_SQ));

  private HashMap<String, Command> eventMap = new HashMap<>();

  public TwoFiveOpenSide(
      boolean red,
      Lift lift,
      Intake intake,
      DriveSubsystem drive,
      Lights lights,
      TagLimelightV2 tagLimelight,
      ScoringLocationUtil scoringLocationUtil) {

    if (red) {
      headstart =
          PathPlanner.loadPath(
              "HeadStartRed",
              new PathConstraints(
                  Cal.SwerveSubsystem.MAX_LINEAR_SPEED_METERS_PER_SEC,
                  Cal.SwerveSubsystem.MAX_LINEAR_ACCELERATION_METERS_PER_SEC_SQ));

      headstart =
          PathPlannerTrajectory.transformTrajectoryForAlliance(
              headstart, DriverStation.Alliance.Red);
    }

    /** Events include: deploy and retract intake */
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

    addRequirements(lift, intake, drive, tagLimelight);
    /** Initialize sequential commands that run for the "15 second autonomous phase" */
    addCommands(
        new InstantCommand(
            () -> scoringLocationUtil.setScoreCol(red ? ScoreCol.LEFT : ScoreCol.RIGHT)),
        new InstantCommand(() -> scoringLocationUtil.setScoreHeight(ScoreHeight.HIGH)),
        new ScoreThisGamePiece(false, lift, lights),
        new GetAndScoreOpenSide(
            red, false, lift, intake, drive, lights, tagLimelight, scoringLocationUtil),
        new FollowPathWithEvents(
                drive.followTrajectoryCommand(headstart, true), headstart.getMarkers(), eventMap)
            .withTimeout(4.0));
  }
}
