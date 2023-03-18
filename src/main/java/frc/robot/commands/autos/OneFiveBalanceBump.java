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
import frc.robot.commands.autos.components.ChargeFarSide;
import frc.robot.commands.autos.components.ScoreThisGamePiece;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.TagLimelightV2;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.ScoringLocationUtil;
import java.util.HashMap;

/**
 * Assuming the robot is starting from cone scoring position nearest the loading zone, this auto
 * component drives to the nearest midfield game object, picks it up, and scores it on the next
 * nearest cone position.
 */
public class OneFiveBalanceBump extends SequentialCommandGroup {
  private PathPlannerTrajectory trajInit =
      PathPlanner.loadPath(
          "OnePlusBump",
          new PathConstraints(
              Cal.SwerveSubsystem.VERY_SLOW_LINEAR_SPEED_METERS_PER_SEC,
              Cal.SwerveSubsystem.VERY_SLOW_LINEAR_ACCELERATION_METERS_PER_SEC_SQ));

  private HashMap<String, Command> eventMap = new HashMap<>();

  public OneFiveBalanceBump(
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
              "OnePlusBumpRed",
              new PathConstraints(
                  Cal.SwerveSubsystem.VERY_SLOW_LINEAR_SPEED_METERS_PER_SEC,
                  Cal.SwerveSubsystem.VERY_SLOW_LINEAR_ACCELERATION_METERS_PER_SEC_SQ));

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
    // eventMap.put("limelight", new LookForTag(tagLimelight, drive, lights).withTimeout(1.0));

    eventMap.put("retractIntake", new InstantCommand(() -> {}, lift, intake));

    final double DISTANCE_ONTO_CHARGE_STATION_METERS = 1.4;

    /** Initialize sequential commands that run for the "15 second autonomous phase" */
    addCommands(
        new ScoreThisGamePiece(fast, lift, lights),
        new FollowPathWithEvents(
            drive.followTrajectoryCommand(trajInit, true), trajInit.getMarkers(), eventMap),
        new ChargeFarSide(drive, DISTANCE_ONTO_CHARGE_STATION_METERS));
  }
}
