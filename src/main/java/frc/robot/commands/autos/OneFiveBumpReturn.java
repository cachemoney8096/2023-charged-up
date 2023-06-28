package frc.robot.commands.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Cal;
import frc.robot.commands.IntakeSequence;
import frc.robot.commands.LookForTag;
import frc.robot.commands.SwerveFollowerWrapper;
import frc.robot.commands.autos.components.DriveDistance;
import frc.robot.commands.autos.components.ScoreThisGamePiece;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeLimelight;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.TagLimelightV2;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.ScoringLocationUtil;
import frc.robot.utils.ScoringLocationUtil.ScoreCol;
import java.util.Optional;

/**
 * Assuming the robot is starting from cone scoring position furthest from the loading zone, this
 * auto drives to the nearest midfield game object, picks it up, and returns.
 */
public class OneFiveBumpReturn extends SequentialCommandGroup {
  private static final boolean TRY_TO_SCORE = true;
  private static final double NORM_SPEED_INTAKING = 0.3;
  private static final double X_METERS_TO_CONE = 1.35;
  private PathPlannerTrajectory firstTraj =
      PathPlanner.loadPath(
          "OneFivePlusBump",
          new PathConstraints(
              Cal.SwerveSubsystem.SLOW_LINEAR_SPEED_METERS_PER_SEC,
              Cal.SwerveSubsystem.SLOW_LINEAR_ACCELERATION_METERS_PER_SEC_SQ));
  private PathPlannerTrajectory secondTraj =
      PathPlanner.loadPath(
          "OneFivePlusBumpReturn",
          new PathConstraints(
              Cal.SwerveSubsystem.VERY_SLOW_LINEAR_SPEED_METERS_PER_SEC,
              Cal.SwerveSubsystem.VERY_SLOW_LINEAR_ACCELERATION_METERS_PER_SEC_SQ));

  public OneFiveBumpReturn(
      boolean red,
      boolean fast,
      Lift lift,
      Intake intake,
      DriveSubsystem drive,
      Lights lights,
      IntakeLimelight limelight,
      TagLimelightV2 tagLimelight,
      ScoringLocationUtil scoringLocationUtil) {
    if (red) {
      // default to blue, only change for red
      firstTraj =
          PathPlanner.loadPath(
              "OneFivePlusBumpRed",
              new PathConstraints(
                  Cal.SwerveSubsystem.SLOW_LINEAR_SPEED_METERS_PER_SEC,
                  Cal.SwerveSubsystem.SLOW_LINEAR_ACCELERATION_METERS_PER_SEC_SQ));

      secondTraj =
          PathPlanner.loadPath(
              "OneFivePlusBumpReturnRed",
              new PathConstraints(
                  Cal.SwerveSubsystem.VERY_SLOW_LINEAR_SPEED_METERS_PER_SEC,
                  Cal.SwerveSubsystem.VERY_SLOW_LINEAR_ACCELERATION_METERS_PER_SEC_SQ));

      firstTraj =
          PathPlannerTrajectory.transformTrajectoryForAlliance(
              firstTraj, DriverStation.Alliance.Red);

      secondTraj =
          PathPlannerTrajectory.transformTrajectoryForAlliance(
              secondTraj, DriverStation.Alliance.Red);
    }

    addRequirements(lift, intake, drive, limelight);

    /** Initialize sequential commands that run for the "15 second autonomous phase" */
    addCommands(
        new ScoreThisGamePiece(fast, lift, lights),
        drive.followTrajectoryCommand(firstTraj, true),
        // Turn to cone, intake it
        new InstantCommand(
            () -> {
              Optional<Double> coneAngleDeg = limelight.getAngleToConeDeg();
              drive.offsetCurrentHeading(coneAngleDeg.isPresent() ? coneAngleDeg.get() : 0.0);
            }),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                drive.turnInPlace(0.6),
                new DriveDistance(drive, NORM_SPEED_INTAKING, X_METERS_TO_CONE, 0.0, red)),
            IntakeSequence.interruptibleIntakeSequence(intake, lift, lights)),
        // new DriveDistance(drive, NORM_SPEED_I NTAKING, -X_METERS_TO_CONE, 0.0, red),
        new InstantCommand(
            () -> scoringLocationUtil.setScoreCol(red ? ScoreCol.LEFT : ScoreCol.RIGHT)),
        drive.followTrajectoryCommand(secondTraj, false),
        new InstantCommand(drive::setNoMove, drive),
        new InstantCommand(() -> drive.setZeroTargetHeading()),
        new InstantCommand(()-> drive.turnInPlace(0.75)),
        new InstantCommand(()-> drive.setNoMove()),
        new WaitCommand(0.1),
        new LookForTag(tagLimelight, drive, lights).withTimeout(0.05),
        new SwerveFollowerWrapper(drive)
            .finallyDo(
                (boolean interrupted) -> {
                  System.out.println("Driving to tag interrupted? " + interrupted);
                }),
        drive.stopDrivingCommand(),
        new ConditionalCommand(
            // Score the game piece
            new SequentialCommandGroup(
                new WaitCommand(0.02), new ScoreThisGamePiece(fast, lift, lights)),
            // Do nothing
            new InstantCommand(),
            () -> TRY_TO_SCORE));
  }
}
