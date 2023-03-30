package frc.robot.commands.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Cal;
import frc.robot.commands.IntakeSequence;
import frc.robot.commands.SwerveToPointWrapper;
import frc.robot.commands.autos.components.DriveDistance;
import frc.robot.commands.autos.components.DriveUntilBalanced;
import frc.robot.commands.autos.components.ScoreThisGamePiece;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeLimelight;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.ScoringLocationUtil;
import java.util.Optional;

/**
 * Assuming the robot is starting from cone scoring position furthest from the loading zone, this
 * auto drives to the nearest midfield game object, picks it up, and balances from the far side.
 */
public class OneFiveBalanceBump extends SequentialCommandGroup {
  private static final double NORM_SPEED_INTAKING = 0.3;
  private double X_METERS_TO_CONE = 1.35;
  private Pose2d desiredPose = new Pose2d(5.85, 2.5, Rotation2d.fromDegrees(0.0));

  private PathPlannerTrajectory firstTraj =
      PathPlanner.loadPath(
          "OneFivePlusBump",
          new PathConstraints(
              Cal.SwerveSubsystem.SLOW_LINEAR_SPEED_METERS_PER_SEC,
              Cal.SwerveSubsystem.SLOW_LINEAR_ACCELERATION_METERS_PER_SEC_SQ));

  public OneFiveBalanceBump(
      boolean red,
      boolean fast,
      Lift lift,
      Intake intake,
      DriveSubsystem drive,
      Lights lights,
      IntakeLimelight limelight,
      ScoringLocationUtil scoringLocationUtil) {
    if (red) {
      // default to blue, only change for red
      firstTraj =
          PathPlanner.loadPath(
              "OneFivePlusBumpRed",
              new PathConstraints(
                  Cal.SwerveSubsystem.SLOW_LINEAR_SPEED_METERS_PER_SEC,
                  Cal.SwerveSubsystem.SLOW_LINEAR_ACCELERATION_METERS_PER_SEC_SQ));

      firstTraj =
          PathPlannerTrajectory.transformTrajectoryForAlliance(
              firstTraj, DriverStation.Alliance.Red);
    }

    addRequirements(lift, intake, drive, limelight);

    /** Initialize sequential commands that run for the "15 second autonomous phase" */
    addCommands(
        new ScoreThisGamePiece(fast, lift, lights),
        drive.followTrajectoryCommand(firstTraj, true),
        // Turn to cone, intake it
        new InstantCommand( //TODO this command is used a number of times, clean up after Buckeye
            () -> {
              Optional<Double> coneAngleDeg = limelight.getAngleToConeDeg();
              drive.offsetCurrentHeading(coneAngleDeg.isPresent() ? coneAngleDeg.get() : 0.0);
            }),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                drive.turnInPlace(0.8),
                new DriveDistance(drive, NORM_SPEED_INTAKING, X_METERS_TO_CONE, 0.0, false)),
            IntakeSequence.interruptibleIntakeSequence(intake, lift, lights)),
        new SwerveToPointWrapper(red, drive, () -> desiredPose, 2.0, 2.0),
        new DriveUntilBalanced(drive, false));
  }
}
