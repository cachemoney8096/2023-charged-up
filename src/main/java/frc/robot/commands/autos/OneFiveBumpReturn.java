package frc.robot.commands.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Cal;
import frc.robot.commands.IntakeSequence;
import frc.robot.commands.autos.components.DriveDistance;
import frc.robot.commands.autos.components.ScoreThisGamePiece;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeLimelight;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.ScoringLocationUtil;

/**
 * Assuming the robot is starting from cone scoring position nearest the loading zone, this auto
 * component drives to the nearest midfield game object, picks it up, and returns to starting
 * position
 */
public class OneFiveBumpReturn extends SequentialCommandGroup {
  private static final double DISTANCE_AT_CONE_METERS = 5.9;
  private static final double NORM_SPEED_INTAKING = 0.3;
  private double startXMeters = 0;
  private PathPlannerTrajectory firstTraj =
      PathPlanner.loadPath(
          "OnePlusBump",
          new PathConstraints(
              Cal.SwerveSubsystem.VERY_SLOW_LINEAR_SPEED_METERS_PER_SEC,
              Cal.SwerveSubsystem.VERY_SLOW_LINEAR_ACCELERATION_METERS_PER_SEC_SQ));
  private PathPlannerTrajectory secondTraj =
      PathPlanner.loadPath(
          "OnePlusBumpReturn",
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
      ScoringLocationUtil scoringLocationUtil) {
    if (red) {
      // default to blue, only change for red
      firstTraj =
          PathPlanner.loadPath(
              "OnePlusBumpRed",
              new PathConstraints(
                  Cal.SwerveSubsystem.VERY_SLOW_LINEAR_SPEED_METERS_PER_SEC,
                  Cal.SwerveSubsystem.VERY_SLOW_LINEAR_ACCELERATION_METERS_PER_SEC_SQ));

      secondTraj =
          PathPlanner.loadPath(
              "OnePlusBumpReturnRed",
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
        new InstantCommand(
            () -> {
              startXMeters = drive.getPose().getX();
            }),
        drive.followTrajectoryCommand(firstTraj, true),
        // Turn to cone, intake it
        new InstantCommand(
            () -> {
              drive.offsetCurrentHeading(limelight.getAngleToConeDeg());
            }),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                new DriveDistance(drive, NORM_SPEED_INTAKING, 1.5, 0.0, red),
                new IntakeSequence(intake, lift, lights)
                    .finallyDo(
                        (boolean interrupted) -> {
                          lift.home();
                          lift.closeGrabber();
                          intake.setDesiredDeployed(false);
                          intake.setDesiredClamped(false);
                          intake.stopIntakingGamePiece();
                        })),
            new DriveDistance(drive, NORM_SPEED_INTAKING, -1.5, 0.0, red),
            drive.followTrajectoryCommand(secondTraj, false)));
  }
}
