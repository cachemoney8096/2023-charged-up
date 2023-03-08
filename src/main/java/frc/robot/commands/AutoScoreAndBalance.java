package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Cal;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.LiftPosition;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.ScoringLocationUtil;

/** Scores a game piece, then drives onto the charge station and balances */
public class AutoScoreAndBalance extends SequentialCommandGroup {
  private static final double DISTANCE_UP_CHARGE_STATION_METERS = 2.21;

  private PathPlannerTrajectory traj =
      PathPlanner.loadPath(
          "ScoreAndBalanceTraj",
          new PathConstraints(
              Cal.SwerveSubsystem.MAX_LINEAR_SPEED_METERS_PER_SEC,
              Cal.SwerveSubsystem.MAX_LINEAR_ACCELERATION_METERS_PER_SEC_SQ));

  public AutoScoreAndBalance(
      Lift lift,
      DriveSubsystem drive,
      Lights lights,
      ScoringLocationUtil scoringLocationUtil) {
    addCommands(
      // new InstantCommand(() -> lift.ManualPrepScoreSequence(lights), lift),
      // new WaitUntilCommand(() -> lift.atPosition(LiftPosition.PRE_SCORE_HIGH_CONE)),
      new InstantCommand(lift::startScore, lift),
      new WaitUntilCommand(() -> lift.atPosition(LiftPosition.SCORE_HIGH_CONE)),
      new finishScore(lift, lights),
      new WaitUntilCommand(() -> lift.atPosition(LiftPosition.STARTING)),
      // Note: even if we're red, the robot is never flipped so we can just pass false here:
      new AutoChargeStationSequence(false, drive, DISTANCE_UP_CHARGE_STATION_METERS));
  }

  public PathPlannerTrajectory getTrajectory() {
    return this.traj;
  }
}
