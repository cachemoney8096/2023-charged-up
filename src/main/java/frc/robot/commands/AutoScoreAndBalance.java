package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Cal;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.LiftPosition;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.ScoringLocationUtil.ScoreCol;
import frc.robot.utils.ScoringLocationUtil.ScoreHeight;
import frc.robot.utils.ScoringLocationUtil;

/** Scores a game piece, then drives onto the charge station and balances */
public class AutoScoreAndBalance extends SequentialCommandGroup {
  private PathPlannerTrajectory traj =
      PathPlanner.loadPath(
          "ScoreAndBalanceTraj",
          new PathConstraints(
              Cal.SwerveSubsystem.MAX_LINEAR_SPEED_METERS_PER_SEC,
              Cal.SwerveSubsystem.MAX_LINEAR_ACCELERATION_METERS_PER_SEC_SQ));

  public AutoScoreAndBalance(boolean isFirstPath, Lift lift, DriveSubsystem drive, ScoringLocationUtil scoringLocationUtil) {
    addCommands(
      new InstantCommand(()->scoringLocationUtil.toggleMiddleGrid()), //is this needed??
      new InstantCommand(() -> scoringLocationUtil.setScoreCol(ScoreCol.RIGHT)),
      new InstantCommand(() -> scoringLocationUtil.setScoreHeight(ScoreHeight.HIGH)),
      new InstantCommand(lift::ManualPrepScoreSequence, lift),
      new WaitUntilCommand(() -> lift.atPosition(LiftPosition.PRE_SCORE_HIGH_CONE)),
      new InstantCommand(lift::startScore, lift),
      new WaitUntilCommand(() -> lift.atPosition(LiftPosition.SCORE_HIGH_CONE)),
      new finishScore(lift),
      drive.followTrajectoryCommand(traj, isFirstPath),
      new AutoChargeStationBalance(drive),
      new InstantCommand(()->scoringLocationUtil.toggleMiddleGrid())); //i assume i should toggle this back as we want it to by default be at false
  }

  public PathPlannerTrajectory getTrajectory() {
    return this.traj;
  }
}
