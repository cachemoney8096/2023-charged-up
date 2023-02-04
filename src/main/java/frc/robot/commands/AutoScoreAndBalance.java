package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Cal;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.drive.DriveSubsystem;

/** Scores a game piece, then drives onto the charge station and balances */
public class AutoScoreAndBalance extends SequentialCommandGroup {
  private PathPlannerTrajectory traj =
      PathPlanner.loadPath(
          "ScoreAndBalanceTraj.path",
          new PathConstraints(Cal.PLACEHOLDER_DOUBLE, Cal.PLACEHOLDER_DOUBLE));

  public AutoScoreAndBalance(boolean isFirstPath, Lift lift, DriveSubsystem drive) {
    addCommands(
        new RunCommand(lift::scoreGamePiece, lift).until(lift::doneScoring),
        drive.followTrajectoryCommand(traj, isFirstPath),
        new AutoChargeStationBalance(drive));
  }

  public PathPlannerTrajectory getTrajectory() {
    return this.traj;
  }
}
