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
          "ScoreAndBalanceTraj",
          new PathConstraints(
              Cal.SwerveSubsystem.MAX_LINEAR_SPEED_METERS_PER_SEC,
              Cal.SwerveSubsystem.MAX_LINEAR_ACCELERATION_METERS_PER_SEC_SQ));

  public AutoScoreAndBalance(boolean isFirstPath, Lift lift, DriveSubsystem drive) {
    // TODO update this since scoreGamePiece and doneScoring are stubbed
    addCommands(
        new RunCommand(lift::scoreGamePiece, lift).until(lift::doneScoring),
        drive.followTrajectoryCommand(traj, isFirstPath),
        new AutoChargeStationBalance(drive));
  }

  public PathPlannerTrajectory getTrajectory() {
    return this.traj;
  }
}
