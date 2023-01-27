package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoScoreAndBalance extends SequentialCommandGroup {
    private PathPlannerTrajectory traj = PathPlanner.loadPath("Trajectory",
            new PathConstraints(Calibrations.PLACEHOLDER_DOUBLE, Calibrations.PLACEHOLDER_DOUBLE));
    public AutoScoreAndBalance(boolean isFirstPath, Lift lift, DriveSubsystem drive) {
        addCommands(
                new InstantCommand(lift::scoreGamePiece, lift),
                drive.followTrajectoryCommand(traj, isFirstPath),
                new AutoChargeStationBalance(drive));
    }

    public PathPlannerTrajectory getTrajectory() {
        return this.traj;
    }
}
