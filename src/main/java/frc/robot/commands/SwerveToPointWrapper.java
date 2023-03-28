package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.function.Supplier;

public class SwerveToPointWrapper extends CommandBase {

  public Command swerveFollowerCmd;
  private DriveSubsystem drive;
  private Supplier<Pose2d> desiredPoseSupplier;
  private double finalSpeedMetersPerSec;
  boolean flipForRed;
  double timeoutSec;

  public SwerveToPointWrapper(
      boolean red,
      DriveSubsystem driveSubsystem,
      Supplier<Pose2d> finalPoseSupplier,
      double timeoutSeconds,
      double endSpeedMps) {
    flipForRed = red;
    desiredPoseSupplier = finalPoseSupplier;
    timeoutSec = timeoutSeconds;
    finalSpeedMetersPerSec = endSpeedMps;
    drive = driveSubsystem;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    Pose2d finalPose = desiredPoseSupplier.get();
    if (flipForRed) {
      finalPose =
          new Pose2d(
              finalPose.getX(),
              Constants.FIELD_WIDTH_METERS - finalPose.getY(),
              finalPose.getRotation());
    }

    PathPlannerTrajectory trajectory = drive.pathToPoint(finalPose, finalSpeedMetersPerSec);
    swerveFollowerCmd = drive.followTrajectoryCommand(trajectory, false).withTimeout(timeoutSec);
    swerveFollowerCmd.initialize();
  }

  @Override
  public void execute() {
    swerveFollowerCmd.execute();
  }

  @Override
  public void end(boolean interrupted) {
    swerveFollowerCmd.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return swerveFollowerCmd.isFinished();
  }
}
