package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class SwerveToPointWrapper extends CommandBase {

  public Command swerveFollowerCmd;
  private DriveSubsystem drive;
  private Pose2d finalPose;
  private double finalSpeedMetersPerSec;
  boolean redAlliance;
  double timeoutSec;

  public SwerveToPointWrapper(
      boolean red,
      DriveSubsystem driveSubsystem,
      Pose2d desiredPose,
      double timeoutSeconds,
      double endSpeedMps) {
    if (!red) {
      finalPose = desiredPose;
    } else {
      finalPose =
          new Pose2d(
              desiredPose.getX(),
              Constants.FIELD_WIDTH_METERS - desiredPose.getY(),
              desiredPose.getRotation());
    }
    timeoutSec = timeoutSeconds;
    finalSpeedMetersPerSec = endSpeedMps;
    redAlliance = red;
    drive = driveSubsystem;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
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
