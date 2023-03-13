package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class SwerveFollowerWrapper extends CommandBase {

  public Command swerveFollowerCmd;
  private DriveSubsystem drive;
  boolean redAlliance;

  public SwerveFollowerWrapper(boolean red, DriveSubsystem driveSubsystem) {
    redAlliance = red;
    drive = driveSubsystem;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    swerveFollowerCmd =
        drive.followTrajectoryCommand(drive.poseToPath(redAlliance), false).withTimeout(3.0);
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
