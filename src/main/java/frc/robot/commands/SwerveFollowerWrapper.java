package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class SwerveFollowerWrapper extends CommandBase {
  
  public Command swerveFollowerCmd;
  private DriveSubsystem drive;

  public SwerveFollowerWrapper(DriveSubsystem driveSubsystem) {
    drive = driveSubsystem;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    swerveFollowerCmd = drive.followTrajectoryCommand(drive.pathToScoreBasedOnTag, false, Optional.of(3.0));
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
