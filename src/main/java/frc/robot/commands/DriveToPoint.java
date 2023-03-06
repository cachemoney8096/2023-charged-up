package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Cal;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DriveToPoint extends CommandBase {

  DriveSubsystem drive;
  Pose2d finalPose = new Pose2d();
  Pose2d firstPose = new Pose2d();
  double totalDistanceMeters = 1.0;

  /** In: meters. Out: m/s. */
  ProfiledPIDController controller = new ProfiledPIDController(
    2.0,
    0.0,
    0.0,
    new TrapezoidProfile.Constraints(
              1.0,
              1.0));

  public DriveToPoint(DriveSubsystem driveSubsystem) {
    drive = driveSubsystem;
    controller.setTolerance(0.05);
    addRequirements(drive);
  }
  
  @Override
  public void initialize() {
    finalPose = drive.targetPose;
    firstPose = drive.getPose();
    totalDistanceMeters = firstPose.minus(finalPose).getTranslation().getNorm();
    controller.setGoal(totalDistanceMeters);
  }

  @Override
  public void execute() {
    Pose2d curPose = drive.getPose();
    double distanceTraveledMeters = curPose.minus(firstPose).getTranslation().getNorm();
    Translation2d direction = finalPose.minus(firstPose).getTranslation();
    direction = direction.div(direction.getNorm());
    double translationDemand = controller.calculate(distanceTraveledMeters);
    double feedforwardDemand = controller.getSetpoint().velocity / Cal.SwerveSubsystem.MAX_LINEAR_SPEED_METERS_PER_SEC;

    drive.targetHeadingDegrees = 0.0; // let heading lock figure this out
    direction = direction.times(translationDemand + feedforwardDemand);

    drive.drive(direction.getX(), direction.getY(), 0, true);
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
      return controller.atGoal();
  }
}
