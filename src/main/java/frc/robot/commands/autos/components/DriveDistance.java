package frc.robot.commands.autos.components;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveSubsystem;

/** Drives in a direction at a speed for a certain distance */
public class DriveDistance extends SequentialCommandGroup {
  double distanceTraveledMeters = 0;
  Pose2d startPose = new Pose2d();
  Translation2d direction = new Translation2d();
  Translation2d speeds = new Translation2d();
  double distanceToTravel = 1.0;

  /**
   * @param normSpeed should be in [0,1]
   */
  public DriveDistance(
      DriveSubsystem drive,
      double normSpeed,
      double XMeters,
      double YMeters,
      boolean fieldRelative) {
    Translation2d desiredTranslation = new Translation2d(XMeters, YMeters);
    distanceToTravel = desiredTranslation.getNorm();
    direction = desiredTranslation.div(distanceToTravel);
    speeds = direction.times(normSpeed);

    addCommands(
        new InstantCommand(
            () -> {
              startPose = drive.getPose();
            }),
        new RunCommand(
                () -> {
                  drive.drive(speeds.getX(), speeds.getY(), 0.0, fieldRelative);
                }, drive)
            .until(
                () -> {
                  Pose2d curPose = drive.getPose();
                  Translation2d movement = curPose.minus(startPose).getTranslation();
                  // This does not ensure that the movement is in the intended direction
                  // but it does ensure that we don't drive durther than we intended
                  return movement.getNorm() > distanceToTravel;
                }),
        drive.stopDrivingCommand());
  }
}
