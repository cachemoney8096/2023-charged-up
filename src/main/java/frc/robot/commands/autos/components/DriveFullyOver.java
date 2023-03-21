package frc.robot.commands.autos.components;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DriveFullyOver extends SequentialCommandGroup {

  private static final double NORM_SPEED_OVER_CHARGE_STATION = 0.5;
  private static final double FILTERED_PITCH_THRESHOLD_DEG = 15.0;
  private static final double CLOSE_TO_ZERO_PITCH_THRESHOLD_DEG = 6.0;
  boolean pitchHasBeenBig = false;

  public DriveFullyOver(DriveSubsystem drive, boolean forwards) {
    double normDriveSpeed =
        forwards ? NORM_SPEED_OVER_CHARGE_STATION : -NORM_SPEED_OVER_CHARGE_STATION;

    addCommands(
        // Drive until we've pitched a good amount and then we pass zero
        new RunCommand(
                () -> {
                  drive.drive(normDriveSpeed, 0, 0, true);
                },
                drive)
            .until(
                () -> {
                  double currentPitchDegrees = drive.getFilteredPitch();
                  if (Math.abs(currentPitchDegrees) > FILTERED_PITCH_THRESHOLD_DEG) {
                    pitchHasBeenBig = true;
                  }
                  boolean currentPitchNearZero =
                      Math.abs(currentPitchDegrees) < CLOSE_TO_ZERO_PITCH_THRESHOLD_DEG;

                  return pitchHasBeenBig && currentPitchNearZero;
                }),
        // Then drive a bit more
        new RunCommand(
                () -> {
                  drive.drive(normDriveSpeed, 0, 0, true);
                },
                drive)
            .withTimeout(0.3),
        drive.stopDrivingCommand());
  }
}
