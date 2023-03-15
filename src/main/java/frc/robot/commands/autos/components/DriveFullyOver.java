package frc.robot.commands.autos.components;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DriveFullyOver extends SequentialCommandGroup {

  private static final double NORM_SPEED_OVER_CHARGE_STATION = 0.5;
  private static final double FILTERED_PITCH_THRESHOLD_DEG = 10.0;
  private static final double CLOSE_TO_ZERO_PITCH_THRESHOLD_DEG = 6.0;
  boolean pitchHasBeenNegative = false;
  boolean pitchHasBeenPositive = false;

  public DriveFullyOver(DriveSubsystem drive, boolean forwards) {
    double normDriveSpeed =
        forwards ? NORM_SPEED_OVER_CHARGE_STATION : -NORM_SPEED_OVER_CHARGE_STATION;

    addCommands(
        new RunCommand(
                () -> {
                  drive.drive(normDriveSpeed, 0, 0, true);
                },
                drive)
            .until(
                () -> {
                  double currentPitchDegrees = drive.getFilteredPitch();
                  if (currentPitchDegrees > FILTERED_PITCH_THRESHOLD_DEG) {
                    pitchHasBeenPositive = true;
                  }
                  if (currentPitchDegrees < -FILTERED_PITCH_THRESHOLD_DEG) {
                    pitchHasBeenNegative = true;
                  }
                  boolean currentPitchNearZero =
                      Math.abs(currentPitchDegrees) < CLOSE_TO_ZERO_PITCH_THRESHOLD_DEG;
                  return pitchHasBeenPositive && pitchHasBeenNegative && currentPitchNearZero;
                }),
        drive.stopDrivingCommand());
  }
}
