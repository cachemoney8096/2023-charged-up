package frc.robot.commands.autos.components;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DriveUntilBalanced extends SequentialCommandGroup {

  private static final double NORM_SPEED_UP_CHARGE_STATION = 0.5;
  private static final double FILTERED_PITCH_THRESHOLD = 10.0;

  public DriveUntilBalanced(DriveSubsystem drive, boolean forwards) {
    double normDriveSpeed = forwards ? NORM_SPEED_UP_CHARGE_STATION : -NORM_SPEED_UP_CHARGE_STATION;

    addCommands(
        // drive until pitched
        new RunCommand(
                () -> {
                  drive.drive(normDriveSpeed, 0, 0, true);
                },
                drive)
            .until(() -> Math.abs(drive.getFilteredPitch()) > FILTERED_PITCH_THRESHOLD),
        new AutoChargeStationBalance(drive));
  }
}
