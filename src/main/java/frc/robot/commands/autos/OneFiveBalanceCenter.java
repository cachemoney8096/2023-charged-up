package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.IntakeSequence;
import frc.robot.commands.autos.components.AutoChargeStationBalance;
import frc.robot.commands.autos.components.ScoreThisGamePiece;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeLimelight;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.ScoringLocationUtil;

/** Scores a game piece, drives over the charge station, then back on and balances */
public class OneFiveBalanceCenter extends SequentialCommandGroup {
  private static final double DISTANCE_PAST_CHARGE_METERS = 4.75;
  private static final double DISTANCE_AT_CONE_METERS = 5.9;
  private static final double DISTANCE_BACK_CHARGE_STATION_METERS = 3.15;
  private static final double NORM_SPEED_UP_CHARGE_STATION = 0.5;
  private static final double NORM_SPEED_BACK_CHARGE_STATION = -0.4;
  private static final double NORM_SPEED_INTAKING = 0.3;
  private double startXMeters = 0;

  public OneFiveBalanceCenter(
      Lift lift,
      DriveSubsystem drive,
      Lights lights,
      ScoringLocationUtil scoringLocationUtil,
      IntakeLimelight limelight,
      Intake intake) {
    addCommands(
        new ScoreThisGamePiece(false, lift, lights),
        new InstantCommand(
            () -> {
              startXMeters = drive.getPose().getX();
            }),

        // Drive just over charge station
        new InstantCommand(
            () -> {
              drive.drive(NORM_SPEED_UP_CHARGE_STATION, 0, 0, true);
            },
            drive),
        new WaitUntilCommand(
            () -> {
              return (drive.getPose().getX() - startXMeters) > DISTANCE_PAST_CHARGE_METERS;
            }),
        drive.stopDrivingCommand(),
        new WaitCommand(0.25),

        // Turn to cone, intake it
        new InstantCommand(
            () -> {
              drive.offsetCurrentHeading(limelight.getAngleToConeDeg());
            }),
        // new ScheduleCommand(new IntakeSequence(intake, lift, lights).withTimeout(2.0)),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                new RunCommand(
                        () -> {
                          drive.rotateOrKeepHeading(NORM_SPEED_INTAKING, 0, 0, false, -1);
                        })
                    .until(
                        () -> (drive.getPose().getX() - startXMeters) > DISTANCE_AT_CONE_METERS)),
            new IntakeSequence(intake, lift, lights)
                .finallyDo(
                    (boolean interrupted) -> {
                      lift.home();
                      lift.closeGrabber();
                      intake.setDesiredDeployed(false);
                      intake.setDesiredClamped(false);
                      intake.stopIntakingGamePiece();
                    })),

        // Drive back
        new RunCommand(
                () -> {
                  drive.rotateOrKeepHeading(-NORM_SPEED_INTAKING, 0, 0, false, -1);
                })
            .until(() -> (drive.getPose().getX() - startXMeters) < DISTANCE_PAST_CHARGE_METERS),

        // Turn towards charge station and drive on
        new InstantCommand(drive::setZeroTargetHeading),
        new RunCommand(
                () -> {
                  drive.rotateOrKeepHeading(0, 0, 0, true, -1);
                })
            .withTimeout(0.25),
        new InstantCommand(
            () -> {
              drive.drive(NORM_SPEED_BACK_CHARGE_STATION, 0, 0, true);
            }),
        new WaitUntilCommand(
            () -> {
              return (drive.getPose().getX() - startXMeters) < DISTANCE_BACK_CHARGE_STATION_METERS;
            }),

        // And balance!
        new AutoChargeStationBalance(drive));
  }
}
