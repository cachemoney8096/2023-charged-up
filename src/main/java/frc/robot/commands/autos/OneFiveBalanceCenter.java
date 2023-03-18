package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeSequence;
import frc.robot.commands.autos.components.DriveDistance;
import frc.robot.commands.autos.components.DriveFullyOver;
import frc.robot.commands.autos.components.DriveUntilBalanced;
import frc.robot.commands.autos.components.ScoreThisGamePiece;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeLimelight;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.ScoringLocationUtil;

/** Scores a game piece, drives over the charge station, then back on and balances */
public class OneFiveBalanceCenter extends SequentialCommandGroup {
  private static final double DISTANCE_TO_CONE_METERS = 1.5;
  private static final double NORM_SPEED_INTAKING = 0.3;

  public OneFiveBalanceCenter(
      Lift lift,
      DriveSubsystem drive,
      Lights lights,
      ScoringLocationUtil scoringLocationUtil,
      IntakeLimelight limelight,
      Intake intake) {
    addCommands(
        new ScoreThisGamePiece(false, lift, lights),
        new DriveFullyOver(drive, true),
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
                new DriveDistance(drive, NORM_SPEED_INTAKING, DISTANCE_TO_CONE_METERS, 0, false)),
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
        new DriveDistance(drive, -NORM_SPEED_INTAKING, -DISTANCE_TO_CONE_METERS, 0, false),

        // Turn towards charge station and drive on
        new InstantCommand(drive::setZeroTargetHeading),
        new RunCommand(
                () -> {
                  drive.rotateOrKeepHeading(0, 0, 0, true, -1);
                })
            .withTimeout(0.25),
        new DriveUntilBalanced(drive, false));
  }
}
