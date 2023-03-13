package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.LiftPosition;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.ScoringLocationUtil;

/** Scores a game piece, then drives onto the charge station and balances */
public class AutoScoreMobilityAndBalance extends SequentialCommandGroup {
  private static final double DISTANCE_UP_CHARGE_STATION_METERS = 4.5;
  private static final double DISTANCE_BACK_CHARGE_STATION_METERS = 3.15;

  public AutoScoreMobilityAndBalance(
      Lift lift, DriveSubsystem drive, Lights lights, ScoringLocationUtil scoringLocationUtil) {
    addCommands(
        new InstantCommand(() -> lift.ManualPrepScoreSequence(lights), lift),
        new WaitUntilCommand(() -> lift.atPosition(LiftPosition.PRE_SCORE_HIGH_CONE))
            .withTimeout(0.75),
        new InstantCommand(lift::startScore, lift),
        new WaitUntilCommand(() -> lift.atPosition(LiftPosition.SCORE_HIGH_CONE)).withTimeout(0.25),
        new finishScore(lift, lights),
        new WaitUntilCommand(() -> lift.atPosition(LiftPosition.STARTING)).withTimeout(0.5),
        new AutoMobilityChargeStationSequence(
            drive, DISTANCE_UP_CHARGE_STATION_METERS, DISTANCE_BACK_CHARGE_STATION_METERS));
  }
}
