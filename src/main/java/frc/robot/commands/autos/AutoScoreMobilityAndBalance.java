package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autos.components.AutoMobilityChargeStationSequence;
import frc.robot.commands.autos.components.ScoreThisGamePiece;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.ScoringLocationUtil;

/** Scores a game piece, drives over the charge station, then back on and balances */
public class AutoScoreMobilityAndBalance extends SequentialCommandGroup {
  private static final double DISTANCE_UP_CHARGE_STATION_METERS = 4.5;
  private static final double DISTANCE_BACK_CHARGE_STATION_METERS = 3.15;

  public AutoScoreMobilityAndBalance(
      Lift lift, DriveSubsystem drive, Lights lights, ScoringLocationUtil scoringLocationUtil) {
    addCommands(
        new ScoreThisGamePiece(false, lift, lights),
        new AutoMobilityChargeStationSequence(
            drive, DISTANCE_UP_CHARGE_STATION_METERS, DISTANCE_BACK_CHARGE_STATION_METERS));
  }
}
