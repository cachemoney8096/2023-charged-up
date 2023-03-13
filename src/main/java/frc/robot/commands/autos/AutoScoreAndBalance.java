package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autos.components.AutoChargeStationSequence;
import frc.robot.commands.autos.components.ScoreThisGamePiece;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.ScoringLocationUtil;

/** Scores a game piece, then drives onto the charge station and balances */
public class AutoScoreAndBalance extends SequentialCommandGroup {
  private static final double DISTANCE_UP_CHARGE_STATION_METERS = 2.21;

  public AutoScoreAndBalance(
      Lift lift, DriveSubsystem drive, Lights lights, ScoringLocationUtil scoringLocationUtil) {
    addCommands(
        new ScoreThisGamePiece(false, lift, lights),
        new AutoChargeStationSequence(drive, DISTANCE_UP_CHARGE_STATION_METERS));
  }
}
