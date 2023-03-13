package frc.robot.commands.autos.components;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Cal;
import frc.robot.commands.finishScore;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.LiftPosition;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.LightCode;

/**
 * Assuming the robot is already in position to score and the scoring location util has been
 * updated, this command will score the game piece and send the lift back to starting position
 * (including a wait at the end)
 *
 * @param fast If true, this will use the fast timeouts on the lift movements
 */
public class ScoreThisGamePiece extends SequentialCommandGroup {
  public ScoreThisGamePiece(boolean fast, Lift lift, Lights lights) {
    addCommands(
        new InstantCommand(() -> lift.ManualPrepScoreSequence(lights), lift),
        new WaitUntilCommand(() -> lift.atPosition(LiftPosition.PRE_SCORE_HIGH_CONE))
            .withTimeout(Cal.Lift.START_TO_PRESCORE_HIGH_SEC),
        new InstantCommand(
            () -> {
              lights.setLight(LightCode.READY_TO_SCORE);
            }),
        new InstantCommand(lift::startScore, lift),
        new WaitUntilCommand(() -> lift.atPosition(LiftPosition.SCORE_HIGH_CONE))
            .withTimeout(Cal.Lift.PRESCORE_TO_SCORE_SEC),
        new finishScore(lift, lights),
        new WaitUntilCommand(() -> lift.atPosition(LiftPosition.STARTING))
            .withTimeout(fast ? Cal.Lift.SCORE_TO_START_FAST_SEC : Cal.Lift.SCORE_TO_START_SEC));
  }
}
