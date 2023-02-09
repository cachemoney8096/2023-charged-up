package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.LiftPosition;
import frc.robot.utils.ScoringLocationUtil;
import frc.robot.utils.ScoringLocationUtil.ScoreCol;
import frc.robot.utils.ScoringLocationUtil.ScoreHeight;

/** takes the column and height from ScoringLocationUtil.java
 * and converts that to a LiftPosition
 * then gives the position to the given lift */
public class ManualPrepScoreSequence extends SequentialCommandGroup {

  public ManualPrepScoreSequence(Lift lift, ScoringLocationUtil scoreLoc) {
    ScoreHeight height = scoreLoc.getScoreHeight();
    ScoreCol col = scoreLoc.getScoreCol();

    // left and right columns are for cones
    // but bottom can technically be either cone or cube
    if (col == ScoreCol.LEFT || col == ScoreCol.RIGHT) {
      if (height == ScoreHeight.LOW) {
        lift.setDesiredPosition(LiftPosition.SCORE_LOW_CONE);
      } else if (height == ScoreHeight.MID) {
        lift.setDesiredPosition(LiftPosition.SCORE_MID_CONE);
      } else {
        lift.setDesiredPosition(LiftPosition.SCORE_HIGH_CONE);
      }
    }
    // middle columns are for cubes
    // but bottom can technically be either cone or cube
    else {
      if (height == ScoreHeight.LOW) {
        lift.setDesiredPosition(LiftPosition.SCORE_LOW_CUBE);
      } else if (height == ScoreHeight.MID) {
        lift.setDesiredPosition(LiftPosition.SCORE_MID_CUBE);
      } else {
        lift.setDesiredPosition(LiftPosition.SCORE_HIGH_CUBE);
      }
    }
  }
}
