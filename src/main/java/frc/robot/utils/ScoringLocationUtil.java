package frc.robot.utils;

public class ScoringLocationUtil {
  /** Height to score at */
  public enum ScoreHeight {
    LOW,
    MID,
    HIGH
  }

  /** Column to score in */
  public enum ScoreCol {
    LEFT,
    CENTER,
    RIGHT
  }

  private ScoreHeight scoreHeight = ScoreHeight.HIGH; // default value
  private ScoreCol scoreCol = ScoreCol.LEFT; // default value
  private boolean middleGrid = false; // default value

  /** Flip the value of middleGrid */
  public void toggleMiddleGrid() {
    middleGrid = !middleGrid;
  }

  // TODO map scoreCol and scoreHeight to tag IDs.
  public void setScoreCol(ScoreCol chooseCol) {
    scoreCol = chooseCol;
  }

  public ScoreCol getScoreCol() {
    return scoreCol;
  }

  public void setScoreHeight(ScoreHeight chooseHeight) {
    scoreHeight = chooseHeight;
  }

  public ScoreHeight getScoreHeight() {
    return scoreHeight;
  }

  public boolean isCone(){
    if(scoreHeight == ScoreHeight.LOW || scoreCol == ScoreCol.CENTER){
      return false;
    } else{
      return true;
    }
  }
}
