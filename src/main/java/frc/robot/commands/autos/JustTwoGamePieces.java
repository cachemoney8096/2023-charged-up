package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autos.components.GetAndScoreOpenSide;
import frc.robot.commands.autos.components.ScoreThisGamePiece;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.TagLimelightV2;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.ScoringLocationUtil;
import frc.robot.utils.ScoringLocationUtil.ScoreCol;
import frc.robot.utils.ScoringLocationUtil.ScoreHeight;

/**
 * This auto assumes a start at the cone scoring position nearest the loading zone. It scores a
 * cone, grabs another, and scores that one too.
 */
public class JustTwoGamePieces extends SequentialCommandGroup {

  public JustTwoGamePieces(
      boolean red,
      Lift lift,
      Intake intake,
      DriveSubsystem drive,
      Lights lights,
      TagLimelightV2 tagLimelight,
      ScoringLocationUtil scoringLocationUtil) {
    addRequirements(lift, intake, drive, tagLimelight);
    /** Initialize sequential commands that run for the "15 second autonomous phase" */
    addCommands(
        new InstantCommand(
            () -> scoringLocationUtil.setScoreCol(red ? ScoreCol.LEFT : ScoreCol.RIGHT)),
        new InstantCommand(() -> scoringLocationUtil.setScoreHeight(ScoreHeight.HIGH)),
        new ScoreThisGamePiece(false, lift, lights),
        new GetAndScoreOpenSide(
            red, false, lift, intake, drive, lights, tagLimelight, scoringLocationUtil),
        new InstantCommand(
            () -> {
              drive.resetYaw();
            }));
  }
}
