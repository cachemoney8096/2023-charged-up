package frc.robot.commands.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Cal;
import frc.robot.commands.autos.components.AutoChargeStationSequence;
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
 * cone, grabs another, and scores that one too. Finally, it gets on the charge station and tries to
 * balance.
 */
public class TwoGamePiecesThatEngage extends SequentialCommandGroup {
  private PathPlannerTrajectory trajCharge =
      PathPlanner.loadPath(
          "ScoringLocToChargeStation",
          new PathConstraints(
              Cal.SwerveSubsystem.MAX_LINEAR_SPEED_METERS_PER_SEC,
              Cal.SwerveSubsystem.MAX_LINEAR_ACCELERATION_METERS_PER_SEC_SQ));

  private static final double DISTANCE_UP_CHARGE_STATION_METERS = 1.6;

  public TwoGamePiecesThatEngage(
      boolean red,
      Lift lift,
      Intake intake,
      DriveSubsystem drive,
      Lights lights,
      TagLimelightV2 tagLimelight,
      ScoringLocationUtil scoringLocationUtil) {
    if (red) {
      trajCharge =
          PathPlanner.loadPath(
              "ScoringLocToChargeStationRed",
              new PathConstraints(
                  Cal.SwerveSubsystem.MAX_LINEAR_SPEED_METERS_PER_SEC,
                  Cal.SwerveSubsystem.MAX_LINEAR_ACCELERATION_METERS_PER_SEC_SQ));

      trajCharge =
          PathPlannerTrajectory.transformTrajectoryForAlliance(
              trajCharge, DriverStation.Alliance.Red);
    }

    addRequirements(lift, intake, drive, tagLimelight);

    /** Initialize sequential commands that run for the "15 second autonomous phase" */
    addCommands(
        new InstantCommand(
            () -> scoringLocationUtil.setScoreCol(red ? ScoreCol.LEFT : ScoreCol.RIGHT)),
        new InstantCommand(() -> scoringLocationUtil.setScoreHeight(ScoreHeight.HIGH)),
        new ScoreThisGamePiece(true, lift, lights),
        new GetAndScoreOpenSide(
            red, true, lift, intake, drive, lights, tagLimelight, scoringLocationUtil),
        drive.followTrajectoryCommand(trajCharge, false).withTimeout(2.0),
        new InstantCommand(
            () -> {
              drive.resetYaw();
            }),
        new AutoChargeStationSequence(drive, DISTANCE_UP_CHARGE_STATION_METERS));
  }
}
