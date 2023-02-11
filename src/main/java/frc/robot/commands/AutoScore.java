package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.TagLimelight;
import frc.robot.subsystems.Lift.LiftPosition;
import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import frc.robot.Cal;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoScore extends SequentialCommandGroup{
    private DriveSubsystem drive;
    private TagLimelight tagLimelight;
    private PathPlannerTrajectory trajInit =
        PathPlanner.loadPath(
    "InitScoreAndGetGamePiece",
        new PathConstraints(Cal.PLACEHOLDER_DOUBLE, Cal.PLACEHOLDER_DOUBLE));
    private PathPlannerTrajectory trajCharge =
        PathPlanner.loadPath(
    "ScoringLocToChargeStation",
        new PathConstraints(Cal.PLACEHOLDER_DOUBLE, Cal.PLACEHOLDER_DOUBLE));
    private HashMap<String, Command> eventMap = new HashMap<>();
    private boolean isFirstPath = true;
    private FollowPathWithEvents pathWithEvents;
    
    public AutoScore(Lift lift, Intake intake){
        eventMap.put("deployIntake", new InstantCommand(()->{intake.setDesiredDeployed(true);}, intake));
        eventMap.put("closeIntake", new InstantCommand(()->{intake.setDesiredDeployed(false);}, intake));
        addCommands(
            new InstantCommand(lift::manualPrepScore, lift),
            new WaitUntilCommand(()->lift.atPosition(LiftPosition.MANUAL_PREP_SCORE)),
            new startScore(),
            new WaitUntilCommand(()->lift.atPosition(LiftPosition.SCORE_HIGH_CONE)),
            new finishScore(),
            new WaitUntilCommand(()->lift.atPosition(LiftPosition.STARTING)),
            new FollowPathWithEvents(
                drive.followTrajectoryCommand(trajInit, true),
                trajInit.getMarkers(), eventMap),
            /*limelight goes here */
            new InstantCommand(lift::manualPrepScore, lift),
            new WaitUntilCommand(()->lift.atPosition(LiftPosition.MANUAL_PREP_SCORE)),
            drive.followTrajectoryCommand(trajCharge, isFirstPath), //this does not accept the FollowPathWithEvents
            new AutoChargeStationBalance(drive)
        );
    }
    
}
