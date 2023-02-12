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
    private PathPlannerTrajectory trajInit =
        PathPlanner.loadPath(
    "InitScoreAndGetGamePiece",
        new PathConstraints(Cal.PLACEHOLDER_DOUBLE, Cal.PLACEHOLDER_DOUBLE));

    private PathPlannerTrajectory trajCharge =
        PathPlanner.loadPath(
    "ScoringLocToChargeStation",
        new PathConstraints(Cal.PLACEHOLDER_DOUBLE, Cal.PLACEHOLDER_DOUBLE));

    private HashMap<String, Command> eventMap = new HashMap<>();
    private boolean isFirstPath = false;
    
    /** Events include: open intake and close intake before and after obtaining game piece */
    public AutoScore(Lift lift, Intake intake, DriveSubsystem drive, TagLimelight tagLimelight){
        addRequirements(lift, intake, drive, tagLimelight)
         /** Events include: open intake and close intake before and after obtaining game piece, */
        eventMap.put("deployIntake", new InstantCommand(()->{intake.setDesiredDeployed(true);}, intake));
        eventMap.put("closeIntake", new InstantCommand(()->{intake.setDesiredDeployed(false);}, intake));
        /**TODO: configure limelight to "take over" driving process after certain point 
        AFTER obtaining game piece to be more accurate with distance */


        /** Initialize sequential commands that run for the "15 second autonomous phase" */
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

            /** TODO: Limelight code goes here */

            new InstantCommand(lift::manualPrepScore, lift),
            new WaitUntilCommand(()->lift.atPosition(LiftPosition.MANUAL_PREP_SCORE)),
            drive.followTrajectoryCommand(trajCharge, isFirstPath), //this does not accept the FollowPathWithEvents
            new AutoChargeStationBalance(drive)
        );
    }
    
}
