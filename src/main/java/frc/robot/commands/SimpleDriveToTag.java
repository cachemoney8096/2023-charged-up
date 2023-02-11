package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TagLimelight;
import frc.robot.subsystems.drive.DriveSubsystem;

public class SimpleDriveToTag extends CommandBase {
    private TagLimelight limelight;
    private DriveSubsystem drive;

    public SimpleDriveToTag(TagLimelight limelight, DriveSubsystem drive){
        addRequirements(limelight, drive);
        this.limelight = limelight;
        this.drive = drive;
    }

    @Override
    public void execute(){
        // Rotation2d robotRotation = drive.getPose().getRotation();
        // Translation2d robotPos = drive.getPose().getTranslation();
        if (limelight.isValidTarget()){
            Translation2d tagRelativeToRobot = limelight.getTargetTranslation();
            double correctedXVal = -(tagRelativeToRobot.getX());
            double correctedYVal = -(tagRelativeToRobot.getY());
            Translation2d correctedTagRelativeToRobot = new Translation2d(correctedXVal, correctedYVal);
            Translation2d zeroedTranslation = new Translation2d(0, 0);
            Rotation2d zeroedRotation = new Rotation2d(0);
            PathPoint robotPoint = new PathPoint(zeroedTranslation, zeroedRotation);
            PathPoint tagPoint = new PathPoint(correctedTagRelativeToRobot, f)
            // PathPlannerTrajectory robotToTag = PathPlanner.generatePath(
                
            );
        }
    }
}
