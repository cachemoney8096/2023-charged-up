package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Cal;
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
            Translation2d zeroTranslation = new Translation2d(0, 0);
            Rotation2d zeroRotation = new Rotation2d(0);
            Pose2d currentPose = drive.getPose();
        
            Translation2d tagPosFromRobot = limelight.getTargetTranslation();
            Rotation2d tagAngleRelativeToRobot = new Rotation2d(tagPosFromRobot.getY() / tagPosFromRobot.getX());
            Pose2d tagPoseFromRobot = new Pose2d(tagPosFromRobot, tagAngleRelativeToRobot);

            // double correctedXVal = -(tagPos.getX());
            // double correctedYVal = -(tagPos.getY());
            // Translation2d correctedTagRelativeToRobot = new Translation2d(correctedXVal, correctedYVal);
            // PathPoint robotPoint = new PathPoint(zeroTranslation, zeroRotation);
            // PathPoint tagPoint = new PathPoint(correctedTagRelativeToRobot, tagAngleRelativeToRobot);
            
            // TODO get actual limelight position
            // TODO get translation of intended robot loc vs tag loc
            PathPlannerTrajectory robotToTag = PathPlanner.generatePath(
                new PathConstraints(Cal.SwerveSubsystem.MAX_LINEAR_SPEED_METERS_PER_SEC, Cal.SwerveSubsystem.MAX_LINEAR_ACCELERATION_METERS_PER_SEC_SQ),
                new PathPoint(currentPose.getTranslation(), currentPose.getRotation()),
                new PathPoint(tagPoseFromRobot.getTranslation(), tagAngleRelativeToRobot)
            );
            drive.followTrajectoryCommand(robotToTag, false);
        }
    }
}
