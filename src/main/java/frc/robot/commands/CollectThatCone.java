package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autos.components.DriveDistance;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeLimelight;
import frc.robot.subsystems.IntakeLimelight.ConeDetection;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.Optional;

public class CollectThatCone extends SequentialCommandGroup {

  private static final double DISTANCE_IN_FRONT_OF_CONE_METERS = Units.inchesToMeters(24);
  private Optional<ConeDetection> coneDetection = Optional.empty();
  private Pose2d inFrontOfCone = new Pose2d();
  private static final double SPEED_AT_CONE_MPS = 2.0;
  private static final double NORM_SPEED_INTAKING = 0.5;

  /** How long to wait before beginning to drive */
  private static double waitTime(double distanceToConeMeters) {
    return distanceToConeMeters > 1.5 ? 0.03 : 0.5;
  }

  private static Pose2d getPoseInFrontOfCone(DriveSubsystem drive, ConeDetection cone) {
    Rotation2d coneAngle = Rotation2d.fromDegrees(cone.angleDeg);
    double coneDistanceMeters = cone.distanceMeters;
    Pose2d poseAtDetection = drive.getPastPose(cone.latencySec);
    Transform2d toInFrontOfCone =
        new Transform2d(
            new Translation2d(coneDistanceMeters - DISTANCE_IN_FRONT_OF_CONE_METERS, 0)
                .rotateBy(coneAngle),
            coneAngle);
    return poseAtDetection.plus(toInFrontOfCone);
  }

  public CollectThatCone(
      DriveSubsystem drive, IntakeLimelight limelight, Intake intake, Lift lift, Lights lights) {
    addCommands(
        new RunCommand(
                () -> {
                  coneDetection = limelight.getConePos();
                  if (!coneDetection.isPresent()) {
                    return;
                  }
                  inFrontOfCone = getPoseInFrontOfCone(drive, coneDetection.get());
                })
            .until(() -> coneDetection.isPresent()),
        new ParallelRaceGroup(
            new SequentialCommandGroup(
                    new ProxyCommand(
                        () -> new WaitCommand(waitTime(coneDetection.get().distanceMeters))),
                    new SwerveToPointWrapper(false, drive, inFrontOfCone, 2.0, SPEED_AT_CONE_MPS),
                    new DriveDistance(
                        drive, NORM_SPEED_INTAKING, DISTANCE_IN_FRONT_OF_CONE_METERS, 0.0, false))
                .asProxy()
                .finallyDo(
                    (boolean interrupted) -> {
                      drive.stopDriving();
                    }),
            new IntakeSequence(intake, lift, lights)
                .finallyDo(
                    (boolean interrupted) -> {
                      lift.home();
                      lift.closeGrabber();
                      intake.setDesiredDeployed(false);
                      intake.setDesiredClamped(false);
                      intake.stopIntakingGamePiece();
                    })));
  }
}
