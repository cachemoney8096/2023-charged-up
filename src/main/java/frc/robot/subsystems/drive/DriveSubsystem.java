// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Cal;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.utils.GeometryUtils;

public class DriveSubsystem extends SubsystemBase {
  private double targetHeadingDegrees;

  // Create SwerveModules
  private final SwerveModule frontLeft =
      new SwerveModule(
          RobotMap.FRONT_LEFT_DRIVING_CAN_ID,
          RobotMap.FRONT_LEFT_TURNING_CAN_ID,
          Cal.SwerveSubsystem.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET_RAD);

  private final SwerveModule frontRight =
      new SwerveModule(
          RobotMap.FRONT_RIGHT_DRIVING_CAN_ID,
          RobotMap.FRONT_RIGHT_TURNING_CAN_ID,
          Cal.SwerveSubsystem.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD);

  private final SwerveModule rearLeft =
      new SwerveModule(
          RobotMap.REAR_LEFT_DRIVING_CAN_ID,
          RobotMap.REAR_LEFT_TURNING_CAN_ID,
          Cal.SwerveSubsystem.BACK_LEFT_CHASSIS_ANGULAR_OFFSET_RAD);

  private final SwerveModule rearRight =
      new SwerveModule(
          RobotMap.REAR_RIGHT_DRIVING_CAN_ID,
          RobotMap.REAR_RIGHT_TURNING_CAN_ID,
          Cal.SwerveSubsystem.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD);

  // The gyro sensor
  private final WPI_Pigeon2 gyro = new WPI_Pigeon2(RobotMap.PIGEON_CAN_ID);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(
          Constants.SwerveDrive.DRIVE_KINEMATICS,
          Rotation2d.fromDegrees(gyro.getYaw()),
          new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
          });

  private boolean halfSpeed = false;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    gyro.configFactoryDefault();
    gyro.reset();
    gyro.configMountPose(AxisDirection.PositiveY, AxisDirection.PositiveZ);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(
        Rotation2d.fromDegrees(gyro.getYaw()),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          rearLeft.getPosition(),
          rearRight.getPosition()
        });
  }

  public void burnFlashSparks() {
    frontLeft.burnFlashSparks();
    frontRight.burnFlashSparks();
    rearLeft.burnFlashSparks();
    rearRight.burnFlashSparks();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        Rotation2d.fromDegrees(gyro.getYaw()),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          rearLeft.getPosition(),
          rearRight.getPosition()
        },
        pose);
  }

  /**
   * Correction for swerve second order dynamics issue. Borrowed from 254:
   * https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/subsystems/Drive.java#L325
   * Discussion:
   * https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
   */
  private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
    final double LOOP_TIME_S = 0.02;
    Pose2d futureRobotPose =
        new Pose2d(
            originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
            originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
            Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
    Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
    ChassisSpeeds updatedSpeeds =
        new ChassisSpeeds(
            twistForPose.dx / LOOP_TIME_S,
            twistForPose.dy / LOOP_TIME_S,
            twistForPose.dtheta / LOOP_TIME_S);
    return updatedSpeeds;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Desired speed of the robot in the x direction (forward), [-1,1].
   * @param ySpeed Desired speed of the robot in the y direction (sideways), [-1,1].
   * @param rot Desired angular rate of the robot, [-1,1].
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // x, y, and rot are all being deadbanded from 0.1 to 0.0, so checking if
    // they're equal to 0
    // does account for controller deadzones.
    if (xSpeed == 0 && ySpeed == 0 && rot == 0) {
      setX();
      return;
    }

    // Adjust input based on max speed
    xSpeed *= Constants.SwerveDrive.MAX_SPEED_METERS_PER_SECOND;
    ySpeed *= Constants.SwerveDrive.MAX_SPEED_METERS_PER_SECOND;
    rot *= Constants.SwerveDrive.MAX_ANGULAR_SPEED_RAD_PER_SECONDS;

    if (halfSpeed) {
      xSpeed /= 2;
      ySpeed /= 2;
    }

    ChassisSpeeds desiredChassisSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rot, Rotation2d.fromDegrees(gyro.getYaw()))
            : new ChassisSpeeds(xSpeed, ySpeed, rot);

    desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);

    var swerveModuleStates =
        Constants.SwerveDrive.DRIVE_KINEMATICS.toSwerveModuleStates(desiredChassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.SwerveDrive.MAX_SPEED_METERS_PER_SECOND);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.SwerveDrive.MAX_SPEED_METERS_PER_SECOND);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    rearLeft.resetEncoders();
    frontRight.resetEncoders();
    rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeadingDegrees() {
    return Rotation2d.fromDegrees(gyro.getYaw()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyro.getRate() * (Constants.SwerveDrive.GYRO_REVERSED ? -1.0 : 1.0);
  }

  /**
   * Keeps the heading of the robot when the driver is not turning, by using PID to keep the
   * distance between the actual heading and the last intended heading to 0.
   *
   * @param x Desired speed of the robot in the x direction (forward), [-1,1].
   * @param y Desired speed of the robot in the y direction (sideways), [-1,1].
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void keepHeading(double x, double y, boolean fieldRelative) {
    double currentHeadingDegrees = getHeadingDegrees();
    double headingDifferenceDegrees = currentHeadingDegrees - targetHeadingDegrees;
    double offsetHeadingDegrees = MathUtil.inputModulus(headingDifferenceDegrees, -180, 180);

    double desiredRotation =
        Cal.SwerveSubsystem.ROTATE_TO_TARGET_PID_CONTROLLER.calculate(offsetHeadingDegrees, 0.0)
            + Math.signum(offsetHeadingDegrees) * Cal.SwerveSubsystem.ROTATE_TO_TARGET_FF;

    if (Math.abs(desiredRotation) < Cal.SwerveSubsystem.ROTATION_DEADBAND_THRESHOLD) {
      desiredRotation = 0;
    }

    drive(x, y, desiredRotation, fieldRelative);
  }

  public int convertCardinalDirections(int povAngleDeg) {
    // change d-pad values for left and right to 45 degree angles
    if (povAngleDeg == 270) {
      povAngleDeg += 45;
    } else if (povAngleDeg == 90) {
      povAngleDeg -= 45;
    }
    // targetHeadingDegrees is counterclockwise so need to flip povAngle
    povAngleDeg = 360 - povAngleDeg;
    return povAngleDeg;
  }

  /**
   * Determines whether to rotate according to input or to run the keep heading code, by checking if
   * the (already deadbanded) rotation input is equal to 0.
   *
   * @param x Desired speed of the robot in the x direction (forward), [-1,1].
   * @param y Desired speed of the robot in the y direction (sideways), [-1,1].
   * @param rot Desired angular rate of the robot, [-1,1].
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param povAngleDeg Get the angle in degrees of the D-pad (clockwise, -1 means POV not pressed).
   */
  public void rotateOrKeepHeading(
      double x, double y, double rot, boolean fieldRelative, int povAngleDeg) {
    if (povAngleDeg != -1) {
      targetHeadingDegrees = convertCardinalDirections(povAngleDeg);
      keepHeading(x, y, fieldRelative);
    } else if (rot == 0) {
      keepHeading(x, y, fieldRelative);
    } else {
      targetHeadingDegrees = getHeadingDegrees();
      drive(x, y, rot, fieldRelative);
    }
  }

  public void setForward() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  }

  public WPI_Pigeon2 getGyro() {
    return gyro;
  }

  /** Taken from Github */
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
            new RunCommand(() -> setForward())
                .withTimeout(0.1)
                .unless(
                    () -> {
                      return !isFirstPath;
                    }),
            new InstantCommand(
                () -> {
                  // Reset odometry for the first path you run during auto
                  if (isFirstPath) {
                    this.resetOdometry(traj.getInitialHolonomicPose());
                  }
                }),
            new PPSwerveControllerCommand(
                traj,
                this::getPose, // Pose supplier
                Constants.SwerveDrive.DRIVE_KINEMATICS, // SwerveDriveKinematics
                Cal.SwerveSubsystem
                    .PATH_X_CONTROLLER, // X controller. Tune these values for your robot. Leaving
                // them
                // 0 will only use feedforwards.
                Cal.SwerveSubsystem
                    .PATH_Y_CONTROLLER, // Y controller (usually the same values as X controller)
                Cal.SwerveSubsystem
                    .PATH_THETA_CONTROLLER, // Rotation controller. Tune these values for your
                // robot.
                // Leaving them 0 will only use feedforwards.
                this::setModuleStates, // Module states consumer
                false, // Should the path be automatically mirrored depending on alliance color.
                // Optional, defaults to true
                this // Requires this drive subsystem
                ),
            new InstantCommand(
                () -> {
                  targetHeadingDegrees = getPose().getRotation().getDegrees();
                }))
        .withName("Follow trajectory");
  }

  public Command transformToPath(Transform2d transform) {
    // Transform is to get the limelight to the correct location, not to get the robot
    // Here we correct for that
    Transform2d flipTransform =
        new Transform2d(
            new Translation2d(-transform.getX(), transform.getY()), transform.getRotation());
    System.out.println("Transform:");
    System.out.println(flipTransform.getX());
    System.out.println(flipTransform.getY());
    Pose2d curPose = getPose();
    Transform2d curPoseTransform = new Transform2d(curPose.getTranslation(), curPose.getRotation());
    Transform2d finalTransform = curPoseTransform.plus(flipTransform);
    Rotation2d startHeading = flipTransform.getTranslation().getAngle().plus(curPose.getRotation());
    Rotation2d finalHeading = startHeading.plus(Rotation2d.fromDegrees(180));
    PathPlannerTrajectory path =
        PathPlanner.generatePath(
            new PathConstraints(1.0, 1.0),
            new PathPoint(curPose.getTranslation(), startHeading, curPose.getRotation()),
            // We know the robot needs to be at zero relative to start of match so let's just use
            // that
            new PathPoint(
                finalTransform.getTranslation(), finalHeading, Rotation2d.fromDegrees(0)));
    return followTrajectoryCommand(path, false);
  }

  public void toggleSkids() {
    // TODO do this once we can add skids
  }

  public void halfSpeedToggle() {
    // Toggle halfSpeed. If it is true, set it to false, otherwise set it to true.
    halfSpeed = !halfSpeed;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    addChild("X Controller", Cal.SwerveSubsystem.PATH_X_CONTROLLER);
    addChild("Y Controller", Cal.SwerveSubsystem.PATH_Y_CONTROLLER);
    addChild("Theta Controller", Cal.SwerveSubsystem.PATH_THETA_CONTROLLER);
    addChild("Rotate to target controller", Cal.SwerveSubsystem.ROTATE_TO_TARGET_PID_CONTROLLER);
    addChild("Front Right", frontRight);
    addChild("Front Left", frontLeft);
    addChild("Rear Right", rearRight);
    addChild("Rear Left", rearLeft);
    builder.addBooleanProperty(
        "Half speed?",
        () -> {
          return halfSpeed;
        },
        null);
    builder.addDoubleProperty(
        "Target Heading (deg)",
        () -> {
          return targetHeadingDegrees;
        },
        null);
    builder.addDoubleProperty("Gyro Yaw (deg)", gyro::getYaw, null);
    builder.addDoubleProperty(
        "Odometry X (m)",
        () -> {
          return getPose().getX();
        },
        null);
    builder.addDoubleProperty(
        "Odometry Y (m)",
        () -> {
          return getPose().getY();
        },
        null);
    builder.addDoubleProperty(
        "Odometry Yaw (deg)",
        () -> {
          return getPose().getRotation().getDegrees();
        },
        null);
  }
}
