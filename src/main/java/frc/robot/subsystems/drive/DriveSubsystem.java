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
import edu.wpi.first.math.filter.MedianFilter;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Cal;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.Lights;
import frc.robot.utils.GeometryUtils;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public class DriveSubsystem extends SubsystemBase {
  private double targetHeadingDegrees;

  private Lights lights;

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
  private ChassisSpeeds lastSetChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  public Optional<Pose2d> targetPose = Optional.empty();
  private BooleanSupplier throttleForLift;
  public boolean generatedPath = false;
  private MedianFilter pitchFilter = new MedianFilter(5);
  private double latestFilteredPitchDeg = 0.0;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(
          Constants.SwerveDrive.DRIVE_KINEMATICS,
          Rotation2d.fromDegrees(0.0),
          getModulePositions());

  /** Multiplier for drive speed, does not affect trajectory following */
  private double throttleMultiplier = 1.0;

  private BooleanSupplier isTimedMatch;

  /**
   * Creates a new DriveSubsystem.
   *
   * @param throttleForLiftFunc Function to check if we should throttle due to lift position.
   */
  public DriveSubsystem(
      BooleanSupplier throttleForLiftFunc,
      Lights lightsSubsystem,
      BooleanSupplier isTimedMatchFunc) {
    throttleForLift = throttleForLiftFunc;
    gyro.configFactoryDefault();
    gyro.reset();
    gyro.configMountPose(AxisDirection.PositiveY, AxisDirection.PositiveZ);
    lights = lightsSubsystem;
    isTimedMatch = isTimedMatchFunc;
  }

  public double getFilteredPitch() {
    return latestFilteredPitchDeg - Cal.SwerveSubsystem.IMU_PITCH_BIAS_DEG;
  }

  @Override
  public void periodic() {
    latestFilteredPitchDeg = pitchFilter.calculate(gyro.getPitch());

    // Update the odometry in the periodic block
    frontLeft.periodic();
    frontRight.periodic();
    rearLeft.periodic();
    rearRight.periodic();
    odometry.update(Rotation2d.fromDegrees(gyro.getYaw()), getModulePositions());
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      rearLeft.getPosition(),
      rearRight.getPosition()
    };
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
    odometry.resetPosition(Rotation2d.fromDegrees(gyro.getYaw()), getModulePositions(), pose);
  }

  public void resetYaw() {
    gyro.reset();
    Pose2d curPose = getPose();
    Pose2d resetPose = new Pose2d(curPose.getTranslation(), Rotation2d.fromDegrees(0));
    odometry.resetPosition(Rotation2d.fromDegrees(0), getModulePositions(), resetPose);
    targetHeadingDegrees = 0.0;
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

  /** Keep modules in current position, don't drive */
  public void setNoMove() {
    Rotation2d frontLeftCurrRot = frontLeft.getPosition().angle;
    Rotation2d frontRightCurrRot = frontRight.getPosition().angle;
    Rotation2d rearLeftCurrRot = rearLeft.getPosition().angle;
    Rotation2d rearRightCurrRot = rearRight.getPosition().angle;
    frontLeft.setDesiredState(new SwerveModuleState(0, frontLeftCurrRot));
    frontRight.setDesiredState(new SwerveModuleState(0, frontRightCurrRot));
    rearLeft.setDesiredState(new SwerveModuleState(0, rearLeftCurrRot));
    rearRight.setDesiredState(new SwerveModuleState(0, rearRightCurrRot));
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
    if (isTimedMatch.getAsBoolean()
        && DriverStation.isTeleop()
        && DriverStation.getMatchTime() < 0.3) {
      setX();
      return;
    }

    // x, y, and rot are all being deadbanded from 0.1 to 0.0, so checking if
    // they're equal to 0
    // does account for controller deadzones.
    if (xSpeed == 0 && ySpeed == 0 && rot == 0) {
      setNoMove();
      return;
    }

    // Adjust input based on max speed
    xSpeed *= Constants.SwerveDrive.MAX_SPEED_METERS_PER_SECOND;
    ySpeed *= Constants.SwerveDrive.MAX_SPEED_METERS_PER_SECOND;
    rot *= Constants.SwerveDrive.MAX_ANGULAR_SPEED_RAD_PER_SECONDS;

    if (throttleForLift.getAsBoolean()) {
      xSpeed *= Cal.SwerveSubsystem.THROTTLE_FOR_SCORING_AND_SHELF;
      ySpeed *= Cal.SwerveSubsystem.THROTTLE_FOR_SCORING_AND_SHELF;
      rot *= Cal.SwerveSubsystem.THROTTLE_FOR_SCORING_AND_SHELF;
    } else {
      xSpeed *= throttleMultiplier;
      ySpeed *= throttleMultiplier;
      rot *= throttleMultiplier;
    }

    ChassisSpeeds desiredChassisSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rot, Rotation2d.fromDegrees(gyro.getYaw()))
            : new ChassisSpeeds(xSpeed, ySpeed, rot);

    desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);
    lastSetChassisSpeeds = desiredChassisSpeeds;

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
    lights.setPartyMode();
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    targetHeadingDegrees = getHeadingDegrees();
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

    double pidRotation =
        Cal.SwerveSubsystem.ROTATE_TO_TARGET_PID_CONTROLLER.calculate(offsetHeadingDegrees, 0.0);
    double ffRotation = Math.signum(offsetHeadingDegrees) * Cal.SwerveSubsystem.ROTATE_TO_TARGET_FF;

    double desiredRotation = pidRotation - ffRotation;

    if (Math.abs(desiredRotation) < Cal.SwerveSubsystem.ROTATION_DEADBAND_THRESHOLD) {
      desiredRotation = 0;
    }

    drive(x, y, desiredRotation, fieldRelative);
  }

  public int convertCardinalDirections(int povAngleDeg) {
    // change d-pad values for left and right to 20 degree angles
    if (povAngleDeg == 270) {
      povAngleDeg += 70;
    } else if (povAngleDeg == 90) {
      povAngleDeg -= 70;
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
                Constants.SwerveDrive.DRIVE_KINEMATICS,
                Cal.SwerveSubsystem.PATH_X_CONTROLLER,
                Cal.SwerveSubsystem.PATH_Y_CONTROLLER, // usually the same values as X controller
                Cal.SwerveSubsystem.PATH_THETA_CONTROLLER,
                this::setModuleStates, // Module states consumer
                false, // Should the path be automatically mirrored depending on alliance color.
                this // Requires this drive subsystem
                ),
            new InstantCommand(
                () -> {
                  targetHeadingDegrees = getHeadingDegrees();
                }),
            new PrintCommand("Finished a trajectory"))
        .withName("Follow trajectory");
  }

  public Pose2d getPastPose(double latencySec) {
    Pose2d curPose = getPose();
    double latencyAdjustmentSec = 0.00;
    latencySec += latencyAdjustmentSec;
    Transform2d pastTransform =
        new Transform2d(
            new Translation2d(
                -lastSetChassisSpeeds.vxMetersPerSecond * latencySec,
                -lastSetChassisSpeeds.vyMetersPerSecond * latencySec),
            Rotation2d.fromRadians(lastSetChassisSpeeds.omegaRadiansPerSecond * latencySec)
                .unaryMinus());
    Pose2d pastPose = curPose.plus(pastTransform);
    return pastPose;
  }

  public void setLimelightTargetFromTransform(Transform2d transform, double latencySec) {
    // Transform is to get the limelight to the correct location, not to get the robot
    // Here we correct for that
    Transform2d flipTransform =
        new Transform2d(
            new Translation2d(-transform.getX(), transform.getY()), transform.getRotation());
    System.out.println("Flip Transform: " + flipTransform.getX() + " " + flipTransform.getY());

    Pose2d curPose = getPose();
    double latencyAdjustmentSec = 0.00;
    latencySec += latencyAdjustmentSec;
    Transform2d pastTransform =
        new Transform2d(
            new Translation2d(
                -lastSetChassisSpeeds.vxMetersPerSecond * latencySec,
                -lastSetChassisSpeeds.vyMetersPerSecond * latencySec),
            Rotation2d.fromRadians(lastSetChassisSpeeds.omegaRadiansPerSecond * latencySec)
                .unaryMinus());
    Pose2d pastPose = curPose.plus(pastTransform);

    final boolean useLatencyAdjustment = true;

    targetPose =
        useLatencyAdjustment
            ? Optional.of(pastPose.plus(flipTransform))
            : Optional.of(curPose.plus(flipTransform));
  }

  public PathPlannerTrajectory pathToPoint(Pose2d finalPose, double finalSpeedMetersPerSec) {
    Pose2d curPose = getPose();
    Transform2d finalTransform =
        new Transform2d(finalPose.getTranslation(), finalPose.getRotation());
    System.out.println(
        "Trajectory Transform: " + finalTransform.getX() + " " + finalTransform.getY());
    Rotation2d finalHeading = Rotation2d.fromDegrees(180);
    Rotation2d finalHolonomicRotation = finalPose.getRotation();
    PathPlannerTrajectory path =
        PathPlanner.generatePath(
            new PathConstraints(
                Cal.SwerveSubsystem.MEDIUM_LINEAR_SPEED_METERS_PER_SEC,
                Cal.SwerveSubsystem.MEDIUM_LINEAR_ACCELERATION_METERS_PER_SEC_SQ),
            PathPoint.fromCurrentHolonomicState(curPose, lastSetChassisSpeeds),
            new PathPoint(
                    finalTransform.getTranslation(),
                    finalHeading,
                    finalHolonomicRotation,
                    finalSpeedMetersPerSec)
                .withPrevControlLength(0.01));
    return path;
  }

  public Optional<PathPlannerTrajectory> poseToPath() {
    Pose2d curPose = getPose();
    double coastLatencySec = 0.00;
    Transform2d coastTransform =
        new Transform2d(
            new Translation2d(
                lastSetChassisSpeeds.vxMetersPerSecond * coastLatencySec,
                lastSetChassisSpeeds.vyMetersPerSecond * coastLatencySec),
            Rotation2d.fromRadians(lastSetChassisSpeeds.omegaRadiansPerSecond * coastLatencySec));
    Pose2d futurePose = curPose.plus(coastTransform);

    System.out.println("Acquired target? " + targetPose.isPresent());
    if (!targetPose.isPresent()) {
      return Optional.empty();
    }

    Transform2d trajectoryTransform = targetPose.get().minus(futurePose);
    System.out.println(
        "Trajectory Transform: " + trajectoryTransform.getX() + " " + trajectoryTransform.getY());
    // TODO we should probably just get rid of this guess if there's no target pose
    Pose2d finalPose = targetPose.get();
    Transform2d finalTransform =
        new Transform2d(finalPose.getTranslation(), finalPose.getRotation());
    Rotation2d finalHeading = Rotation2d.fromDegrees(180);
    Rotation2d finalHolonomicRotation = Rotation2d.fromDegrees(0);
    PathPlannerTrajectory path =
        PathPlanner.generatePath(
            new PathConstraints(
                Cal.SwerveSubsystem.SLOW_LINEAR_SPEED_METERS_PER_SEC,
                Cal.SwerveSubsystem.SLOW_LINEAR_ACCELERATION_METERS_PER_SEC_SQ),
            PathPoint.fromCurrentHolonomicState(futurePose, lastSetChassisSpeeds),
            new PathPoint(finalTransform.getTranslation(), finalHeading, finalHolonomicRotation)
                .withPrevControlLength(0.5));
    generatedPath = true;
    return Optional.of(path);
  }

  public void toggleSkids() {
    // TODO do this once we can add skids
  }

  /** Driving inputs will get multiplied by the throttle value, so it should be in [0,1] */
  public void throttle(double throttleValue) {
    throttleMultiplier = throttleValue;
  }

  public void offsetCurrentHeading(double offsetDegrees) {
    targetHeadingDegrees = getHeadingDegrees() + offsetDegrees;
  }

  public void setZeroTargetHeading() {
    targetHeadingDegrees = 0.0;
  }

  public void stopDriving() {
    drive(0, 0, 0, true);
  }

  public Command stopDrivingCommand() {
    return new InstantCommand(this::stopDriving, this);
  }

  public void zeroFrontLeftAtCurrentPos() {
    Cal.SwerveSubsystem.SWERVE_FRONT_LEFT_ANGULAR_OFFSET_RAD = frontLeft.getEncoderAbsPositionRad();
    System.out.println(
        "New Zero for Front Left Swerve: "
            + Cal.SwerveSubsystem.SWERVE_FRONT_LEFT_ANGULAR_OFFSET_RAD);
  }

  public void zeroFrontRightAtCurrentPos() {
    Cal.SwerveSubsystem.SWERVE_FRONT_RIGHT_ANGULAR_OFFSET_RAD =
        frontRight.getEncoderAbsPositionRad();
    System.out.println(
        "New Zero for Front Right Swerve: "
            + Cal.SwerveSubsystem.SWERVE_FRONT_RIGHT_ANGULAR_OFFSET_RAD);
  }

  public void zeroRearLeftAtCurrentPos() {
    Cal.SwerveSubsystem.SWERVE_BACK_LEFT_ANGULAR_OFFSET_RAD = rearLeft.getEncoderAbsPositionRad();
    System.out.println(
        "New Zero for Rear Left Swerve: "
            + Cal.SwerveSubsystem.SWERVE_BACK_LEFT_ANGULAR_OFFSET_RAD);
  }

  public void zeroRearRightAtCurrentPos() {
    Cal.SwerveSubsystem.SWERVE_BACK_RIGHT_ANGULAR_OFFSET_RAD = rearRight.getEncoderAbsPositionRad();
    System.out.println(
        "New Zero for Rear Right Swerve: "
            + Cal.SwerveSubsystem.SWERVE_BACK_RIGHT_ANGULAR_OFFSET_RAD);
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
    builder.addDoubleProperty("Filtered pitch deg", this::getFilteredPitch, null);
    builder.addDoubleProperty(
        "Throttle multiplier",
        () -> {
          return throttleMultiplier;
        },
        null);
    builder.addBooleanProperty("Throttle for lift", () -> throttleForLift.getAsBoolean(), null);
    builder.addDoubleProperty(
        "Target Heading (deg)",
        () -> {
          return targetHeadingDegrees;
        },
        null);
    builder.addDoubleProperty("Gyro Yaw (deg)", gyro::getYaw, null);
    builder.addDoubleProperty("Odometry X (m)", () -> getPose().getX(), null);
    builder.addDoubleProperty("Odometry Y (m)", () -> getPose().getY(), null);
    builder.addDoubleProperty(
        "Odometry Yaw (deg)", () -> getPose().getRotation().getDegrees(), null);
    builder.addDoubleProperty(
        "Front Left Abs Encoder (rad)", frontLeft::getEncoderAbsPositionRad, null);

    builder.addDoubleProperty(
        "Rear Right Abs Encoder (rad)", rearRight::getEncoderAbsPositionRad, null);
    builder.addDoubleProperty(
        "Front Left Module Pos (rad)", () -> frontLeft.getPosition().angle.getRadians(), null);
    builder.addDoubleProperty(
        "Front Right Module Pos (rad)", () -> frontRight.getPosition().angle.getRadians(), null);
    builder.addDoubleProperty(
        "Rear Left Module Pos (rad)", () -> rearLeft.getPosition().angle.getRadians(), null);
    builder.addDoubleProperty(
        "Rear Right Module Pos (rad)", () -> rearRight.getPosition().angle.getRadians(), null);
    builder.addDoubleProperty(
        "Front Left Distance (m)", () -> frontLeft.getPosition().distanceMeters, null);
    builder.addDoubleProperty(
        "Front Right Distance (m)", () -> frontRight.getPosition().distanceMeters, null);
    builder.addDoubleProperty(
        "Rear Left Distance (m)", () -> rearLeft.getPosition().distanceMeters, null);
    builder.addDoubleProperty(
        "Rear Right Distance (m)", () -> rearRight.getPosition().distanceMeters, null);
  }
}
