// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double PLACEHOLDER_DOUBLE = 0.0;
  public static final int PLACEHOLDER_INT = 0;
  public static final String PLACEHOLDER_STRING = "";

  /** This is the maximum pitch of the charge station in degrees. */
  public static final double MAX_PITCH_DEGREES = 15;

  public static final double REVOLUTIONS_TO_DEGREES = 360.0;

  public static final class SwerveModule {
    /**
     * Invert the turning encoder, since the output shaft rotates in the opposite direction of the
     * steering motor in the MAXSwerve Module.
     */
    public static final boolean TURNING_ENCODER_INVERTED = true;

    /** Calculations required for driving motor conversion factors and feed forward */
    public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotor.FREE_SPEED_RPM / 60,
        WHEEL_DIAMETER_METERS = Units.inchesToMeters(3),
        WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

    public static final double DRIVING_MOTOR_REDUCTION = PLACEHOLDER_DOUBLE;
    public static final double DRIVE_WHEEL_FREE_SPEED_FUDGE_FACTOR = 1.0;
    public static final double DRIVE_WHEEL_FREE_SPEED_METERS_PER_SECOND =
        DRIVE_WHEEL_FREE_SPEED_FUDGE_FACTOR
            * ((DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS)
                / DRIVING_MOTOR_REDUCTION);

    public static final double DRIVING_ENCODER_POSITION_FACTOR_METERS =
        (WHEEL_DIAMETER_METERS * Math.PI) / DRIVING_MOTOR_REDUCTION; // meters
    public static final double DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND =
        ((WHEEL_DIAMETER_METERS * Math.PI) / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second

    public static final double TURNING_ENCODER_POSITION_FACTOR_RADIANS = (2 * Math.PI); // radians
    public static final double TURNING_ENCODER_VELOCITY_FACTOR_RAD_PER_SEC =
        (2 * Math.PI) / 60.0; // radians per second

    public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS = 0; // radians
    public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS =
        TURNING_ENCODER_POSITION_FACTOR_RADIANS; // radians

    public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

    public static final int DRIVING_MOTOR_CURRENT_LIMIT_AMPS = 50; // amps
    public static final int TURNING_MOTOR_CURRENT_LIMIT_AMPS = 20; // amps
  }

  public static final class NeoMotor {
    public static final double FREE_SPEED_RPM = 5676;
  }

  public static final class SwerveDrive {
    /**
     * Driving Parameters - Note that these are not the maximum capable speeds of the robot, rather
     * the allowed maximum speeds
     */
    public static final double MAX_SPEED_METERS_PER_SECOND = 4,
        MAX_ANGULAR_SPEED_RAD_PER_SECONDS = 2 * Math.PI; // radians per second

    /** Chassis configuration */
    public static final double TRACK_WIDTH_METERS = PLACEHOLDER_DOUBLE;

    /** Distance between centers of right and left wheels on robot */
    public static final double WHEEL_BASE_METERS = PLACEHOLDER_DOUBLE;

    /** Distance between front and back wheels on robot */
    public static final SwerveDriveKinematics DRIVE_KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
            new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),
            new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
            new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2));

    /** Angular offsets of the modules relative to the chassis in radians */
    public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2,
        FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0,
        BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI,
        BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

    public static final boolean GYRO_REVERSED = false;
  }

  public static final class Lift {
    /* Scalar for elevator motor encoder from RPM to real inches per seconds */
    public static final double ELEVATOR_MOTOR_ENCODER_VELOCITY_SCALAR =
        Constants.PLACEHOLDER_DOUBLE;

    /* Scalar for elevator motor encoder from rotations to real inches */
    public static final double ELEVATOR_MOTOR_ENCODER_SCALAR = Constants.PLACEHOLDER_DOUBLE;

    /* Scalar for arm motor encoder from RPM to real degrees per seconds */
    public static final double ARM_MOTOR_ENCODER_VELOCITY_SCALAR = Constants.PLACEHOLDER_DOUBLE;

    /* Scalar for arm motor encoder from rotations to real degrees */
    public static final double ARM_MOTOR_ENCODER_SCALAR = Constants.PLACEHOLDER_DOUBLE;

    public static final double ARM_POSITION_WHEN_HORIZONTAL_DEGREES = 90;

    /** Scalar for the difference in the elevator's absolute encoders in inches per degree */
    public static final double ELEVATOR_MOTOR_ENCODER_DIFFERENCES_SCALAR_INCHES_PER_DEGREE =
        Constants.PLACEHOLDER_DOUBLE;
  }

  public static final class Intake {
    /* Scalar for deploy motor encoder from RPM to real degrees per seconds */
    public static final double DEPLOY_MOTOR_ENCODER_VELOCITY_SCALAR = Constants.PLACEHOLDER_DOUBLE;

    /* Scalar for arm motor encoder from rotations to real degrees */
    public static final double DEPLOY_MOTOR_ENCODER_SCALAR = Constants.PLACEHOLDER_DOUBLE;

    public static final double POSITION_WHEN_HORIZONTAL_DEGREES = 180;
  }
}
