package frc.robot;

import edu.wpi.first.math.controller.PIDController;

/** This class provides a place for Calibrations: arbitrary but tuned values, like PID values. */
public final class Cal {
  public static final double PLACEHOLDER_DOUBLE = 0.0;
  public static final int PLACEHOLDER_INT = 0;

  public static final class SwerveModule {
    /** Input meters/second, output [-1,1] */
    public static final double DRIVING_P = PLACEHOLDER_DOUBLE,
        DRIVING_I = PLACEHOLDER_DOUBLE,
        DRIVING_D = PLACEHOLDER_DOUBLE,
        DRIVING_FF = 1 / Constants.SwerveModule.DRIVE_WHEEL_FREE_SPEED_METERS_PER_SECOND;

    public static final double DRIVING_MIN_OUTPUT = -1;
    public static final double DRIVING_MAX_OUTPUT = 1;

    /** Input radians, output [-1,1] */
    public static final double TURNING_P = PLACEHOLDER_DOUBLE,
        TURNING_I = PLACEHOLDER_DOUBLE,
        TURNING_D = PLACEHOLDER_DOUBLE,
        TURNING_FF = 0;

    public static final double TURNING_MIN_OUTPUT = -1;
    public static final double TURNING_MAX_OUTPUT = 1;
  }

  public static final class SwerveSubsystem {
    /**
     * Controller on module speed for rotating to target, input degrees [-180,180], output [0,1].
     */
    public static final PIDController ROTATE_TO_TARGET_PID_CONTROLLER =
        new PIDController(0.030, 0, 0.000); // From 2022

    /** Feed forward for rotating to target, gets added to or subtracted from PID controller. */
    public static final double ROTATE_TO_TARGET_FF = 0.1; // From 2022

    /** Auton path finding controllers */
    public static final PIDController PATH_X_CONTROLLER = new PIDController(0.100506, 0.0, 0.0),
        PATH_Y_CONTROLLER = new PIDController(0.1, 0.0, 0.0);

    /** High profile constraints = pure P controller */
    public static final PIDController PATH_THETA_CONTROLLER = new PIDController(9.0, 0.0, 0.80);
  }

  public static final class Intake {
    public static final double INTAKING_POWER = 1.0;
    public static final double EJECTION_POWER = -1.0;

    public static final double AUTO_CLAMP_WAIT_TIME_SECONDS = 0.5;

    /** Input degrees, output [0,1] */
    public static final double DEPLOY_MOTOR_P = Cal.PLACEHOLDER_DOUBLE,
        DEPLOY_MOTOR_I = Cal.PLACEHOLDER_DOUBLE,
        DEPLOY_MOTOR_D = Cal.PLACEHOLDER_DOUBLE;

    /** Intake positions in degrees */
    public static final double STARTING_POSITION_DEGREES = Cal.PLACEHOLDER_DOUBLE,
        DEPLOYED_POSITION_DEGREES = Cal.PLACEHOLDER_DOUBLE;

    /** Difference in what the intake absolute encoder says is 0 and what is actually 0 */
    public static final double ABSOLUTE_ENCODER_OFFSET_DEG = Cal.PLACEHOLDER_DOUBLE;

    /** Voltage required to hold the intake in the horizontal position */
    public static final double ARBITRARY_FEED_FORWARD_VOLTS = Cal.PLACEHOLDER_DOUBLE;

    /** Parameters for intake SmartMotion */
    public static final double
        DEPLOY_MAX_ACCELERATION_DEG_PER_SECOND_SQUARED = Cal.PLACEHOLDER_DOUBLE,
        DEPLOY_MAX_VELOCITY_DEG_PER_SECOND = Cal.PLACEHOLDER_DOUBLE,
        DEPLOY_MIN_OUTPUT_VELOCITY_DEG_PER_SECOND = Cal.PLACEHOLDER_DOUBLE,
        DEPLOY_ALLOWED_CLOSED_LOOP_ERROR_DEG = Cal.PLACEHOLDER_DOUBLE;

    /** Threshold for having achieved the desired intake position (in degrees) */
    public static final double POSITION_THRESHOLD_DEGREES = 3.0;
  }

  public static final class Lift {
    /** Input inches, output [0,1] */
    public static final double ELEVATOR_P = Cal.PLACEHOLDER_DOUBLE,
        ELEVATOR_I = Cal.PLACEHOLDER_DOUBLE,
        ELEVATOR_D = Cal.PLACEHOLDER_DOUBLE;

    /** Input degrees, output [0,1] */
    public static final double ARM_P = Cal.PLACEHOLDER_DOUBLE,
        ARM_I = Cal.PLACEHOLDER_DOUBLE,
        ARM_D = Cal.PLACEHOLDER_DOUBLE;

    /** Difference in what the arm absolute encoder says is 0 and what is actually 0 */
    public static final double ARM_ABSOLUTE_ENCODER_OFFSET_DEG = Cal.PLACEHOLDER_DOUBLE;

    /** Voltage required to hold the arm in the horizontal position */
    public static final double ARBITRARY_ARM_FEED_FORWARD_VOLTS = Cal.PLACEHOLDER_DOUBLE;

    /** Voltage required to hold the elevator */
    public static final double ARBITRARY_ELEVATOR_FEED_FORWARD_VOLTS = Cal.PLACEHOLDER_DOUBLE;

    /** Parameters for arm SmartMotion */
    public static final double ARM_MAX_ACCELERATION_DEG_PER_SECOND_SQUARED = Cal.PLACEHOLDER_DOUBLE,
        ARM_MAX_VELOCITY_DEG_PER_SECOND = Cal.PLACEHOLDER_DOUBLE,
        ARM_MIN_OUTPUT_VELOCITY_DEG_PER_SECOND = Cal.PLACEHOLDER_DOUBLE,
        ARM_ALLOWED_CLOSED_LOOP_ERROR_DEG = Cal.PLACEHOLDER_DOUBLE;

    /** Parameters for elevator SmartMotion */
    public static final double
        ELEVATOR_MAX_ACCELERATION_IN_PER_SECOND_SQUARED = Cal.PLACEHOLDER_DOUBLE,
        ELEVATOR_MAX_VELOCITY_IN_PER_SECOND = Cal.PLACEHOLDER_DOUBLE,
        ELEVATOR_MIN_OUTPUT_VELOCITY_IN_PER_SECOND = Cal.PLACEHOLDER_DOUBLE,
        ELEVATOR_ALLOWED_CLOSED_LOOP_ERROR_IN = Cal.PLACEHOLDER_DOUBLE;

    /**
     * Thresholds for when we consider the lift has reached a position. This is logical (for
     * considering where the lift can go) but not functional (does not stop arm control or something
     * when reached). We apply broader thresholds for the Starting position as the lift transits
     * through this position.
     */
    public static final double ELEVATOR_THRESHOLD_INCHES = 0.5,
        ARM_THRESHOLD_DEGREES = 2.0,
        ELEVATOR_START_THRESHOLD_INCHES = 1.0,
        ARM_START_THRESHOLD_DEGREES = 4.0;
  }

  public static final class AutoBalance {
    /** Max speed to go on charge station in meters per second */
    public static final double MAX_CHARGE_STATION_CLIMB_SPEED_MPS = 0.5;

    /** Max speed to go on charge station in [-1,1] */
    public static final double MAX_CHARGE_STATION_CLIMB_NORM_SPEED =
        MAX_CHARGE_STATION_CLIMB_SPEED_MPS / Constants.SwerveDrive.MAX_SPEED_METERS_PER_SECOND;

    /**
     * Conversion factor between the robot pitch (in degrees) to a velocity (in [-1,1]) for
     * balancing on the charge station
     */
    public static final double CHARGE_STATION_PITCH_DEGREES_TO_NORM_VELOCITY =
        -1 * Constants.MAX_PITCH_DEGREES / MAX_CHARGE_STATION_CLIMB_NORM_SPEED;

    /**
     * Maximum speed to deadband at (that is to say, it won't auto-balance if the velocity is <=
     * 0.05, for a velocity of [-1,1])
     */
    public static final double CHARGE_STATION_DEADBAND_NORM_VELOCITY = 0.05;
  }

  public static final int SPARK_INIT_RETRY_ATTEMPTS = 5;
}
