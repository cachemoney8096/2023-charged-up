package frc.robot;

import edu.wpi.first.math.controller.PIDController;

/** This class provides a place for arbitrary but tuned values, like PID values. */
public final class Calibrations {
  public static final double PLACEHOLDER_DOUBLE = 1.0;
  public static final int PLACEHOLDER_INT = 1;

  /** Controller on module speed for rotating to target, input degrees [-180,180], output [0,1]. */
  public static final PIDController ROTATE_TO_TARGET_PID_CONTROLLER =
      new PIDController(0.030, 0, 0.000); // From 2022
  /** Feed forward for rotating to target, gets added to or subtracted from PID controller. */
  public static final double ROTATE_TO_TARGET_FF = 0.1; // From 2022

  public static final class SwerveModuleCalibrations {
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

  public static final double INTAKE_INTAKING_POWER = 1.0;
  public static final double INTAKE_EJECTION_POWER = -1.0;
  public static final double AUTO_CLAMP_WAIT_TIME_SECONDS = 0.5;


  /** Max speed to go on charge station in meters per second */
  public static final double MAX_CHARGE_STATION_CLIMB_SPEED_MPS = 0.5;
  /** Max speed to go on charge station in [-1,1] */
  public static final double MAX_CHARGE_STATION_CLIMB_SPEED = MAX_CHARGE_STATION_CLIMB_SPEED_MPS / Constants.SwerveDrive.MAX_SPEED_METERS_PER_SECOND;
  /** Conversion factor between the robot pitch (in degrees) to a velocity (in [-1,1]) for balancing on the charge station */
  public static final double CHARGE_STATION_PITCH_DEGREES_TO_VELOCITY = -1 * Constants.MAX_PITCH_DEGREES / MAX_CHARGE_STATION_CLIMB_SPEED;
  /** Maximum speed to deadband at (that is to say, it won't auto-balance if the velocity is <= 0.05, for a velocity of [-1,1]) */
  public static final double CHARGE_STATION_DEADBAND_VELOCITY = 0.05;
}
