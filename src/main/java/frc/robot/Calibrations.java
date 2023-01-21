package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.SwerveModuleConstants;

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
        DRIVING_FF = 1 / SwerveModuleConstants.DRIVE_WHEEL_FREE_SPEED_METERS_PER_SECOND;

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

  public static final double ELEVATOR_DEPLOYING_POWER = 1.0;
  public static final double ELEVATOR_RETRACTING_POWER = -1.0;

  public static final double ARM_UNFOLDING_POWER = 1.0;
  public static final double ARM_FOLDING_POWER = -1.0;

  /** Input rotations, output [0,1] */
  public static final double ELEVATOR_P = PLACEHOLDER_DOUBLE,
  ELEVATOR_I = PLACEHOLDER_DOUBLE,
  ELEVATOR_D = PLACEHOLDER_DOUBLE;

  /** Input rotations, output [0,1] */
  public static final double ARM_P = PLACEHOLDER_DOUBLE,
  ARM_I = PLACEHOLDER_DOUBLE,
  ARM_D = PLACEHOLDER_DOUBLE;
}
