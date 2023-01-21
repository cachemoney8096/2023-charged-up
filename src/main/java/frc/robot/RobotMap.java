package frc.robot;

/** This class is used to store IDs for physical connections to the robot, like CAN IDs. */
public final class RobotMap {
  public static final int DRIVER_CONTROLLER_PORT = 0;

  /** Driving SPARK MAX CAN IDs */
  public static final int FRONT_LEFT_DRIVING_CAN_ID = 11,
      REAR_LEFT_DRIVING_CAN_ID = 13,
      FRONT_RIGHT_DRIVING_CAN_ID = 15,
      REAR_RIGHT_DRIVING_CAN_ID = 17;

  /** Turning SPARK MAX CAN IDs */
  public static final int FRONT_LEFT_TURNING_CAN_ID = 10,
      REAR_LEFT_TURNING_CAN_ID = 12,
      FRONT_RIGHT_TURNING_CAN_ID = 14,
      REAR_RIGHT_TURNING_CAN_ID = 16;

  /** Intake deployment pneumatic channels */
  public static final int INTAKE_DEPLOY_LEFT_FORWARD_CHANNEL = 0,
      INTAKE_DEPLOY_LEFT_REVERSE_CHANNEL = 1,
      INTAKE_DEPLOY_RIGHT_FORWARD_CHANNEL = 2,
      INTAKE_DEPLOY_RIGHT_REVERSE_CHANNEL = 3;

  /** Intake clamp pneumatic channels */
  public static final int INTAKE_CLAMP_LEFT_FORWARD_CHANNEL = 4,
      INTAKE_CLAMP_LEFT_REVERSE_CHANNEL = 5,
      INTAKE_CLAMP_RIGHT_FORWARD_CHANNEL = 6,
      INTAKE_CLAMP_RIGHT_REVERSE_CHANNEL = 7;

  /** Intake Spark MAX CAN IDs */
  public static final int INTAKE_LEFT_MOTOR_CAN_ID = 22,
    INTAKE_RIGHT_MOTOR_CAN_ID = 23;

  /** Lift Spark MAX CAN IDs */
  public static final int ELEVATOR_MOTOR_CAN_ID = 37,
    ARM_MOTOR_CAN_ID = 38;
    
  public static final int PIGEON_CAN_ID = 1;
}