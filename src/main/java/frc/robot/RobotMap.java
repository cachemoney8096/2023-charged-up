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
  public static final int DEPLOY_MOTOR_CAN_ID = 0;

  /** Intake clamp pneumatic channels */
  public static final int INTAKE_CLAMP_FORWARD_CHANNEL = 4;

  /** Intake Spark MAX CAN IDs */
  public static final int INTAKE_LEFT_MOTOR_CAN_ID = 22, INTAKE_RIGHT_MOTOR_CAN_ID = 23;

  /** Lift Spark MAX CAN IDs */
  public static final int ELEVATOR_MOTOR_CAN_ID = 37, ARM_MOTOR_CAN_ID = 38;

  /** Lift grabbing pneumatic channels */
  public static final int LIFT_GRABBING_FORWARD_CHANNEL = 40, LIFT_GRABBING_REVERSE_CHANNEL = 41;

  /** Sensor DIO */
  public static final int LIFT_GAME_PIECE_DIO = 42;
  public static final int INTAKE_GAME_PIECE_DIO = 43;

  public static final int PIGEON_CAN_ID = 1;

}
