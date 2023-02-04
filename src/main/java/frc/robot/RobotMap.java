package frc.robot;

/** This class is used to store IDs for physical connections to the robot, like CAN IDs. */
public final class RobotMap {
  public static final int DRIVER_CONTROLLER_PORT = 0;

  public static final int PIGEON_CAN_ID = 1;

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

  /** Intake SPARK MAX CAN IDs */
  public static final int INTAKE_DEPLOY_MOTOR_CAN_ID = 18,
      INTAKE_LEFT_MOTOR_CAN_ID = 22,
      INTAKE_RIGHT_MOTOR_CAN_ID = 23;

  /** Lift Spark MAX CAN IDs */
  public static final int ELEVATOR_MOTOR_CAN_ID = 37, ARM_MOTOR_CAN_ID = 38;

  /** Intake clamp pneumatic channels */
  public static final int INTAKE_CLAMP_FORWARD_CHANNEL = 0;

  /** Lift grabbing pneumatic channels */
  public static final int LIFT_GRABBING_FORWARD_CHANNEL = 1, LIFT_GRABBING_REVERSE_CHANNEL = 2;

  /** Sensor DIO */
  public static final int LIFT_GAME_PIECE_DIO = 0,
      INTAKE_GAME_PIECE_DIO = 1,
      ELEVATOR_ENCODER_ONE_DIO = 2,
      ELEVATOR_ENCODER_TWO_DIO = 3;

/** PWM Port for LEDs */
public static final int LED_PWM_PORT = 0;
}
