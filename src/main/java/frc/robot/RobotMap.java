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
  
  public static final int PIGEON_CAN_ID = 1;
}
