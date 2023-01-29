// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.math.Pair;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Cal;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.utils.SparkMaxUtils;
import java.util.TreeMap;

/** Contains code for elevator, arm, and game piece grabber */
public class Lift extends SubsystemBase {

  /** Overall position of the lift including both elevator and arm */
  public enum LiftPosition {
    GRAB_FROM_INTAKE,
    SHELF,
    SCORE_MID,
    SCORE_HIGH,
    STARTING
  }

  // Actuators
  private CANSparkMax elevator =
      new CANSparkMax(RobotMap.ELEVATOR_MOTOR_CAN_ID, MotorType.kBrushless);
  private SparkMaxPIDController elevatorPID = elevator.getPIDController();
  private CANSparkMax arm = new CANSparkMax(RobotMap.ARM_MOTOR_CAN_ID, MotorType.kBrushless);
  private SparkMaxPIDController armPID = arm.getPIDController();
  private DoubleSolenoid grabber =
      new DoubleSolenoid(
          PneumaticsModuleType.REVPH,
          RobotMap.LIFT_GRABBING_FORWARD_CHANNEL,
          RobotMap.LIFT_GRABBING_REVERSE_CHANNEL);

  // Sensors
  private final RelativeEncoder elevatorEncoder = elevator.getEncoder();
  /* Returns [0,1] in revolutions */
  private final DutyCycleEncoder elevatorDutyCycleEncoderOne =
      new DutyCycleEncoder(RobotMap.ELEVATOR_ENCODER_ONE_DIO);
  /* Returns [0,1] in revolutions */
  private final DutyCycleEncoder elevatorDutyCycleEncoderTwo =
      new DutyCycleEncoder(RobotMap.ELEVATOR_ENCODER_TWO_DIO);
  private final RelativeEncoder armEncoder = arm.getEncoder();
  private final AbsoluteEncoder armAbsoluteEncoder = arm.getAbsoluteEncoder(Type.kDutyCycle);
  private final DigitalInput gamePieceSensor = new DigitalInput(RobotMap.LIFT_GAME_PIECE_DIO);

  // Members
  private final int SMART_MOTION_SLOT = 0;

  /**
   * Indicates the elevator and arm positions at each position of the lift. The first value
   * indicates the elevator position in inches and the second value indicates the arm position in
   * degrees
   */
  TreeMap<LiftPosition, Pair<Double, Double>> liftPositionMap;

  /** Creates a new Lift */
  public Lift() {
    SparkMaxUtils.initWithRetry(this::initSparks, Cal.SPARK_INIT_RETRY_ATTEMPTS);

    // Map of all LiftPosition with according values
    liftPositionMap = new TreeMap<LiftPosition, Pair<Double, Double>>();
    liftPositionMap.put(
        LiftPosition.GRAB_FROM_INTAKE,
        new Pair<Double, Double>(Cal.PLACEHOLDER_DOUBLE, Cal.PLACEHOLDER_DOUBLE));
    liftPositionMap.put(
        LiftPosition.SHELF,
        new Pair<Double, Double>(Cal.PLACEHOLDER_DOUBLE, Cal.PLACEHOLDER_DOUBLE));
    liftPositionMap.put(
        LiftPosition.SCORE_MID,
        new Pair<Double, Double>(Cal.PLACEHOLDER_DOUBLE, Cal.PLACEHOLDER_DOUBLE));
    liftPositionMap.put(
        LiftPosition.SCORE_HIGH,
        new Pair<Double, Double>(Cal.PLACEHOLDER_DOUBLE, Cal.PLACEHOLDER_DOUBLE));
    liftPositionMap.put(
        LiftPosition.STARTING,
        new Pair<Double, Double>(Cal.PLACEHOLDER_DOUBLE, Cal.PLACEHOLDER_DOUBLE));
  }

  /** Does all the initialization for the sparks, return true on success */
  boolean initSparks() {
    int errors = 0;

    errors += SparkMaxUtils.check(elevator.restoreFactoryDefaults());
    errors += SparkMaxUtils.check(arm.restoreFactoryDefaults());

    // Get positions and degrees of elevator through encoder in inches
    errors +=
        SparkMaxUtils.check(
            elevatorEncoder.setPositionConversionFactor(
                Constants.Lift.ELEVATOR_MOTOR_ENCODER_SCALAR));
    errors +=
        SparkMaxUtils.check(
            elevatorEncoder.setVelocityConversionFactor(
                Constants.Lift.ELEVATOR_MOTOR_ENCODER_VELOCITY_SCALAR));

    // Set PID of Elevator
    errors += SparkMaxUtils.check(elevatorPID.setP(Cal.Lift.ELEVATOR_P));
    errors += SparkMaxUtils.check(elevatorPID.setI(Cal.Lift.ELEVATOR_I));
    errors += SparkMaxUtils.check(elevatorPID.setD(Cal.Lift.ELEVATOR_D));
    errors +=
        SparkMaxUtils.check(
            elevatorPID.setSmartMotionMaxAccel(
                Cal.Lift.ELEVATOR_MAX_ACCELERATION_IN_PER_SECOND_SQUARED, SMART_MOTION_SLOT));
    errors +=
        SparkMaxUtils.check(
            elevatorPID.setSmartMotionMaxVelocity(
                Cal.Lift.ELEVATOR_MAX_VELOCITY_IN_PER_SECOND, SMART_MOTION_SLOT));
    errors +=
        SparkMaxUtils.check(
            elevatorPID.setSmartMotionMinOutputVelocity(
                Cal.Lift.ELEVATOR_MIN_OUTPUT_VELOCITY_IN_PER_SECOND, SMART_MOTION_SLOT));
    errors +=
        SparkMaxUtils.check(
            elevatorPID.setSmartMotionAllowedClosedLoopError(
                Cal.Lift.ELEVATOR_ALLOWED_CLOSED_LOOP_ERROR_IN, SMART_MOTION_SLOT));

    // Set PID of Arm
    errors += SparkMaxUtils.check(armPID.setP(Cal.Lift.ARM_P));
    errors += SparkMaxUtils.check(armPID.setI(Cal.Lift.ARM_I));
    errors += SparkMaxUtils.check(armPID.setD(Cal.Lift.ARM_D));
    errors +=
        SparkMaxUtils.check(
            armPID.setSmartMotionMaxAccel(
                Cal.Lift.ARM_MAX_ACCELERATION_DEG_PER_SECOND_SQUARED, SMART_MOTION_SLOT));
    errors +=
        SparkMaxUtils.check(
            armPID.setSmartMotionMaxVelocity(
                Cal.Lift.ARM_MAX_VELOCITY_DEG_PER_SECOND, SMART_MOTION_SLOT));
    errors +=
        SparkMaxUtils.check(
            armPID.setSmartMotionMinOutputVelocity(
                Cal.Lift.ARM_MIN_OUTPUT_VELOCITY_DEG_PER_SECOND, SMART_MOTION_SLOT));
    errors +=
        SparkMaxUtils.check(
            armPID.setSmartMotionAllowedClosedLoopError(
                Cal.Lift.ARM_ALLOWED_CLOSED_LOOP_ERROR_DEG, SMART_MOTION_SLOT));

    // Get positions and degrees of arm through encoder in degrees
    errors +=
        SparkMaxUtils.check(
            armEncoder.setPositionConversionFactor(Constants.Lift.ARM_MOTOR_ENCODER_SCALAR));
    errors +=
        SparkMaxUtils.check(
            armEncoder.setVelocityConversionFactor(
                Constants.Lift.ARM_MOTOR_ENCODER_VELOCITY_SCALAR));
    errors +=
        SparkMaxUtils.check(
            armAbsoluteEncoder.setPositionConversionFactor(Constants.REVOLUTIONS_TO_DEGREES));

    return errors == 0;
  }

  public void goToPosition(LiftPosition pos) {
    elevatorPID.setReference(
        liftPositionMap.get(pos).getFirst(),
        CANSparkMax.ControlType.kSmartMotion,
        SMART_MOTION_SLOT,
        Cal.Lift.ARBITRARY_ELEVATOR_FEED_FORWARD_VOLTS,
        ArbFFUnits.kVoltage);
    armPID.setReference(
        liftPositionMap.get(pos).getSecond(),
        CANSparkMax.ControlType.kSmartMotion,
        SMART_MOTION_SLOT,
        Cal.Lift.ARBITRARY_ARM_FEED_FORWARD_VOLTS * getCosineArmAngle(),
        ArbFFUnits.kVoltage);
  }

  public void grab() {
    grabber.set(DoubleSolenoid.Value.kForward);
  }

  public void drop() {
    grabber.set(DoubleSolenoid.Value.kReverse);
  }

  /** Returns true if the game piece sensor sees a game piece */
  public boolean seeGamePiece() {
    // Sensor is false if there's a game piece
    return !gamePieceSensor.get();
  }

  /**
   * Scores the currently loaded game piece Currently empty because we don't know exactly what it
   * will do
   */
  public void scoreGamePiece() {}

  /**
   * Says whether or not the robot is done scoring a game piece Currently empty because we don't
   * know exactly what it will look like
   */
  public boolean doneScoring() {
    return true; // Placeholder until logic is made
  }

  /** Returns the cosine of the arm angle in degrees off of the horizontal. */
  public double getCosineArmAngle() {
    return Math.cos(armEncoder.getPosition() - Constants.Lift.ARM_POSITION_WHEN_HORIZONTAL_DEGREES);
  }

  public void initialize() {
    // Set arm encoder position from absolute
    armEncoder.setPosition(
        armAbsoluteEncoder.getPosition() + Cal.Lift.ARM_ABSOLUTE_ENCODER_OFFSET_DEG);

    // Set elevator encoder position from absolute encoders
    double elevatorDutyCycleEncodersDifferenceDegrees =
        ((elevatorDutyCycleEncoderOne.getAbsolutePosition()
                    - elevatorDutyCycleEncoderTwo.getAbsolutePosition())
                * Constants.REVOLUTIONS_TO_DEGREES)
            % 360;
    if (elevatorDutyCycleEncodersDifferenceDegrees < 0.0) {
      elevatorDutyCycleEncodersDifferenceDegrees += Constants.REVOLUTIONS_TO_DEGREES;
    }
    elevatorEncoder.setPosition(
        elevatorDutyCycleEncodersDifferenceDegrees
            * Constants.Lift.ELEVATOR_MOTOR_ENCODER_DIFFERENCES_SCALAR_INCHES_PER_DEGREE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Elevator kP", elevatorPID::getP, elevatorPID::setP);
    builder.addDoubleProperty("Elevator kI", elevatorPID::getI, elevatorPID::setI);
    builder.addDoubleProperty("Elevator kD", elevatorPID::getD, elevatorPID::setD);
    builder.addDoubleProperty(
        "Elevator Position", elevatorEncoder::getPosition, elevatorEncoder::setPosition);
    builder.addDoubleProperty("Arm kP", armPID::getP, armPID::setP);
    builder.addDoubleProperty("Arm kI", armPID::getI, armPID::setI);
    builder.addDoubleProperty("Arm kD", armPID::getD, armPID::setD);
    builder.addDoubleProperty("Arm Position", armEncoder::getPosition, armEncoder::setPosition);
    builder.addBooleanProperty("Done Scoring", this::doneScoring, null);
    builder.addBooleanProperty("See Game Piece", this::seeGamePiece, null);
  }
}
