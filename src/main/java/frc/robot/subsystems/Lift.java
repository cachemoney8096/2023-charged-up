// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations;
import frc.robot.Constants;
import frc.robot.RobotMap;
import java.util.TreeMap;

/** Contains code for elevator, arm, and game piece grabber */
public class Lift extends SubsystemBase {
  private CANSparkMax elevator =
      new CANSparkMax(RobotMap.ELEVATOR_MOTOR_CAN_ID, MotorType.kBrushless);

  private final RelativeEncoder elevatorEncoder = elevator.getEncoder();

  private SparkMaxPIDController elevatorPID = elevator.getPIDController();

  private CANSparkMax arm = new CANSparkMax(RobotMap.ARM_MOTOR_CAN_ID, MotorType.kBrushless);

  private final RelativeEncoder armEncoder = arm.getEncoder();

  private SparkMaxPIDController armPID = arm.getPIDController();

  private final int SMART_MOTION_SLOT = 0;

  private DoubleSolenoid grabber =
      new DoubleSolenoid(
          PneumaticsModuleType.REVPH,
          RobotMap.LIFT_GRABBING_FORWARD_CHANNEL,
          RobotMap.LIFT_GRABBING_REVERSE_CHANNEL);

  // Sensors
  private final DigitalInput gamePieceSensor = new DigitalInput(RobotMap.LIFT_GAME_PIECE_DIO);

  /**
   * Indicates the elevator and arm positions at each position of the lift. The first value
   * indicates the elevator position in inches and the second value indicates the arm position in
   * degrees
   */
  TreeMap<LiftPosition, Pair<Double, Double>> liftPositionMap;

  /** Creates a new Lift */
  public Lift() {
    elevator.restoreFactoryDefaults();
    arm.restoreFactoryDefaults();

    /* Get positions and degrees of elevator through encoder in inches */
    elevatorEncoder.setPositionConversionFactor(Constants.ELEVATOR_MOTOR_ENCODER_SCALAR);
    elevatorEncoder.setVelocityConversionFactor(Constants.ELEVATOR_MOTOR_ENCODER_VELOCITY_SCALAR);

    /* Get positions and degrees of arm through encoder in degrees */
    armEncoder.setPositionConversionFactor(Constants.ARM_MOTOR_ENCODER_SCALAR);
    armEncoder.setVelocityConversionFactor(Constants.ARM_MOTOR_ENCODER_VELOCITY_SCALAR);

    /* Set PID of Elevator */
    elevatorPID.setP(Calibrations.ELEVATOR_P);
    elevatorPID.setI(Calibrations.ELEVATOR_I);
    elevatorPID.setD(Calibrations.ELEVATOR_D);

    /* Set PID of Arm */
    armPID.setP(Calibrations.ARM_P);
    armPID.setI(Calibrations.ARM_I);
    armPID.setD(Calibrations.ARM_D);

    armPID.setSmartMotionMaxAccel(Calibrations.ARM_MAX_ACCELERATION_RPM, SMART_MOTION_SLOT);
    armPID.setSmartMotionMaxVelocity(Calibrations.ARM_MAX_VELOCITY_RPM, SMART_MOTION_SLOT);
    armPID.setSmartMotionMinOutputVelocity(
        Calibrations.ARM_MIN_OUTPUT_VELOCITY_RPM, SMART_MOTION_SLOT);
    armPID.setSmartMotionAllowedClosedLoopError(
        Calibrations.ARM_ALLOWED_CLOSED_LOOP_ERROR, SMART_MOTION_SLOT);

    /* Map of all LiftPosition with according values */

    liftPositionMap = new TreeMap<LiftPosition, Pair<Double, Double>>();
    liftPositionMap.put(
        LiftPosition.GRAB_FROM_INTAKE,
        new Pair<Double, Double>(Calibrations.PLACEHOLDER_DOUBLE, Calibrations.PLACEHOLDER_DOUBLE));
    liftPositionMap.put(
        LiftPosition.SHELF,
        new Pair<Double, Double>(Calibrations.PLACEHOLDER_DOUBLE, Calibrations.PLACEHOLDER_DOUBLE));
    liftPositionMap.put(
        LiftPosition.SCORE_MID,
        new Pair<Double, Double>(Calibrations.PLACEHOLDER_DOUBLE, Calibrations.PLACEHOLDER_DOUBLE));
    liftPositionMap.put(
        LiftPosition.SCORE_HIGH,
        new Pair<Double, Double>(Calibrations.PLACEHOLDER_DOUBLE, Calibrations.PLACEHOLDER_DOUBLE));
    liftPositionMap.put(
        LiftPosition.STARTING,
        new Pair<Double, Double>(Calibrations.PLACEHOLDER_DOUBLE, Calibrations.PLACEHOLDER_DOUBLE));
  }

  public enum LiftPosition {
    GRAB_FROM_INTAKE,
    SHELF,
    SCORE_MID,
    SCORE_HIGH,
    STARTING
  }

  public void goToPosition(LiftPosition pos) {
    elevatorPID.setReference(
        liftPositionMap.get(pos).getFirst(),
        CANSparkMax.ControlType.kSmartMotion,
        SMART_MOTION_SLOT,
        Calibrations.ARBITRARY_ARM_FEED_FORWARD_VOLTS * getCosineArmAngle(),
        ArbFFUnits.kVoltage);
    armPID.setReference(
        liftPositionMap.get(pos).getSecond(),
        CANSparkMax.ControlType.kSmartMotion,
        SMART_MOTION_SLOT,
        Calibrations.ARBITRARY_ARM_FEED_FORWARD_VOLTS * getCosineArmAngle(),
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
    return Math.cos(armEncoder.getPosition() - Constants.ARM_POSITION_WHEN_HORIZONTAL_DEGREES);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
