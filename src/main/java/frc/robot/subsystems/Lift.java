// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
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

  private final RelativeEncoder elevatorEncoder;
  private final RelativeEncoder armEncoder;

  private SparkMaxPIDController elevatorPID = elevator.getPIDController();

  private CANSparkMax arm = new CANSparkMax(RobotMap.ARM_MOTOR_CAN_ID, MotorType.kBrushless);
  private SparkMaxPIDController armPID = arm.getPIDController();

  private DoubleSolenoid grabber =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          RobotMap.LIFT_GRABBING_FORWARD_CHANNEL,
          RobotMap.LIFT_GRABBING_REVERSE_CHANNEL);

  // Sensors
  private final DigitalInput gamePieceSensor;

  /** Indicates the elevator and arm positions at each position of the lift.
   * The first value indicates the elevator position in inches
   * and the second value indicates the arm position in degrees
   */
  TreeMap<LiftPosition, Pair<Double, Double>> liftPositionMap;

  /** Creates a new Lift */
  public Lift() {
    elevator.restoreFactoryDefaults();
    arm.restoreFactoryDefaults();

    /* Get positions and degrees of elevator through encoder in inches*/
    elevatorEncoder = elevator.getEncoder();
    elevatorEncoder.setPositionConversionFactor(Constants.ELEVATOR_MOTOR_ENCODER_SCALAR);
    elevatorEncoder.setVelocityConversionFactor(Constants.ELEVATOR_MOTOR_ENCODER_VELOCITY_SCALAR);

    /* Get positions and degrees of arm through encoder in degrees*/
    armEncoder = arm.getEncoder();
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

    gamePieceSensor = new DigitalInput(RobotMap.LIFT_GAME_PIECE_DIO);
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
        liftPositionMap.get(pos).getFirst(), CANSparkMax.ControlType.kPosition);
    armPID.setReference(liftPositionMap.get(pos).getSecond(), CANSparkMax.ControlType.kPosition);
  }

  public void grab() {
    grabber.set(DoubleSolenoid.Value.kForward);
  }

  public void drop() {
    grabber.set(DoubleSolenoid.Value.kReverse);
  }

  /** Returns true if the cargo sensor see a cargo */
  public boolean seeCargo() {
    // Sensor is false if there's a ball
    return !gamePieceSensor.get();
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
