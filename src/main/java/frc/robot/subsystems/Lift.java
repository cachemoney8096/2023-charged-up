// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations;
import frc.robot.Constants;
import frc.robot.RobotMap;

import java.util.TreeMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Lift extends SubsystemBase {
  private CANSparkMax elevator = new CANSparkMax(RobotMap.LIFT_MOTOR_CAN_ID, MotorType.kBrushless);

  private final RelativeEncoder elevatorEncoder;
  private final RelativeEncoder armEncoder;

  private SparkMaxPIDController elevatorPID =  elevator.getPIDController();

  private CANSparkMax arm = new CANSparkMax(RobotMap.ARM_MOTOR_CAN_ID, MotorType.kBrushless);
  private SparkMaxPIDController armPID =  arm.getPIDController();

  TreeMap<LiftPositions, Pair<Double, Double>> liftPositionsMap;


  /** Creates a new Lift */
  public Lift() {
    elevator.restoreFactoryDefaults();
    arm.restoreFactoryDefaults();

    /* Get positions and degrees of elevator through encoder*/
    elevatorEncoder = elevator.getEncoder();
    elevatorEncoder.setPositionConversionFactor(Constants.ELEVATOR_MOTOR_ENCODER_SCALAR);
    elevatorEncoder.setVelocityConversionFactor(Constants.ELEVATOR_MOTOR_ENCODER_VELOCITY_SCALAR);
    /* Get positions and degrees of arm through encoder*/
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

    /* Map of all LiftPositions with according values */

    liftPositionsMap = new TreeMap<LiftPositions, Pair<Double, Double>>();
    liftPositionsMap.put(LiftPositions.GRAB_FROM_INTAKE, new Pair<Double, Double>(Calibrations.PLACEHOLDER_DOUBLE, Calibrations.PLACEHOLDER_DOUBLE));
    liftPositionsMap.put(LiftPositions.SHELF, new Pair<Double, Double>(Calibrations.PLACEHOLDER_DOUBLE, Calibrations.PLACEHOLDER_DOUBLE));
    liftPositionsMap.put(LiftPositions.SCORE_MID, new Pair<Double, Double>(Calibrations.PLACEHOLDER_DOUBLE, Calibrations.PLACEHOLDER_DOUBLE));
    liftPositionsMap.put(LiftPositions.SCORE_HIGH, new Pair<Double, Double>(Calibrations.PLACEHOLDER_DOUBLE, Calibrations.PLACEHOLDER_DOUBLE));
    liftPositionsMap.put(LiftPositions.STOWED, new Pair<Double, Double>(Calibrations.PLACEHOLDER_DOUBLE, Calibrations.PLACEHOLDER_DOUBLE));
  }


  public enum LiftPositions {
    GRAB_FROM_INTAKE,
    SHELF,
    SCORE_MID,
    SCORE_HIGH,
    STOWED
  }

  public void goToPosition(LiftPositions pos) {
    elevatorPID.setReference(liftPositionsMap.get(pos).getFirst(), CANSparkMax.ControlType.kPosition);
    armPID.setReference(liftPositionsMap.get(pos).getSecond(), CANSparkMax.ControlType.kPosition);
  }

  /** Pushes the lift up */
  public void deployElevator() {
    elevatorPID.setReference(Calibrations.ELEVATOR_DEPLOYING_POWER, CANSparkMax.ControlType.kVelocity);
  }

  /** Pushes the lift down */
  public void retractElevator() {
    elevatorPID.setReference(Calibrations.ELEVATOR_RETRACTING_POWER, CANSparkMax.ControlType.kVelocity);
  }

  /** Unfolds the arm */
  public void unfoldArm() {
    armPID.setReference(Calibrations.ARM_UNFOLDING_POWER, CANSparkMax.ControlType.kVelocity);
  }

  /** Folds the arm */
  public void foldArm() {
    armPID.setReference(Calibrations.ARM_FOLDING_POWER, CANSparkMax.ControlType.kVelocity);
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