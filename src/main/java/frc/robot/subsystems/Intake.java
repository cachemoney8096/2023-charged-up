// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations;
import frc.robot.Constants;
import frc.robot.RobotMap;

/**
 * The game piece intake.
 *
 * <p>Deploys out of the frame perimeter. Collect game pieces (cubes and cones) from the ground and
 * preps them to be grabbed by the lift.
 */
public class Intake extends SubsystemBase {

  // Sensors
  private final DigitalInput gamePieceSensor = new DigitalInput(RobotMap.INTAKE_GAME_PIECE_DIO);
  ;

  private CANSparkMax deployMotor =
      new CANSparkMax(RobotMap.INTAKE_DEPLOY_MOTOR_CAN_ID, MotorType.kBrushless);
  private SparkMaxPIDController deployMotorPID = deployMotor.getPIDController();

  private final RelativeEncoder deployMotorEncoder = deployMotor.getEncoder();
  private Solenoid clamp =
      new Solenoid(PneumaticsModuleType.REVPH, RobotMap.INTAKE_CLAMP_FORWARD_CHANNEL);

  private CANSparkMax intakeLeft =
      new CANSparkMax(RobotMap.INTAKE_LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
  private CANSparkMax intakeRight =
      new CANSparkMax(RobotMap.INTAKE_RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);

  private Timer clampTimer = new Timer();

  /** Creates a new Intake. */
  public Intake() {
    deployMotor.restoreFactoryDefaults();
    deployMotorEncoder.setPositionConversionFactor(Constants.DEPLOY_MOTOR_ENCODER_SCALAR);
    deployMotorEncoder.setVelocityConversionFactor(Constants.DEPLOY_MOTOR_ENCODER_VELOCITY_SCALAR);

    deployMotorPID.setP(Calibrations.INTAKE_DEPLOY_MOTOR_P);
    deployMotorPID.setI(Calibrations.INTAKE_DEPLOY_MOTOR_I);
    deployMotorPID.setD(Calibrations.INTAKE_DEPLOY_MOTOR_D);

    intakeLeft.restoreFactoryDefaults();

    intakeRight.restoreFactoryDefaults();
    intakeRight.follow(intakeLeft, true);
  }

  /** Deploys the intake out */
  public void deploy() {
    deployMotorPID.setReference(
        Calibrations.INTAKE_DEPLOYED_POSITION_DEGREES, CANSparkMax.ControlType.kPosition, Calibrations.SMART_MOTION_SLOT, Calibrations.ARBITRARY_INTAKE_FEED_FORWARD * getCosineIntakeAngle());
    clampTimer.reset();
    clampTimer.start();
  }

  /** Brings the intake back in */
  public void retract() {
    unclampIntake();
    deployMotorPID.setReference(
        Calibrations.INTAKE_STARTING_POSITION_DEGREES, CANSparkMax.ControlType.kPosition, Calibrations.SMART_MOTION_SLOT, Calibrations.ARBITRARY_INTAKE_FEED_FORWARD * getCosineIntakeAngle());
  }

  /** Runs the intake wheels inward */
  public void intakeGamePiece() {
    intakeLeft.set(Calibrations.INTAKE_INTAKING_POWER);
  }

  /** Runs the intake wheels outward */
  public void ejectGamePiece() {
    intakeLeft.set(Calibrations.INTAKE_EJECTION_POWER);
  }

  public void clampIntake() {
    clamp.set(false);
  }

  public void unclampIntake() {
    clamp.set(true);
  }

  /** Returns true if the game piece sensor sees a game piece */
  public boolean seeGamePiece() {
    // Sensor is false if there's a game piece
    return !gamePieceSensor.get();
  }

   /**
   * Returns the cosine of the intake angle in degrees off of the horizontal.
   */
  public double getCosineIntakeAngle() {
    return Math.cos(deployMotorEncoder.getPosition() + 180);
  }

  @Override
  public void periodic() {
    if (clampTimer.hasElapsed(Calibrations.AUTO_CLAMP_WAIT_TIME_SECONDS)) {
      clampIntake();
      clampTimer.stop();
      clampTimer.reset();
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
