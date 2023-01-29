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
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.utils.SparkMaxUtils;
import java.util.Optional;

/**
 * The game piece intake.
 *
 * <p>Deploys out of the frame perimeter. Collect game pieces (cubes and cones) from the ground and
 * preps them to be grabbed by the lift.
 */
public class Intake extends SubsystemBase {
  // Actuators
  private CANSparkMax deployMotor =
      new CANSparkMax(RobotMap.INTAKE_DEPLOY_MOTOR_CAN_ID, MotorType.kBrushless);
  private SparkMaxPIDController deployMotorPID = deployMotor.getPIDController();
  private Solenoid clamp =
      new Solenoid(PneumaticsModuleType.REVPH, RobotMap.INTAKE_CLAMP_FORWARD_CHANNEL);
  private CANSparkMax intakeLeft =
      new CANSparkMax(RobotMap.INTAKE_LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
  private CANSparkMax intakeRight =
      new CANSparkMax(RobotMap.INTAKE_RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);

  // Sensors
  private final DigitalInput gamePieceSensor = new DigitalInput(RobotMap.INTAKE_GAME_PIECE_DIO);
  private final RelativeEncoder deployMotorEncoder = deployMotor.getEncoder();
  private final AbsoluteEncoder deployMotorAbsoluteEncoder =
      deployMotor.getAbsoluteEncoder(Type.kDutyCycle);

  // Members
  /**
   * Timer for clamping. Not present means retracted or retracting (clamp!). Exists means deploying
   * or already deployed (check timer for whether to clamp). Elapsed means we should clamp or stay
   * clamped.
   */
  private Optional<Timer> clampTimer = Optional.empty();

  private double intakeDesiredPositionDegrees = Calibrations.Intake.STARTING_POSITION_DEGREES;

  private final int SMART_MOTION_SLOT = 0;

  /** Creates a new Intake. */
  public Intake() {
    SparkMaxUtils.initWithRetry(this::setUpDeploySpark, Calibrations.SPARK_INIT_RETRY_ATTEMPTS);
    SparkMaxUtils.initWithRetry(
        this::setUpIntakeWheelSparks, Calibrations.SPARK_INIT_RETRY_ATTEMPTS);
  }

  boolean setUpIntakeWheelSparks() {
    int errors = 0;
    errors += SparkMaxUtils.check(intakeLeft.restoreFactoryDefaults());

    errors += SparkMaxUtils.check(intakeRight.restoreFactoryDefaults());
    errors += SparkMaxUtils.check(intakeRight.follow(intakeLeft, true));
    return errors == 0;
  }

  boolean setUpDeploySpark() {
    int errors = 0;

    errors += SparkMaxUtils.check(deployMotor.restoreFactoryDefaults());
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setDegreesFromGearRatio(
                deployMotorEncoder, Constants.Intake.DEPLOY_MOTOR_GEAR_RATIO));
    errors +=
        SparkMaxUtils.check(
            deployMotorAbsoluteEncoder.setPositionConversionFactor(
                Constants.REVOLUTIONS_TO_DEGREES));
    errors += SparkMaxUtils.check(deployMotorPID.setP(Calibrations.Intake.DEPLOY_MOTOR_P));
    errors += SparkMaxUtils.check(deployMotorPID.setI(Calibrations.Intake.DEPLOY_MOTOR_I));
    errors += SparkMaxUtils.check(deployMotorPID.setD(Calibrations.Intake.DEPLOY_MOTOR_D));

    errors +=
        SparkMaxUtils.check(
            deployMotorPID.setSmartMotionMaxAccel(
                Calibrations.Intake.DEPLOY_MAX_ACCELERATION_DEG_PER_SECOND_SQUARED,
                SMART_MOTION_SLOT));
    errors +=
        SparkMaxUtils.check(
            deployMotorPID.setSmartMotionMaxVelocity(
                Calibrations.Intake.DEPLOY_MAX_VELOCITY_DEG_PER_SECOND, SMART_MOTION_SLOT));
    errors +=
        SparkMaxUtils.check(
            deployMotorPID.setSmartMotionMinOutputVelocity(
                Calibrations.Intake.DEPLOY_MIN_OUTPUT_VELOCITY_DEG_PER_SECOND, SMART_MOTION_SLOT));
    errors +=
        SparkMaxUtils.check(
            deployMotorPID.setSmartMotionAllowedClosedLoopError(
                Calibrations.Intake.DEPLOY_ALLOWED_CLOSED_LOOP_ERROR_DEG, SMART_MOTION_SLOT));

    return errors == 0;
  }

  /** Deploys the intake out. */
  public void deploy() {
    // Set the desired intake position
    deployMotorPID.setReference(
        Calibrations.Intake.DEPLOYED_POSITION_DEGREES,
        CANSparkMax.ControlType.kSmartMotion,
        SMART_MOTION_SLOT,
        Calibrations.Intake.ARBITRARY_FEED_FORWARD_VOLTS * getCosineIntakeAngle(),
        ArbFFUnits.kVoltage);
    intakeDesiredPositionDegrees = Calibrations.Intake.DEPLOYED_POSITION_DEGREES;

    // Set a timer only if we're newly deploying
    // If we have already set a timer, we shouldn't restart the timer
    if (!clampTimer.isPresent()) {
      clampTimer = Optional.of(new Timer());
      clampTimer.get().start();
    }
  }

  /** Brings the intake back in */
  public void retract() {
    // Kill the timer to indicate retraction for clamping
    clampTimer = Optional.empty();

    // Set the desired intake position
    deployMotorPID.setReference(
        Calibrations.Intake.STARTING_POSITION_DEGREES,
        CANSparkMax.ControlType.kSmartMotion,
        SMART_MOTION_SLOT,
        Calibrations.Intake.ARBITRARY_FEED_FORWARD_VOLTS * getCosineIntakeAngle(),
        ArbFFUnits.kVoltage);
    intakeDesiredPositionDegrees = Calibrations.Intake.STARTING_POSITION_DEGREES;
  }

  /** Runs the intake wheels inward */
  public void intakeGamePiece() {
    intakeLeft.set(Calibrations.Intake.INTAKING_POWER);
  }

  /** Runs the intake wheels outward */
  public void ejectGamePiece() {
    intakeLeft.set(Calibrations.Intake.EJECTION_POWER);
  }

  public void stopIntakingGamePiece() {
    intakeLeft.set(0.0);
  }

  private void clampIntake() {
    clamp.set(false);
  }

  private void unclampIntake() {
    clamp.set(true);
  }

  /** Returns true if the game piece sensor sees a game piece */
  public boolean seeGamePiece() {
    // Sensor is false if there's a game piece
    return !gamePieceSensor.get();
  }

  public void initialize() {
    deployMotorEncoder.setPosition(
        deployMotorAbsoluteEncoder.getPosition() + Calibrations.Intake.ABSOLUTE_ENCODER_OFFSET_DEG);
  }

  /** Returns the cosine of the intake angle in degrees off of the horizontal. */
  public double getCosineIntakeAngle() {
    return Math.cos(
        deployMotorEncoder.getPosition() - Constants.Intake.POSITION_WHEN_HORIZONTAL_DEGREES);
  }

  @Override
  public void periodic() {
    if (clampTimer.isPresent()) {
      // If present, that means we're deployed or in the process of deploying
      if (clampTimer.get().hasElapsed(Calibrations.Intake.AUTO_CLAMP_WAIT_TIME_SECONDS)) {
        // If the time has elapsed, that means we've deployed enough to clamp
        clampIntake();
      } else {
        // If time has not elapsed, we are still in the process of deploying and shouldn't clamp yet
        unclampIntake();
      }
    } else {
      // When there's no timer, we're retracting or retracted so we should unclamp
      unclampIntake();
    }

    // If the intake has achieved its desired position, then cut power
    if (Math.abs(intakeDesiredPositionDegrees - deployMotorEncoder.getPosition())
        < Calibrations.Intake.POSITION_THRESHOLD_DEGREES) {
      deployMotorPID.setReference(0.0, CANSparkMax.ControlType.kVoltage);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Intake Deploy kP", deployMotorPID::getP, deployMotorPID::setP);
    builder.addDoubleProperty("Intake Deploy kI", deployMotorPID::getI, deployMotorPID::setI);
    builder.addDoubleProperty("Intake Deploy kD", deployMotorPID::getD, deployMotorPID::setD);
    builder.addDoubleProperty(
        "Intake Desired Position (Degrees)", () -> intakeDesiredPositionDegrees, null);
    builder.addDoubleProperty(
        "Intake Current Position (Degrees)",
        deployMotorEncoder::getPosition,
        deployMotorEncoder::setPosition);
    builder.addBooleanProperty("See Game Piece", this::seeGamePiece, null);
  }
}
