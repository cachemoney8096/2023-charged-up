// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
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
import frc.robot.Cal;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.utils.AngleUtil;
import frc.robot.utils.SparkMaxUtils;
import java.util.Optional;
import java.util.function.BooleanSupplier;

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
  private final int SMART_MOTION_SLOT = 0;
  private double intakeDesiredPositionDegrees = Cal.Intake.STARTING_POSITION_DEGREES;
  private Optional<Boolean> desiredDeployed = Optional.empty();
  private boolean desireClamped = false;
  private BooleanSupplier clearOfIntake;

  /** Creates a new Intake. */
  public Intake(BooleanSupplier clearOfIntakeZone) {
    SparkMaxUtils.initWithRetry(this::setUpDeploySpark, Cal.SPARK_INIT_RETRY_ATTEMPTS);
    SparkMaxUtils.initWithRetry(this::setUpIntakeWheelSparks, Cal.SPARK_INIT_RETRY_ATTEMPTS);
    clearOfIntake = clearOfIntakeZone;
  }

  /** Does all the initialization for the sparks, return true on success */
  private boolean setUpIntakeWheelSparks() {
    int errors = 0;
    errors += SparkMaxUtils.check(intakeLeft.restoreFactoryDefaults());

    errors += SparkMaxUtils.check(intakeRight.restoreFactoryDefaults());
    errors += SparkMaxUtils.check(intakeRight.follow(intakeLeft, true));
    return errors == 0;
  }

  /** Does all the initialization for the spark, return true on success */
  private boolean setUpDeploySpark() {
    int errors = 0;

    errors += SparkMaxUtils.check(deployMotor.restoreFactoryDefaults());
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setDegreesFromGearRatio(
                deployMotorEncoder, Constants.Intake.DEPLOY_MOTOR_GEAR_RATIO));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setDegreesFromGearRatio(deployMotorAbsoluteEncoder, 1.0));
    errors += SparkMaxUtils.check(deployMotorPID.setP(Cal.Intake.DEPLOY_MOTOR_P));
    errors += SparkMaxUtils.check(deployMotorPID.setI(Cal.Intake.DEPLOY_MOTOR_I));
    errors += SparkMaxUtils.check(deployMotorPID.setD(Cal.Intake.DEPLOY_MOTOR_D));

    errors +=
        SparkMaxUtils.check(
            deployMotorPID.setSmartMotionMaxAccel(
                Cal.Intake.DEPLOY_MAX_ACCELERATION_DEG_PER_SECOND_SQUARED, SMART_MOTION_SLOT));
    errors +=
        SparkMaxUtils.check(
            deployMotorPID.setSmartMotionMaxVelocity(
                Cal.Intake.DEPLOY_MAX_VELOCITY_DEG_PER_SECOND, SMART_MOTION_SLOT));
    errors +=
        SparkMaxUtils.check(
            deployMotorPID.setSmartMotionMinOutputVelocity(
                Cal.Intake.DEPLOY_MIN_OUTPUT_VELOCITY_DEG_PER_SECOND, SMART_MOTION_SLOT));
    errors +=
        SparkMaxUtils.check(
            deployMotorPID.setSmartMotionAllowedClosedLoopError(
                Cal.Intake.DEPLOY_ALLOWED_CLOSED_LOOP_ERROR_DEG, SMART_MOTION_SLOT));

    errors +=
        SparkMaxUtils.check(
            deployMotor.setSoftLimit(
                SoftLimitDirection.kForward,
                Cal.Intake.INTAKE_DEPLOY_MOTOR_POSITIVE_LIMIT_DEGREES));
    errors += SparkMaxUtils.check(deployMotor.enableSoftLimit(SoftLimitDirection.kForward, true));

    errors +=
        SparkMaxUtils.check(
            deployMotor.setSoftLimit(
                SoftLimitDirection.kReverse,
                Cal.Intake.INTAKE_DEPLOY_MOTOR_NEGATIVE_LIMIT_DEGREES));
    errors += SparkMaxUtils.check(deployMotor.enableSoftLimit(SoftLimitDirection.kReverse, true));

    return errors == 0;
  }

  public void initialize() {
    deployMotorEncoder.setPosition(
        AngleUtil.wrapAngle(
            deployMotorAbsoluteEncoder.getPosition()
                - (Cal.Intake.ABSOLUTE_ENCODER_START_POS_DEG
                    - Cal.Intake.STARTING_POSITION_DEGREES)));
  }

  /**
   * Burns the current settings to sparks so they keep current settings on reboot. Should be done
   * after all settings are set.
   */
  public void burnFlashSparks() {
    Timer.delay(0.005);
    deployMotor.burnFlash();
    Timer.delay(0.005);
    intakeLeft.burnFlash();
    Timer.delay(0.005);
    intakeRight.burnFlash();
  }

  /** Deploys the intake out. */
  private void deploy() {
    desireClamped = true;

    // Set the desired intake position
    deployMotorPID.setReference(
        Cal.Intake.DEPLOYED_POSITION_DEGREES,
        CANSparkMax.ControlType.kSmartMotion,
        SMART_MOTION_SLOT,
        Cal.Intake.ARBITRARY_FEED_FORWARD_VOLTS * getCosineIntakeAngle(),
        ArbFFUnits.kVoltage);
    intakeDesiredPositionDegrees = Cal.Intake.DEPLOYED_POSITION_DEGREES;
  }

  /** Brings the intake back in */
  private void retract() {
    desireClamped = false;

    // Set the desired intake position
    deployMotorPID.setReference(
        Cal.Intake.STARTING_POSITION_DEGREES,
        CANSparkMax.ControlType.kSmartMotion,
        SMART_MOTION_SLOT,
        Cal.Intake.ARBITRARY_FEED_FORWARD_VOLTS * getCosineIntakeAngle(),
        ArbFFUnits.kVoltage);
    intakeDesiredPositionDegrees = Cal.Intake.STARTING_POSITION_DEGREES;
  }

  /*Setter for whether intake is desired deploy is true retract is false */
  public void setDesiredDeployed(boolean desired) {
    desiredDeployed = Optional.of(desired);
  }

  /** Returns the cosine of the intake angle in degrees off of the horizontal. */
  private double getCosineIntakeAngle() {
    return Math.cos(
        deployMotorEncoder.getPosition() - Constants.Intake.POSITION_WHEN_HORIZONTAL_DEGREES);
  }

  /** If the intake has achieved its desired position, return true */
  public boolean atDesiredPosition() {
    return (Math.abs(intakeDesiredPositionDegrees - deployMotorEncoder.getPosition())
        < Cal.Intake.POSITION_MARGIN_DEGREES);
  }

  public void setDesiredClamped(boolean clamp) {
    desireClamped = clamp;
  }

  private void clampIntake() {
    clamp.set(false);
  }

  private void unclampIntake() {
    clamp.set(true);
  }

  /** Runs the intake wheels inward */
  public void intakeGamePiece() {
    intakeLeft.set(Cal.Intake.INTAKING_POWER);
  }

  /** Runs the intake wheels outward */
  public void ejectGamePiece() {
    intakeLeft.set(Cal.Intake.EJECTION_POWER);
  }

  public void stopIntakingGamePiece() {
    intakeLeft.set(0.0);
  }

  /** Returns true if the game piece sensor sees a game piece */
  public boolean seeGamePiece() {
    // Sensor is false if there's a game piece
    return !gamePieceSensor.get();
  }

  @Override
  public void periodic() {
    if (clearOfIntake.getAsBoolean()) {
      if (desiredDeployed.isPresent()) {
        if (desiredDeployed.get() == true) {
          deploy();
        } else {
          retract();
        }
      }
    }

    // If the intake has achieved its desired position, then cut power
    if (atDesiredPosition()) {
      deployMotorPID.setReference(0.0, CANSparkMax.ControlType.kVoltage);
    }
    // only clamp if it is safe to do so and clamping is desired
    if (desireClamped
        && deployMotorEncoder.getPosition() > Cal.Intake.CLAMP_POSITION_THRESHOLD_DEGREES) {
      clampIntake();
    } else {
      unclampIntake();
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
