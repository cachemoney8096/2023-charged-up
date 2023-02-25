// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Cal;
import frc.robot.Constants;
import frc.robot.utils.SparkMaxUtils;

public class SwerveModule implements Sendable {
  private final CANSparkMax drivingSparkMax;
  public final CANSparkMax turningSparkMax;

  private final RelativeEncoder drivingEncoder;
  public final AbsoluteEncoder turningEncoder;

  private final SparkMaxPIDController drivingPIDController;
  private final SparkMaxPIDController turningPIDController;

  private double chassisAngularOffsetRadians = 0;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public double desiredVelMps = 0.0;
    public double desiredPosRad = 0.0;

  /**
   * Constructs a SwerveModule and configures the driving and turning motor, encoder, and PID
   * controller. This configuration is specific to the REV MAXSwerve Module built with NEOs, SPARKS
   * MAX, and a Through Bore Encoder.
   */
  public SwerveModule(int drivingCanId, int turningCanId, double chassisAngularOffset) {
    drivingSparkMax = new CANSparkMax(drivingCanId, MotorType.kBrushless);
    turningSparkMax = new CANSparkMax(turningCanId, MotorType.kBrushless);
    chassisAngularOffsetRadians = chassisAngularOffset;

    SparkMaxUtils.initWithRetry(this::initDriveSpark, Cal.SPARK_INIT_RETRY_ATTEMPTS);
    SparkMaxUtils.initWithRetry(this::initTurnSpark, Cal.SPARK_INIT_RETRY_ATTEMPTS);

    drivingEncoder = drivingSparkMax.getEncoder();
    drivingPIDController = drivingSparkMax.getPIDController();
    turningEncoder = turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    turningPIDController = turningSparkMax.getPIDController();

    desiredState.angle = new Rotation2d(turningEncoder.getPosition());
    drivingEncoder.setPosition(0);
  }

  /** Does all the initialization for the spark, return true on success */
  boolean initTurnSpark() {
    int errors = 0;

    errors += SparkMaxUtils.check(turningSparkMax.restoreFactoryDefaults());
    turningSparkMax.setInverted(false);

    AbsoluteEncoder turningEncoderTmp = turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    SparkMaxPIDController turningPidTmp = turningSparkMax.getPIDController();
    errors += SparkMaxUtils.check(turningPidTmp.setFeedbackDevice(turningEncoderTmp));

    // Gear ratio 1.0 because the encoder is 1:1 with the module (doesn't involve the actual turning
    // gear ratio)
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setRadsFromGearRatio(turningEncoderTmp, 1.0));

    errors +=
        SparkMaxUtils.check(
            turningEncoderTmp.setInverted(Constants.SwerveModule.TURNING_ENCODER_INVERTED));

    errors += SparkMaxUtils.check(turningPidTmp.setPositionPIDWrappingEnabled(true));
    errors +=
        SparkMaxUtils.check(
            turningPidTmp.setPositionPIDWrappingMinInput(
                Constants.SwerveModule.TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS));
    errors +=
        SparkMaxUtils.check(
            turningPidTmp.setPositionPIDWrappingMaxInput(
                Constants.SwerveModule.TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS));

    errors += SparkMaxUtils.check(turningPidTmp.setP(Cal.SwerveModule.TURNING_P));
    errors += SparkMaxUtils.check(turningPidTmp.setI(Cal.SwerveModule.TURNING_I));
    errors += SparkMaxUtils.check(turningPidTmp.setD(Cal.SwerveModule.TURNING_D));
    errors += SparkMaxUtils.check(turningPidTmp.setFF(Cal.SwerveModule.TURNING_FF));
    errors +=
        SparkMaxUtils.check(
            turningPidTmp.setOutputRange(
                Cal.SwerveModule.TURNING_MIN_OUTPUT, Cal.SwerveModule.TURNING_MAX_OUTPUT));

    errors +=
        SparkMaxUtils.check(
            turningSparkMax.setIdleMode(Constants.SwerveModule.TURNING_MOTOR_IDLE_MODE));
    errors +=
        SparkMaxUtils.check(
            turningSparkMax.setSmartCurrentLimit(
                Constants.SwerveModule.TURNING_MOTOR_CURRENT_LIMIT_AMPS));

    errors += SparkMaxUtils.check(turningSparkMax.setIdleMode(IdleMode.kBrake));

    errors +=
        SparkMaxUtils.check(
            turningSparkMax.setSmartCurrentLimit(Cal.SwerveSubsystem.STEER_CURRENT_LIMIT_AMPS));

    return errors == 0;
  }

  /** Does all the initialization for the spark, return true on success */
  boolean initDriveSpark() {
    int errors = 0;
    errors += SparkMaxUtils.check(drivingSparkMax.restoreFactoryDefaults());

    drivingSparkMax.setInverted(true);

    RelativeEncoder drivingEncoderTmp = drivingSparkMax.getEncoder();
    SparkMaxPIDController drivingPidTmp = drivingSparkMax.getPIDController();
    errors += SparkMaxUtils.check(drivingPidTmp.setFeedbackDevice(drivingEncoderTmp));

    errors += SparkMaxUtils.check(drivingPidTmp.setP(Cal.SwerveModule.DRIVING_P));
    errors += SparkMaxUtils.check(drivingPidTmp.setI(Cal.SwerveModule.DRIVING_I));
    errors += SparkMaxUtils.check(drivingPidTmp.setD(Cal.SwerveModule.DRIVING_D));
    errors += SparkMaxUtils.check(drivingPidTmp.setFF(Cal.SwerveModule.DRIVING_FF));
    errors +=
        SparkMaxUtils.check(
            drivingPidTmp.setOutputRange(
                Cal.SwerveModule.DRIVING_MIN_OUTPUT, Cal.SwerveModule.DRIVING_MAX_OUTPUT));

    errors +=
        SparkMaxUtils.check(
            drivingEncoderTmp.setPositionConversionFactor(
                Constants.SwerveModule.DRIVING_ENCODER_POSITION_FACTOR_METERS));
    errors +=
        SparkMaxUtils.check(
            drivingEncoderTmp.setVelocityConversionFactor(
                Constants.SwerveModule.DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND));

    errors +=
        SparkMaxUtils.check(
            drivingSparkMax.setIdleMode(Constants.SwerveModule.DRIVING_MOTOR_IDLE_MODE));
    errors +=
        SparkMaxUtils.check(
            drivingSparkMax.setSmartCurrentLimit(
                Constants.SwerveModule.DRIVING_MOTOR_CURRENT_LIMIT_AMPS));

    errors += SparkMaxUtils.check(drivingSparkMax.setIdleMode(IdleMode.kCoast));

    errors +=
        SparkMaxUtils.check(
            drivingSparkMax.setSmartCurrentLimit(Cal.SwerveSubsystem.DRIVE_CURRENT_LIMIT_AMPS));

    return errors == 0;
  }

  /**
   * Burns the current settings to sparks so they keep current settings on reboot. Should be done
   * after all settings are set.
   */
  public void burnFlashSparks() {
    Timer.delay(0.005);
    drivingSparkMax.burnFlash();
    Timer.delay(0.005);
    turningSparkMax.burnFlash();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(
        drivingEncoder.getVelocity(),
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffsetRadians));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        drivingEncoder.getPosition(),
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffsetRadians));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle =
        desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffsetRadians));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState =
        SwerveModuleState.optimize(
            correctedDesiredState, new Rotation2d(turningEncoder.getPosition()));

            desiredVelMps = optimizedDesiredState.speedMetersPerSecond;
            desiredPosRad = optimizedDesiredState.angle.getRadians();

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivingPIDController.setReference(
        optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    // This turns off the drive
    // drivingPIDController.setReference( 
    //     0.0, CANSparkMax.ControlType.kVelocity);
    turningPIDController.setReference(
        optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    drivingEncoder.setPosition(0);
  }

  public void initSendable(SendableBuilder builder) {
    builder.setActuator(true);
    builder.addDoubleProperty("Driving kP", drivingPIDController::getP, drivingPIDController::setP);
    builder.addDoubleProperty("Driving kI", drivingPIDController::getI, drivingPIDController::setI);
    builder.addDoubleProperty("Driving kD", drivingPIDController::getD, drivingPIDController::setD);
    builder.addDoubleProperty(
        "Driving kFF", drivingPIDController::getFF, drivingPIDController::setFF);
    builder.addDoubleProperty("Turning kP", turningPIDController::getP, turningPIDController::setP);
    builder.addDoubleProperty("Turning kI", turningPIDController::getI, turningPIDController::setI);
    builder.addDoubleProperty("Turning kD", turningPIDController::getD, turningPIDController::setD);
    builder.addDoubleProperty(
        "Turning kFF", turningPIDController::getFF, turningPIDController::setFF);
    builder.addDoubleProperty("Driving Vel (m/s)", drivingEncoder::getVelocity, null);
    builder.addDoubleProperty("Steering Pos (rad)", turningEncoder::getPosition, null);
    builder.addDoubleProperty(
        "Desired Vel (m/s)",
        () -> {
          return desiredState.speedMetersPerSecond;
        },
        null);
    builder.addDoubleProperty(
        "Desired Steer (rad)",
        () -> {
          return desiredState.angle.getRadians();
        },
        null);
  }
}
