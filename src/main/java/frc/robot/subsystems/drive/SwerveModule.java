// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Calibrations;
import frc.robot.Constants;

public class SwerveModule implements Sendable {
  private final CANSparkMax drivingSparkMax;
  private final CANSparkMax turningSparkMax;

  private final RelativeEncoder drivingEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final SparkMaxPIDController drivingPIDController;
  private final SparkMaxPIDController turningPIDController;

  private double chassisAngularOffsetRadians = 0;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a SwerveModule and configures the driving and turning motor, encoder, and PID
   * controller. This configuration is specific to the REV MAXSwerve Module built with NEOs, SPARKS
   * MAX, and a Through Bore Encoder.
   */
  // TODO: Check error codes from various setters
  public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    drivingSparkMax.restoreFactoryDefaults();
    turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    drivingEncoder = drivingSparkMax.getEncoder();
    turningEncoder = turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    drivingPIDController = drivingSparkMax.getPIDController();
    turningPIDController = turningSparkMax.getPIDController();
    drivingPIDController.setFeedbackDevice(drivingEncoder);
    turningPIDController.setFeedbackDevice(turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    drivingEncoder.setPositionConversionFactor(
        Constants.SwerveModule.DRIVING_ENCODER_POSITION_FACTOR_METERS);
    drivingEncoder.setVelocityConversionFactor(
        Constants.SwerveModule.DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    turningEncoder.setPositionConversionFactor(
        Constants.SwerveModule.TURNING_ENCODER_POSITION_FACTOR_RADIANS);
    turningEncoder.setVelocityConversionFactor(
        Constants.SwerveModule.TURNING_ENCODER_VELOCITY_FACTOR_RAD_PER_SEC);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the Swerve Module.
    turningEncoder.setInverted(Constants.SwerveModule.TURNING_ENCODER_INVERTED);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    turningPIDController.setPositionPIDWrappingEnabled(true);
    turningPIDController.setPositionPIDWrappingMinInput(
        Constants.SwerveModule.TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS);
    turningPIDController.setPositionPIDWrappingMaxInput(
        Constants.SwerveModule.TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS);

    // Set the PID gains for the driving motor.
    drivingPIDController.setP(Calibrations.SwerveModule.DRIVING_P);
    drivingPIDController.setI(Calibrations.SwerveModule.DRIVING_I);
    drivingPIDController.setD(Calibrations.SwerveModule.DRIVING_D);
    drivingPIDController.setFF(Calibrations.SwerveModule.DRIVING_FF);
    drivingPIDController.setOutputRange(
        Calibrations.SwerveModule.DRIVING_MIN_OUTPUT, Calibrations.SwerveModule.DRIVING_MAX_OUTPUT);

    // Set the PID gains for the turning motor.
    turningPIDController.setP(Calibrations.SwerveModule.TURNING_P);
    turningPIDController.setI(Calibrations.SwerveModule.TURNING_I);
    turningPIDController.setD(Calibrations.SwerveModule.TURNING_D);
    turningPIDController.setFF(Calibrations.SwerveModule.TURNING_FF);
    turningPIDController.setOutputRange(
        Calibrations.SwerveModule.TURNING_MIN_OUTPUT, Calibrations.SwerveModule.TURNING_MAX_OUTPUT);

    drivingSparkMax.setIdleMode(Constants.SwerveModule.DRIVING_MOTOR_IDLE_MODE);
    turningSparkMax.setIdleMode(Constants.SwerveModule.TURNING_MOTOR_IDLE_MODE);
    drivingSparkMax.setSmartCurrentLimit(Constants.SwerveModule.DRIVING_MOTOR_CURRENT_LIMIT_AMPS);
    turningSparkMax.setSmartCurrentLimit(Constants.SwerveModule.TURNING_MOTOR_CURRENT_LIMIT_AMPS);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    drivingSparkMax.burnFlash();
    turningSparkMax.burnFlash();

    desiredState.angle = new Rotation2d(turningEncoder.getPosition());
    drivingEncoder.setPosition(0);
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

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivingPIDController.setReference(
        optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turningPIDController.setReference(
        optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    drivingEncoder.setPosition(0);
  }

  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Driving kP", drivingPIDController::getP, drivingPIDController::setP);
    builder.addDoubleProperty("Driving kI", drivingPIDController::getI, drivingPIDController::setI);
    builder.addDoubleProperty("Driving kD", drivingPIDController::getD, drivingPIDController::setD);
    builder.addDoubleProperty("Driving kFF", drivingPIDController::getFF, drivingPIDController::setFF);
    builder.addDoubleProperty("Turning kP", turningPIDController::getP, turningPIDController::setP);
    builder.addDoubleProperty("Turning kI", turningPIDController::getI, turningPIDController::setI);
    builder.addDoubleProperty("Turning kD", turningPIDController::getD, turningPIDController::setD);
    builder.addDoubleProperty("Turning kFF", turningPIDController::getFF, turningPIDController::setFF);
  }
}
