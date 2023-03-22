// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Cal;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.utils.AbsoluteEncoderChecker;
import frc.robot.utils.AngleUtil;
import frc.robot.utils.SendableHelper;
import frc.robot.utils.SparkMaxUtils;
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

  /** Input deg, output Volts */
  private ProfiledPIDController deployMotorController =
      new ProfiledPIDController(
          Cal.Intake.DEPLOY_MOTOR_P,
          Cal.Intake.DEPLOY_MOTOR_I,
          Cal.Intake.DEPLOY_MOTOR_D,
          new TrapezoidProfile.Constraints(
              Cal.Intake.DEPLOY_MAX_VELOCITY_DEG_PER_SECOND,
              Cal.Intake.DEPLOY_MAX_ACCELERATION_DEG_PER_SECOND_SQUARED));

  private Solenoid clampLeft =
      new Solenoid(PneumaticsModuleType.REVPH, RobotMap.INTAKE_CLAMP_LEFT_CHANNEL);
  private Solenoid clampRight =
      new Solenoid(PneumaticsModuleType.REVPH, RobotMap.INTAKE_CLAMP_RIGHT_CHANNEL);
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
  private double intakeControlPositionDegrees = Cal.Intake.STARTING_POSITION_DEGREES;
  private double intakeMostRecentGoalDegrees = Cal.Intake.STARTING_POSITION_DEGREES;
  private boolean desiredDeployed = false;
  private boolean desireClamped = false;
  private BooleanSupplier clearOfIntake;
  private AbsoluteEncoderChecker deployMotorAbsoluteEncoderChecker = new AbsoluteEncoderChecker();
  private double lastControlledTime = Timer.getFPGATimestamp();
  private double lastControlledVelocity = 0.0;

  /** Creates a new Intake. */
  public Intake(BooleanSupplier clearOfIntakeZone) {
    clearOfIntake = clearOfIntakeZone;

    SparkMaxUtils.initWithRetry(this::setUpDeploySpark, Cal.SPARK_INIT_RETRY_ATTEMPTS);
    SparkMaxUtils.initWithRetry(this::setUpIntakeWheelSparks, Cal.SPARK_INIT_RETRY_ATTEMPTS);

    deployMotorController.setTolerance(Cal.Intake.DEPLOY_ALLOWED_CLOSED_LOOP_ERROR_DEG);
  }

  /** Does all the initialization for the sparks, return true on success */
  private boolean setUpIntakeWheelSparks() {
    int errors = 0;
    errors += SparkMaxUtils.check(intakeLeft.restoreFactoryDefaults());

    errors += SparkMaxUtils.check(intakeRight.restoreFactoryDefaults());
    errors += SparkMaxUtils.check(intakeRight.follow(intakeLeft, true));

    intakeLeft.setInverted(true);

    errors += SparkMaxUtils.check(intakeLeft.setIdleMode(IdleMode.kCoast));
    errors += SparkMaxUtils.check(intakeRight.setIdleMode(IdleMode.kCoast));

    errors +=
        SparkMaxUtils.check(
            intakeLeft.setSmartCurrentLimit(Cal.Intake.INTAKE_WHEELS_CURRENT_LIMIT_AMPS));
    errors +=
        SparkMaxUtils.check(
            intakeRight.setSmartCurrentLimit(Cal.Intake.INTAKE_WHEELS_CURRENT_LIMIT_AMPS));

    return errors == 0;
  }

  /** Does all the initialization for the spark, return true on success */
  private boolean setUpDeploySpark() {
    int errors = 0;

    errors += SparkMaxUtils.check(deployMotor.restoreFactoryDefaults());
    deployMotor.setInverted(false);
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setDegreesFromGearRatio(
                deployMotorEncoder, Constants.Intake.DEPLOY_MOTOR_GEAR_RATIO));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setDegreesFromGearRatio(
                deployMotorAbsoluteEncoder, Constants.Intake.DEPLOY_ABS_ENCODER_GEAR_RATIO));

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

    errors += SparkMaxUtils.check(deployMotor.setIdleMode(IdleMode.kBrake));

    errors +=
        SparkMaxUtils.check(
            deployMotor.setSmartCurrentLimit(Cal.Intake.INTAKE_DEPLOY_MOTOR_CURRENT_LIMIT_AMPS));

    return errors == 0;
  }

  public double getOffsetAbsPositionDeg() {
    return deployMotorAbsoluteEncoder.getPosition() + Cal.Intake.ABSOLUTE_ENCODER_ZERO_OFFSET_DEG;
  }

  public void initialize() {
    rezeroIntake();
  }

  public void rezeroIntake() {
    deployMotorEncoder.setPosition(
        AngleUtil.wrapAngle(
            deployMotorAbsoluteEncoderChecker.getMedian()
                - Cal.Intake.ABSOLUTE_ENCODER_START_POS_DEG
                + Cal.Intake.STARTING_POSITION_DEGREES));
    deployMotorController.reset(getOffsetAbsPositionDeg());
  }

  public void zeroIntakeAtCurrentPos() {
    Cal.Intake.ABSOLUTE_ENCODER_START_POS_DEG = getOffsetAbsPositionDeg();
    System.out.println("New Zero for Deploy Motor: " + Cal.Intake.ABSOLUTE_ENCODER_START_POS_DEG);
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

  /** Sends the deploy motor voltage, needs to be called every cycle */
  private void controlPosition(double controlPositionDeg) {
    // if (positionDeg != intakeMostRecentGoalDegrees) {
    //   deployMotorController.setGoal(positionDeg);
    //   intakeMostRecentGoalDegrees = positionDeg;
    // }
    // TODO: should we always set the goal or no?
    deployMotorController.setGoal(controlPositionDeg);

    double currentPositionDeg = getOffsetAbsPositionDeg();

    double desiredAccelDegPerSecSq =
        (deployMotorController.getSetpoint().velocity - lastControlledVelocity)
            / (Timer.getFPGATimestamp() - lastControlledTime);

    double demandVoltsA = deployMotorController.calculate(getOffsetAbsPositionDeg());
    double demandVoltsB =
        Cal.Intake.DEPLOY_FEEDFORWARD.calculate(
            deployMotorController.getSetpoint().velocity, desiredAccelDegPerSecSq);
    double demandVoltsC = Cal.Intake.ARBITRARY_FEED_FORWARD_VOLTS * getCosineIntakeAngle();

    SmartDashboard.putNumber("Intake Volts PID", demandVoltsA);
    SmartDashboard.putNumber("Intake Volts FF", demandVoltsB);
    SmartDashboard.putNumber("Intake Volts Gravity", demandVoltsC);
    lastControlledTime = Timer.getFPGATimestamp();
    lastControlledVelocity = deployMotorController.getSetpoint().velocity;

    // TODO fix this hacky stuff
    final double POSITION_NO_POWER = 185.0;
    if (controlPositionDeg > POSITION_NO_POWER && currentPositionDeg > POSITION_NO_POWER) {
      deployMotor.setVoltage(0.0);
      return;
    }

    final double POSITION_HOLD_INTAKE_IN = 100.0;
    if (controlPositionDeg < POSITION_HOLD_INTAKE_IN
        && currentPositionDeg < POSITION_HOLD_INTAKE_IN) {
      deployMotor.setVoltage(-2.0);
      return;
    }

    deployMotor.setVoltage(demandVoltsA + demandVoltsB + demandVoltsC);
  }

  /** Deploys the intake out. */
  private void deploy() {
    desireClamped = true;

    // Set the intake control position
    intakeControlPositionDegrees = Cal.Intake.DEPLOYED_POSITION_DEGREES;
  }

  /** Brings the intake back in */
  private void retract() {
    desireClamped = false;

    // Set the intake control position
    intakeControlPositionDegrees = Cal.Intake.RETRACTED_POSITION_DEGREES;

    stopIntakingGamePiece();
  }

  /*Setter for whether intake is desired deploy is true retract is false */
  public void setDesiredDeployed(boolean desired) {
    desiredDeployed = desired;
  }

  /** Returns the cosine of the intake angle in degrees off of the horizontal. */
  private double getCosineIntakeAngle() {
    return Math.cos(
        Units.degreesToRadians(
            getOffsetAbsPositionDeg() - Constants.Intake.POSITION_WHEN_HORIZONTAL_DEGREES));
  }

  /** If the intake has achieved its desired position, return true */
  public boolean atDesiredPosition() {
    // May not be the same as the control position, if the lift is in the way
    double desiredPositionDegrees =
        desiredDeployed
            ? Cal.Intake.DEPLOYED_POSITION_DEGREES
            : Cal.Intake.RETRACTED_POSITION_DEGREES;
    return (Math.abs(desiredPositionDegrees - getOffsetAbsPositionDeg())
        < Cal.Intake.DEPLOY_ALLOWED_CLOSED_LOOP_ERROR_DEG);
  }

  public void setDesiredClamped(boolean clamp) {
    desireClamped = clamp;
  }

  private void clampIntake() {
    clampLeft.set(true);
    clampRight.set(true);
  }

  private void unclampIntake() {
    clampLeft.set(false);
    clampRight.set(false);
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
    // return !gamePieceSensor.get();
    return false;
  }

  @Override
  public void periodic() {
    if (clearOfIntake.getAsBoolean()) {
      if (desiredDeployed) {
        deploy();
      } else {
        retract();
      }
    }

    controlPosition(intakeControlPositionDegrees);
    deployMotorAbsoluteEncoderChecker.addReading(deployMotorAbsoluteEncoder.getPosition());

    // Only clamp if it is safe to do so and clamping is desired
    if (desireClamped && getOffsetAbsPositionDeg() > Cal.Intake.CLAMP_POSITION_THRESHOLD_DEGREES) {
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
    SendableHelper.addChild(builder, this, deployMotorController, "DeployController");
    builder.addDoubleProperty(
        "Intake Desired Position (Degrees)", () -> intakeControlPositionDegrees, null);
    builder.addDoubleProperty(
        "Intake Rel Enc Position (Degrees)",
        deployMotorEncoder::getPosition,
        deployMotorEncoder::setPosition);
    builder.addDoubleProperty(
        "Intake Abs Position (deg)", deployMotorAbsoluteEncoder::getPosition, null);
    builder.addDoubleProperty(
        "Intake Abs Offset Position (deg)", this::getOffsetAbsPositionDeg, null);

    builder.addDoubleProperty(
        "Intake Abs Deploy Velocity (deg/s)", deployMotorAbsoluteEncoder::getVelocity, null);
    builder.addBooleanProperty("See Game Piece", this::seeGamePiece, null);
    builder.addBooleanProperty(
        "Desire clamped",
        () -> {
          return desireClamped;
        },
        null);
    builder.addBooleanProperty("At Desired Pos", this::atDesiredPosition, null);
    builder.addDoubleProperty("Intake wheel power in [-1,1]", intakeLeft::get, null);
    builder.addBooleanProperty(
        "Desire deployed",
        () -> {
          return desiredDeployed;
        },
        null);
    builder.addBooleanProperty(
        "Deploy encoder connected", deployMotorAbsoluteEncoderChecker::encoderConnected, null);
  }
}
