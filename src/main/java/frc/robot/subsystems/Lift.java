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
import edu.wpi.first.math.Pair;
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
import frc.robot.utils.ScoringLocationUtil;
import frc.robot.utils.ScoringLocationUtil.ScoreHeight;
import frc.robot.utils.SparkMaxUtils;
import java.util.TreeMap;

/** Contains code for elevator, arm, and game piece grabber */
public class Lift extends SubsystemBase {

  /** Overall position of the lift including both elevator and arm */
  public enum LiftPosition {
    GRAB_FROM_INTAKE,
    SHELF,
    SCORE_LOW,
    SCORE_MID_CUBE,
    SCORE_MID_CONE,
    SCORE_HIGH_CUBE,
    SCORE_HIGH_CONE,
    PRE_SCORE_MID_CONE,
    PRE_SCORE_HIGH_CONE,
    POST_SCORE_HIGH,
    OUTTAKING,
    STARTING
  }

  /** Position of the lift relative to to the start position */
  public enum LiftPositionStartRelative {
    BELOW_START,
    AT_START,
    ABOVE_START
  }

  // Actuators
  private CANSparkMax elevatorLeft =
      new CANSparkMax(RobotMap.ELEVATOR_MOTOR_LEFT_CAN_ID, MotorType.kBrushless);
  private CANSparkMax elevatorRight =
      new CANSparkMax(RobotMap.ELEVATOR_MOTOR_RIGHT_CAN_ID, MotorType.kBrushless);
  private SparkMaxPIDController elevatorLeftPID = elevatorLeft.getPIDController();
  private CANSparkMax arm = new CANSparkMax(RobotMap.ARM_MOTOR_CAN_ID, MotorType.kBrushless);
  private SparkMaxPIDController armPID = arm.getPIDController();
  private Solenoid grabber =
      new Solenoid(PneumaticsModuleType.REVPH, RobotMap.LIFT_GRABBING_CHANNEL);

  // Sensors
  private final RelativeEncoder elevatorLeftEncoder = elevatorLeft.getEncoder();
  private final AbsoluteEncoder elevatorLeftAbsEncoder =
      elevatorLeft.getAbsoluteEncoder(Type.kDutyCycle);
  private final AbsoluteEncoder elevatorRightAbsEncoder =
      elevatorRight.getAbsoluteEncoder(Type.kDutyCycle);
  private final RelativeEncoder armEncoder = arm.getEncoder();
  private final AbsoluteEncoder armAbsoluteEncoder = arm.getAbsoluteEncoder(Type.kDutyCycle);
  private final DigitalInput gamePieceSensor = new DigitalInput(RobotMap.LIFT_GAME_PIECE_DIO);

  // Members
  private final int SMART_MOTION_SLOT = 0;
  private LiftPosition latestPosition = LiftPosition.STARTING;
  private LiftPosition desiredPosition = LiftPosition.STARTING;
  private boolean desiredGrabberClosed = true;
  public ScoringLocationUtil scoreLoc;

  /**
   * Indicates the elevator and arm positions at each position of the lift. The first value
   * indicates the elevator position in inches and the second value indicates the arm position in
   * degrees
   */
  TreeMap<LiftPosition, Pair<Double, Double>> liftPositionMap;

  /** Creates a new Lift */
  public Lift(ScoringLocationUtil scoreLoc) {
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
        LiftPosition.SCORE_LOW,
        new Pair<Double, Double>(Cal.PLACEHOLDER_DOUBLE, Cal.PLACEHOLDER_DOUBLE));
    liftPositionMap.put(
        LiftPosition.SCORE_MID_CUBE,
        new Pair<Double, Double>(Cal.PLACEHOLDER_DOUBLE, Cal.PLACEHOLDER_DOUBLE));
    liftPositionMap.put(
        LiftPosition.SCORE_MID_CONE,
        new Pair<Double, Double>(Cal.PLACEHOLDER_DOUBLE, Cal.PLACEHOLDER_DOUBLE));
    liftPositionMap.put(
        LiftPosition.SCORE_HIGH_CUBE,
        new Pair<Double, Double>(Cal.PLACEHOLDER_DOUBLE, Cal.PLACEHOLDER_DOUBLE));
    liftPositionMap.put(
        LiftPosition.SCORE_HIGH_CONE,
        new Pair<Double, Double>(Cal.PLACEHOLDER_DOUBLE, Cal.PLACEHOLDER_DOUBLE));
    liftPositionMap.put(
        LiftPosition.PRE_SCORE_MID_CONE,
        new Pair<Double, Double>(Cal.PLACEHOLDER_DOUBLE, Cal.PLACEHOLDER_DOUBLE));
    liftPositionMap.put(
        LiftPosition.PRE_SCORE_HIGH_CONE,
        new Pair<Double, Double>(Cal.PLACEHOLDER_DOUBLE, Cal.PLACEHOLDER_DOUBLE));
    liftPositionMap.put(
        LiftPosition.POST_SCORE_HIGH,
        new Pair<Double, Double>(Cal.PLACEHOLDER_DOUBLE, Cal.PLACEHOLDER_DOUBLE));
    liftPositionMap.put(
        LiftPosition.OUTTAKING,
        new Pair<Double, Double>(Cal.PLACEHOLDER_DOUBLE, Cal.PLACEHOLDER_DOUBLE));
    liftPositionMap.put(
        LiftPosition.STARTING,
        new Pair<Double, Double>(Cal.PLACEHOLDER_DOUBLE, Cal.PLACEHOLDER_DOUBLE));

    this.scoreLoc = scoreLoc;
  }

  /** Does all the initialization for the sparks, return true on success */
  private boolean initSparks() {
    int errors = 0;

    errors += SparkMaxUtils.check(elevatorLeft.restoreFactoryDefaults());
    errors += SparkMaxUtils.check(elevatorRight.restoreFactoryDefaults());
    errors += SparkMaxUtils.check(arm.restoreFactoryDefaults());

    errors += SparkMaxUtils.check(elevatorRight.follow(elevatorLeft, true));

    // Get positions and degrees of elevator through encoder in inches
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setDegreesFromGearRatio(elevatorLeftAbsEncoder, 1.0));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setDegreesFromGearRatio(
                elevatorRightAbsEncoder, Constants.Lift.ELEVATOR_RIGHT_ABSOLUTE_ENCODER_RATIO));
    errors +=
        SparkMaxUtils.check(
            elevatorLeftEncoder.setPositionConversionFactor(
                Constants.Lift.ELEVATOR_MOTOR_ENCODER_IN_PER_REV));
    errors +=
        SparkMaxUtils.check(
            elevatorLeftEncoder.setVelocityConversionFactor(
                Constants.Lift.ELEVATOR_MOTOR_ENCODER_IPS_PER_RPM));

    // Set PID of Elevator
    errors += SparkMaxUtils.check(elevatorLeftPID.setP(Cal.Lift.ELEVATOR_P));
    errors += SparkMaxUtils.check(elevatorLeftPID.setI(Cal.Lift.ELEVATOR_I));
    errors += SparkMaxUtils.check(elevatorLeftPID.setD(Cal.Lift.ELEVATOR_D));
    errors +=
        SparkMaxUtils.check(
            elevatorLeftPID.setSmartMotionMaxAccel(
                Cal.Lift.ELEVATOR_MAX_ACCELERATION_IN_PER_SECOND_SQUARED, SMART_MOTION_SLOT));
    errors +=
        SparkMaxUtils.check(
            elevatorLeftPID.setSmartMotionMaxVelocity(
                Cal.Lift.ELEVATOR_MAX_VELOCITY_IN_PER_SECOND, SMART_MOTION_SLOT));
    errors +=
        SparkMaxUtils.check(
            elevatorLeftPID.setSmartMotionMinOutputVelocity(
                Cal.Lift.ELEVATOR_MIN_OUTPUT_VELOCITY_IN_PER_SECOND, SMART_MOTION_SLOT));
    errors +=
        SparkMaxUtils.check(
            elevatorLeftPID.setSmartMotionAllowedClosedLoopError(
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

    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setDegreesFromGearRatio(
                armEncoder, Constants.Lift.ARM_MOTOR_GEAR_RATIO));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setDegreesFromGearRatio(armAbsoluteEncoder, 1.0));

    errors +=
        SparkMaxUtils.check(
            elevatorLeft.setSoftLimit(
                SoftLimitDirection.kForward, Cal.Lift.ELEVATOR_POSITIVE_LIMIT_INCHES));
    errors += SparkMaxUtils.check(elevatorLeft.enableSoftLimit(SoftLimitDirection.kForward, true));
    errors +=
        SparkMaxUtils.check(
            elevatorLeft.setSoftLimit(
                SoftLimitDirection.kReverse, Cal.Lift.ELEVATOR_NEGATIVE_LIMIT_INCHES));
    errors += SparkMaxUtils.check(elevatorLeft.enableSoftLimit(SoftLimitDirection.kReverse, true));

    errors +=
        SparkMaxUtils.check(
            arm.setSoftLimit(SoftLimitDirection.kForward, Cal.Lift.ARM_POSITIVE_LIMIT_DEGREES));
    errors += SparkMaxUtils.check(arm.enableSoftLimit(SoftLimitDirection.kForward, true));
    errors +=
        SparkMaxUtils.check(
            arm.setSoftLimit(SoftLimitDirection.kReverse, Cal.Lift.ARM_NEGATIVE_LIMIT_DEGREES));
    errors += SparkMaxUtils.check(arm.enableSoftLimit(SoftLimitDirection.kReverse, true));

    return errors == 0;
  }

  /**
   * Burns the current settings to sparks so they keep current settings on reboot. Should be done
   * after all settings are set.
   */
  public void burnFlashSparks() {
    Timer.delay(0.005);
    elevatorLeft.burnFlash();
    Timer.delay(0.005);
    arm.burnFlash();
  }

  private void goToPosition(LiftPosition pos) {
    elevatorLeftPID.setReference(
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

  public void closeGrabber() {
    desiredGrabberClosed = true;
  }

  public void openGrabber() {
    desiredGrabberClosed = false;
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
        AngleUtil.wrapAngle(
            armAbsoluteEncoder.getPosition() - Cal.Lift.ARM_ABSOLUTE_ENCODER_ZERO_POS_DEG));

    // Set elevator encoder position from absolute encoders
    double elevatorDutyCycleEncodersDifferenceDegrees =
        AngleUtil.wrapAngle(
            elevatorRightAbsEncoder.getPosition() - elevatorLeftEncoder.getPosition());
    elevatorLeftEncoder.setPosition(
        elevatorDutyCycleEncodersDifferenceDegrees
            * Constants.Lift.ELEVATOR_MOTOR_ENCODER_DIFFERENCES_SCALAR_INCHES_PER_DEGREE);
  }

  public void manualPrepScore() {
    // TODO do this
  }

  public void cancelScore() {
    // TODO do this
  }

  public void home() {
    setDesiredPosition(LiftPosition.STARTING);
  }

  public void prepScore() {
    // TODO do this
  }

  public void startScore() {
    if (!scoreLoc.isCone()) {
      if (scoreLoc.getScoreHeight() == ScoreHeight.HIGH) {
        setDesiredPosition(LiftPosition.PRE_SCORE_HIGH_CONE);
      } else if (scoreLoc.getScoreHeight() == ScoreHeight.MID) {
        setDesiredPosition(LiftPosition.PRE_SCORE_MID_CONE);
      }
    }
  }

  /** Sets the desired position, which the lift may not go to directly. */
  public void setDesiredPosition(LiftPosition pos) {
    desiredPosition = pos;
  }

  /** True if the lift is at the queried position. */
  public boolean atPosition(LiftPosition positionToCheck) {
    double armMarginDegrees =
        positionToCheck == LiftPosition.STARTING
            ? Cal.Lift.ARM_START_MARGIN_DEGREES
            : Cal.Lift.ARM_MARGIN_DEGREES;
    double elevatorMarginInches =
        positionToCheck == LiftPosition.STARTING
            ? Cal.Lift.ELEVATOR_START_MARGIN_INCHES
            : Cal.Lift.ELEVATOR_MARGIN_INCHES;
    double elevatorPositionToCheckInches = liftPositionMap.get(positionToCheck).getFirst();
    double armPositionToCheckDegrees = liftPositionMap.get(positionToCheck).getSecond();
    double elevatorPositionInches = elevatorLeftEncoder.getPosition();
    double armPositionDegrees = armEncoder.getPosition();
    if (Math.abs(armPositionDegrees - armPositionToCheckDegrees) > armMarginDegrees) {
      return false;
    }

    if (Math.abs(elevatorPositionInches - elevatorPositionToCheckInches) > elevatorMarginInches) {
      return false;
    }

    return true;
  }

  /** Converts a lift position to a relative lift position (above or below starting position). */
  private static LiftPositionStartRelative getRelativeLiftPosition(LiftPosition pos) {
    switch (pos) {
      case STARTING:
        return LiftPositionStartRelative.AT_START;
      case SCORE_MID_CUBE:
      case SCORE_MID_CONE:
      case SCORE_HIGH_CUBE:
      case SCORE_HIGH_CONE:
      case SCORE_LOW:
      case PRE_SCORE_MID_CONE:
      case PRE_SCORE_HIGH_CONE:
      case POST_SCORE_HIGH:
      case OUTTAKING:
      case SHELF:
        return LiftPositionStartRelative.ABOVE_START;
      case GRAB_FROM_INTAKE:
        return LiftPositionStartRelative.BELOW_START;
    }
    // should never be used?
    return LiftPositionStartRelative.ABOVE_START;
  }

  /** Called every scheduler run */
  @Override
  public void periodic() {
    // Check if position has updated.
    if (atPosition(LiftPosition.STARTING)) {
      latestPosition = LiftPosition.STARTING;
    } else if (atPosition(desiredPosition)) {
      latestPosition = desiredPosition;
    }

    // If the lift is going from below to above or above to below, we have to transit through
    // starting position. Otherwise, we can go directly to the desired position.
    LiftPositionStartRelative latestPositionStartRelative = getRelativeLiftPosition(latestPosition);
    LiftPositionStartRelative desiredPositionStartRelative =
        getRelativeLiftPosition(desiredPosition);
    if (!seeGamePiece()) {
      goToPosition(desiredPosition);
    } else if (desiredPositionStartRelative == LiftPositionStartRelative.BELOW_START
        && latestPositionStartRelative == LiftPositionStartRelative.ABOVE_START) {
      goToPosition(LiftPosition.STARTING);
    } else if (desiredPositionStartRelative == LiftPositionStartRelative.ABOVE_START
        && latestPositionStartRelative == LiftPositionStartRelative.BELOW_START) {
      goToPosition(LiftPosition.STARTING);
    } else {
      goToPosition(desiredPosition);
    }

    // If the grabber is set to open and it is safe to open, open the grabber (drop). Otherwise,
    // close it (grab).
    if (!desiredGrabberClosed
        && (armEncoder.getPosition() > Cal.Lift.GRABBER_CLOSED_ZONE_TOP_DEGREES
            || armEncoder.getPosition() < Cal.Lift.GRABBER_CLOSED_ZONE_BOTTOM_DEGREES)) {
      grabber.set(false); // drop
    } else {
      grabber.set(true); // grab
    }
  }

  /**
   * Returns true if the lift is clear of the zone where the intake moves, so that the intake can
   * move as soon as the lift is clear of that zone
   */
  public boolean clearOfIntakeZone() {
    if ((elevatorLeftEncoder.getPosition() > Cal.Lift.ELEVATOR_INTAKE_ZONE_THRESHOLD_INCHES)
        || (armEncoder.getPosition() > Cal.Lift.ARM_INTAKE_ZONE_THRESHOLD_DEGREES)) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Elevator kP", elevatorLeftPID::getP, elevatorLeftPID::setP);
    builder.addDoubleProperty("Elevator kI", elevatorLeftPID::getI, elevatorLeftPID::setI);
    builder.addDoubleProperty("Elevator kD", elevatorLeftPID::getD, elevatorLeftPID::setD);
    builder.addDoubleProperty(
        "Elevator Position", elevatorLeftEncoder::getPosition, elevatorLeftEncoder::setPosition);
    builder.addDoubleProperty("Arm kP", armPID::getP, armPID::setP);
    builder.addDoubleProperty("Arm kI", armPID::getI, armPID::setI);
    builder.addDoubleProperty("Arm kD", armPID::getD, armPID::setD);
    builder.addDoubleProperty("Arm Position", armEncoder::getPosition, armEncoder::setPosition);
    builder.addBooleanProperty("Done Scoring", this::doneScoring, null);
    builder.addBooleanProperty("See Game Piece", this::seeGamePiece, null);
  }

  /**
   * takes the column and height from ScoringLocationUtil.java and converts that to a LiftPosition
   * then gives the position to the given lift
   */
  public void ManualPrepScoreSequence() {
    ScoreHeight height = scoreLoc.getScoreHeight();

    // low for all columns is the same height
    if (height == ScoreHeight.LOW) {
      setDesiredPosition(LiftPosition.SCORE_LOW);
    }
    // left and right columns are for cones
    else if (scoreLoc.isCone()) {
      if (height == ScoreHeight.MID) {
        setDesiredPosition(LiftPosition.SCORE_MID_CONE);
      } else {
        setDesiredPosition(LiftPosition.SCORE_HIGH_CONE);
      }
    }
    // middle columns are for cubes
    else {
      if (height == ScoreHeight.MID) {
        setDesiredPosition(LiftPosition.SCORE_MID_CUBE);
      } else {
        setDesiredPosition(LiftPosition.SCORE_HIGH_CUBE);
      }
    }
  }
}
