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
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Cal;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.utils.AbsoluteEncoderChecker;
import frc.robot.utils.AngleUtil;
import frc.robot.utils.ScoringLocationUtil;
import frc.robot.utils.ScoringLocationUtil.ScoreHeight;
import frc.robot.utils.SendableHelper;
import frc.robot.utils.SparkMaxUtils;
import java.util.Arrays;
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
    STARTING,
    ALT_HOME,
    BOOT_UP
  }

  /** Position of the lift relative to to the start position */
  public enum LiftPositionStartRelative {
    BELOW_START,
    AT_START,
    ABOVE_START
  }

  // Actuators
  public CANSparkMax elevatorLeft =
      new CANSparkMax(RobotMap.ELEVATOR_MOTOR_LEFT_CAN_ID, MotorType.kBrushless);
  public CANSparkMax elevatorRight =
      new CANSparkMax(RobotMap.ELEVATOR_MOTOR_RIGHT_CAN_ID, MotorType.kBrushless);

  /** Input in, output Volts */
  private ProfiledPIDController elevatorController =
      new ProfiledPIDController(
          Cal.Lift.ELEVATOR_P,
          Cal.Lift.ELEVATOR_I,
          Cal.Lift.ELEVATOR_D,
          new TrapezoidProfile.Constraints(
              Cal.Lift.ELEVATOR_MAX_VELOCITY_IN_PER_SECOND,
              Cal.Lift.ELEVATOR_MAX_ACCELERATION_IN_PER_SECOND_SQUARED));
  /** Input deg, output Volts */
  private ProfiledPIDController armController =
      new ProfiledPIDController(
          Cal.Lift.ARM_P,
          Cal.Lift.ARM_I,
          Cal.Lift.ARM_D,
          new TrapezoidProfile.Constraints(
              Cal.Lift.ARM_MAX_VELOCITY_DEG_PER_SECOND,
              Cal.Lift.ARM_MAX_ACCELERATION_DEG_PER_SECOND_SQUARED));

  public CANSparkMax armMotor = new CANSparkMax(RobotMap.ARM_MOTOR_CAN_ID, MotorType.kBrushless);
  private Solenoid grabber =
      new Solenoid(PneumaticsModuleType.REVPH, RobotMap.LIFT_GRABBING_CHANNEL);

  // Sensors
  private final RelativeEncoder elevatorLeftEncoder = elevatorLeft.getEncoder();
  private final AbsoluteEncoder elevatorLeftAbsEncoder =
      elevatorLeft.getAbsoluteEncoder(Type.kDutyCycle);
  private final AbsoluteEncoder elevatorRightAbsEncoder =
      elevatorRight.getAbsoluteEncoder(Type.kDutyCycle);
  private final RelativeEncoder armEncoder = armMotor.getEncoder();
  private final AbsoluteEncoder armAbsoluteEncoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);
  private final DigitalInput gamePieceSensor = new DigitalInput(RobotMap.LIFT_GAME_PIECE_DIO);

  // Members
  private LiftPosition latestPosition = LiftPosition.ALT_HOME;
  private LiftPosition desiredPosition = LiftPosition.ALT_HOME;
  private LiftPosition goalPosition = LiftPosition.ALT_HOME;
  private boolean desiredGrabberClosed = true;
  public ScoringLocationUtil scoreLoc;
  private boolean scoringInProgress = false;
  private AbsoluteEncoderChecker elevatorLeftAbsEncoderChecker = new AbsoluteEncoderChecker();
  private AbsoluteEncoderChecker elevatorRightAbsEncoderChecker = new AbsoluteEncoderChecker();
  private AbsoluteEncoderChecker armAbsoluteEncoderChecker = new AbsoluteEncoderChecker();
  private LiftPosition[] posToReprep =
      new LiftPosition[] {
        LiftPosition.PRE_SCORE_HIGH_CONE,
        LiftPosition.PRE_SCORE_MID_CONE,
        LiftPosition.SCORE_HIGH_CUBE,
        LiftPosition.SCORE_MID_CUBE,
        LiftPosition.SCORE_LOW
      };
  /**
   * Indicates the elevator and arm positions at each position of the lift. The first value
   * indicates the elevator position in inches and the second value indicates the arm position in
   * degrees
   */
  TreeMap<LiftPosition, Pair<Double, Double>> liftPositionMap;

  /**
   * Passed into finishScore to see if the scoring action was cancelled (true if scoring action is
   * cancelled)
   */
  private boolean cancelScore = false;

  private Command rumbleBriefly;
  private boolean sawObject = true;

  /** Creates a new Lift */
  public Lift(ScoringLocationUtil scoreLoc, Command rumbleBrieflyCmd) {
    rumbleBriefly = rumbleBrieflyCmd;
    SparkMaxUtils.initWithRetry(this::initSparks, Cal.SPARK_INIT_RETRY_ATTEMPTS);

    // Map of all LiftPosition with according values
    liftPositionMap = new TreeMap<LiftPosition, Pair<Double, Double>>();
    liftPositionMap.put(
        LiftPosition.GRAB_FROM_INTAKE,
        new Pair<Double, Double>(Cal.Lift.ELEVATOR_LOW_POSITION_INCHES, 85.0));
    liftPositionMap.put(
        LiftPosition.SHELF, new Pair<Double, Double>(Cal.Lift.ELEVATOR_LOW_POSITION_INCHES, 190.0));
    liftPositionMap.put(
        LiftPosition.SCORE_LOW,
        new Pair<Double, Double>(Cal.Lift.ELEVATOR_LOW_POSITION_INCHES, 187.0));
    liftPositionMap.put(
        LiftPosition.SCORE_MID_CUBE,
        new Pair<Double, Double>(Cal.Lift.ELEVATOR_LOW_POSITION_INCHES, 216.0));
    liftPositionMap.put(
        LiftPosition.SCORE_MID_CONE,
        new Pair<Double, Double>(Cal.Lift.ELEVATOR_LOW_POSITION_INCHES, 213.0));
    liftPositionMap.put(
        LiftPosition.SCORE_HIGH_CUBE,
        new Pair<Double, Double>(Cal.Lift.ELEVATOR_HIGH_POSITION_INCHES, 216.0));
    liftPositionMap.put(
        LiftPosition.SCORE_HIGH_CONE,
        new Pair<Double, Double>(Cal.Lift.ELEVATOR_HIGH_POSITION_INCHES, 213.0));
    liftPositionMap.put(
        LiftPosition.PRE_SCORE_MID_CONE,
        new Pair<Double, Double>(Cal.Lift.ELEVATOR_LOW_POSITION_INCHES, 193.0));
    liftPositionMap.put(
        LiftPosition.PRE_SCORE_HIGH_CONE,
        new Pair<Double, Double>(Cal.Lift.ELEVATOR_HIGH_POSITION_INCHES, 193.0));
    liftPositionMap.put(
        LiftPosition.POST_SCORE_HIGH,
        new Pair<Double, Double>(Cal.Lift.ELEVATOR_HIGH_POSITION_INCHES, 180.0));
    liftPositionMap.put(
        LiftPosition.OUTTAKING,
        new Pair<Double, Double>(Cal.Lift.ELEVATOR_LOW_POSITION_INCHES, 183.0));
    liftPositionMap.put(
        LiftPosition.STARTING,
        new Pair<Double, Double>(Cal.Lift.ELEVATOR_LOW_POSITION_INCHES, 153.0));
    liftPositionMap.put(
        LiftPosition.ALT_HOME,
        new Pair<Double, Double>(Cal.Lift.ELEVATOR_LOW_POSITION_INCHES, 148.0));
    liftPositionMap.put(
        LiftPosition.BOOT_UP,
        new Pair<Double, Double>(Cal.Lift.ELEVATOR_LOW_POSITION_INCHES, 148.0));

    this.scoreLoc = scoreLoc;
  }

  /** Does all the initialization for the sparks, return true on success */
  private boolean initSparks() {
    int errors = 0;

    errors += SparkMaxUtils.check(elevatorLeft.restoreFactoryDefaults());
    errors += SparkMaxUtils.check(elevatorRight.restoreFactoryDefaults());
    errors += SparkMaxUtils.check(armMotor.restoreFactoryDefaults());

    errors += SparkMaxUtils.check(elevatorRight.follow(elevatorLeft, true));

    // inverting stuff
    errors += SparkMaxUtils.check(armAbsoluteEncoder.setInverted(true));
    errors += SparkMaxUtils.check(elevatorLeftAbsEncoder.setInverted(false));
    armMotor.setInverted(false);

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
            armMotor.setSoftLimit(
                SoftLimitDirection.kForward, Cal.Lift.ARM_POSITIVE_LIMIT_DEGREES));
    errors += SparkMaxUtils.check(armMotor.enableSoftLimit(SoftLimitDirection.kForward, true));
    errors +=
        SparkMaxUtils.check(
            armMotor.setSoftLimit(
                SoftLimitDirection.kReverse, Cal.Lift.ARM_NEGATIVE_LIMIT_DEGREES));
    errors += SparkMaxUtils.check(armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true));

    errors += SparkMaxUtils.check(armMotor.setIdleMode(IdleMode.kBrake));
    errors += SparkMaxUtils.check(elevatorLeft.setIdleMode(IdleMode.kBrake));
    errors += SparkMaxUtils.check(elevatorRight.setIdleMode(IdleMode.kBrake));

    errors +=
        SparkMaxUtils.check(
            elevatorLeft.setSmartCurrentLimit(Cal.Lift.ELEVATOR_CURRENT_LIMIT_AMPS));
    errors +=
        SparkMaxUtils.check(
            elevatorRight.setSmartCurrentLimit(Cal.Lift.ELEVATOR_CURRENT_LIMIT_AMPS));
    errors += SparkMaxUtils.check(armMotor.setSmartCurrentLimit(Cal.Lift.ARM_CURRENT_LIMIT_AMPS));

    return errors == 0;
  }

  public void deployArmLessFar() {
    Pair<Double, Double> curPos = liftPositionMap.get(desiredPosition);
    Pair<Double, Double> newPos =
        new Pair<Double, Double>(curPos.getFirst(), curPos.getSecond() - 0.5);
    liftPositionMap.replace(desiredPosition, newPos);

    System.out.println("Latest angle for " + desiredPosition + ": " + (curPos.getSecond() - 0.5));

    armController.setGoal(newPos.getSecond());
  }

  public void deployArmFurther() {
    Pair<Double, Double> curPos = liftPositionMap.get(desiredPosition);
    Pair<Double, Double> newPos =
        new Pair<Double, Double>(curPos.getFirst(), curPos.getSecond() + 0.5);
    liftPositionMap.replace(desiredPosition, newPos);

    System.out.println("Latest angle for " + desiredPosition + ": " + (curPos.getSecond() + 0.5));

    armController.setGoal(newPos.getSecond());
  }

  /**
   * Burns the current settings to sparks so they keep current settings on reboot. Should be done
   * after all settings are set.
   */
  public void burnFlashSparks() {
    Timer.delay(0.005);
    elevatorLeft.burnFlash();
    Timer.delay(0.005);
    armMotor.burnFlash();
  }

  /** Sends voltage commands to the arm and elevator motors, needs to be called every cycle */
  private void controlPosition(LiftPosition pos) {
    if (goalPosition != pos) {
      goalPosition = pos;
      elevatorController.setGoal(liftPositionMap.get(pos).getFirst());
    }
    armController.setGoal(liftPositionMap.get(pos).getSecond());

    double elevatorDemandVolts = elevatorController.calculate(elevatorLeftEncoder.getPosition());
    elevatorDemandVolts +=
        Cal.Lift.ELEVATOR_FEEDFORWARD.calculate(elevatorController.getSetpoint().velocity);
    elevatorLeft.setVoltage(elevatorDemandVolts);

    double armDemandVoltsA = armController.calculate(armEncoder.getPosition());
    double armDemandVoltsB =
        Cal.Lift.ARM_FEEDFORWARD.calculate(armController.getSetpoint().velocity);
    double armDemandVoltsC = Cal.Lift.ARBITRARY_ARM_FEED_FORWARD_VOLTS * getCosineArmAngle();
    armMotor.setVoltage(armDemandVoltsA + armDemandVoltsB + armDemandVoltsC);

    SmartDashboard.putNumber("Arm PID", armDemandVoltsA);
    SmartDashboard.putNumber("Arm FF", armDemandVoltsB);
    SmartDashboard.putNumber("Arm Gravity", armDemandVoltsC);
  }

  public void closeGrabber() {
    desiredGrabberClosed = true;
  }

  public void openGrabber() {
    desiredGrabberClosed = false;
  }

  /** If true, we want the robot to drive slower so the driver can line up to score */
  public boolean throttleForLift() {
    switch (desiredPosition) {
      case ALT_HOME:
      case STARTING:
      case OUTTAKING:
      case GRAB_FROM_INTAKE: // note: this is already throttling in the intake sequence button
        // trigger
        return false;
      case SCORE_MID_CUBE:
      case SCORE_MID_CONE:
      case SCORE_HIGH_CUBE:
      case SCORE_HIGH_CONE:
      case SCORE_LOW:
      case PRE_SCORE_MID_CONE:
      case PRE_SCORE_HIGH_CONE:
      case POST_SCORE_HIGH:
      case SHELF:
        return true;
      default:
        return false;
    }
  }

  /** Returns true if the game piece sensor sees a game piece */
  public boolean seeGamePiece() {
    // Sensor is false if there's a game piece
    boolean seeGamePieceNow = !gamePieceSensor.get();
    if (!sawObject && seeGamePieceNow) {
      rumbleBriefly.schedule();
    }

    sawObject = seeGamePieceNow;
    return seeGamePieceNow;
    // return false;
  }

  /** Returns the cosine of the arm angle in degrees off of the horizontal. */
  public double getCosineArmAngle() {
    return Math.cos(
        Units.degreesToRadians(
            armEncoder.getPosition() - Constants.Lift.ARM_POSITION_WHEN_HORIZONTAL_DEGREES));
  }

  public void initialize() {
    rezeroLift();
    liftPositionMap.put(
        LiftPosition.BOOT_UP,
        new Pair<Double, Double>(elevatorLeftEncoder.getPosition(), armEncoder.getPosition()));
  }

  public void rezeroLift() {
    // Set arm encoder position from absolute
    armEncoder.setPosition(
        AngleUtil.wrapAngle(
            armAbsoluteEncoderChecker.getMedian() - Cal.Lift.ARM_ABSOLUTE_ENCODER_ZERO_POS_DEG));

    // Set elevator encoder position from absolute encoders
    double elevatorDutyCycleEncodersDifferenceDegrees =
        AngleUtil.wrapAngleAroundZero(
            (elevatorLeftAbsEncoderChecker.getMedian()
                - elevatorRightAbsEncoderChecker.getMedian()));
    elevatorLeftEncoder.setPosition(
        (elevatorDutyCycleEncodersDifferenceDegrees
                * Constants.Lift.ELEVATOR_MOTOR_ENCODER_DIFFERENCES_SCALAR_INCHES_PER_DEGREE)
            - Cal.Lift.ELEVATOR_ABS_ENCODER_POS_AT_START_INCHES);

    armController.setTolerance(Cal.Lift.ARM_ALLOWED_CLOSED_LOOP_ERROR_DEG);
    armController.reset(armEncoder.getPosition());
    armController.setGoal(armEncoder.getPosition());
    elevatorController.setTolerance(Cal.Lift.ELEVATOR_ALLOWED_CLOSED_LOOP_ERROR_IN);
    elevatorController.reset(elevatorLeftEncoder.getPosition());
    elevatorController.setGoal(elevatorLeftEncoder.getPosition());
  }

  public void zeroArmAtCurrentPos() {
    Cal.Lift.ARM_ABSOLUTE_ENCODER_ZERO_POS_DEG = armAbsoluteEncoder.getPosition();
    System.out.println("New Zero for Arm: " + Cal.Lift.ARM_ABSOLUTE_ENCODER_ZERO_POS_DEG);
  }

  public void zeroElevatorAtCurrentPos() {
    // Set elevator encoder position from absolute encoders
    double elevatorDutyCycleEncodersDifferenceDegrees =
        AngleUtil.wrapAngleAroundZero(
            (elevatorLeftAbsEncoder.getPosition() - elevatorRightAbsEncoder.getPosition()));
    Cal.Lift.ELEVATOR_ABS_ENCODER_POS_AT_START_INCHES =
        (elevatorDutyCycleEncodersDifferenceDegrees
            * Constants.Lift.ELEVATOR_MOTOR_ENCODER_DIFFERENCES_SCALAR_INCHES_PER_DEGREE);
    System.out.println(
        "New Zero for Elevator: " + Cal.Lift.ELEVATOR_ABS_ENCODER_POS_AT_START_INCHES);
  }

  /**
   * if the robot has completed startScore but hasn't started finishScore, then stop the robot from
   * scoring
   */
  public void cancelScore() {
    if (scoringInProgress) {
      setCancelScore(true);
    }
  }

  /** Runs instead of finishScore if cancelScore is true. */
  public void finishScoreCancelled(Lights lights) {
    setCancelScore(false);
    ManualPrepScoreSequence(lights);
  }

  /** returns cancelScore (true if scoring action is cancelled) */
  public boolean getCancelScore() {
    return this.cancelScore;
  }

  /** sets cancelScore (true if scoring action is cancelled) */
  public void setCancelScore(boolean cancelled) {
    this.cancelScore = cancelled;
  }

  /** sets scoringInProgress */
  public void setScoringInProgress(boolean isScoring) {
    scoringInProgress = isScoring;
  }

  public void home() {
    setDesiredPosition(LiftPosition.STARTING);
  }

  public void startScore() {
    scoringInProgress = true;
    if (scoreLoc.isCone()) {
      if (scoreLoc.getScoreHeight() == ScoreHeight.HIGH) {
        setDesiredPosition(LiftPosition.SCORE_HIGH_CONE);
      } else if (scoreLoc.getScoreHeight() == ScoreHeight.MID) {
        setDesiredPosition(LiftPosition.SCORE_MID_CONE);
      }
    }
  }

  /** Sets the desired position, which the lift may not go to directly. */
  public void setDesiredPosition(LiftPosition pos) {
    desiredPosition = pos;
  }

  private boolean atDesiredArmPosition() {
    double armMarginDegrees =
        desiredPosition == LiftPosition.STARTING
            ? Cal.Lift.ARM_START_MARGIN_DEGREES
            : Cal.Lift.ARM_MARGIN_DEGREES;
    double armPositionToCheckDegrees = liftPositionMap.get(desiredPosition).getSecond();
    double armPositionDegrees = armEncoder.getPosition();
    if (Math.abs(armPositionDegrees - armPositionToCheckDegrees) > armMarginDegrees) {
      return false;
    }
    return true;
  }

  public boolean atDesiredElevatorPosition() {
    double elevatorMarginInches =
        desiredPosition == LiftPosition.STARTING
            ? Cal.Lift.ELEVATOR_START_MARGIN_INCHES
            : Cal.Lift.ELEVATOR_MARGIN_INCHES;
    double elevatorPositionToCheckInches = liftPositionMap.get(desiredPosition).getFirst();
    double elevatorPositionInches = elevatorLeftEncoder.getPosition();

    if (Math.abs(elevatorPositionInches - elevatorPositionToCheckInches) > elevatorMarginInches) {
      return false;
    }

    return true;
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
      case ALT_HOME:
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
    // LiftPositionStartRelative latestPositionStartRelative =
    // getRelativeLiftPosition(latestPosition);
    // LiftPositionStartRelative desiredPositionStartRelative =
    //     getRelativeLiftPosition(desiredPosition);
    // if (!seeGamePiece()) {
    //   controlPosition(desiredPosition);
    // } else if (desiredPositionStartRelative == LiftPositionStartRelative.BELOW_START
    //     && latestPositionStartRelative == LiftPositionStartRelative.ABOVE_START) {
    //   controlPosition(LiftPosition.STARTING);
    // } else if (desiredPositionStartRelative == LiftPositionStartRelative.ABOVE_START
    //     && latestPositionStartRelative == LiftPositionStartRelative.BELOW_START) {
    //   controlPosition(LiftPosition.STARTING);
    // } else {
    //   controlPosition(desiredPosition);
    // }
    controlPosition(desiredPosition);
    elevatorLeftAbsEncoderChecker.addReading(elevatorLeftAbsEncoder.getPosition());
    elevatorRightAbsEncoderChecker.addReading(elevatorRightAbsEncoder.getPosition());
    armAbsoluteEncoderChecker.addReading(armAbsoluteEncoder.getPosition());

    // // If the grabber is set to open and it is safe to open, open the grabber (drop). Otherwise,
    // // close it (grab).
    // if (!desiredGrabberClosed
    //     && (armEncoder.getPosition() > Cal.Lift.GRABBER_CLOSED_ZONE_TOP_DEGREES
    //         || armEncoder.getPosition() < Cal.Lift.GRABBER_CLOSED_ZONE_BOTTOM_DEGREES)) {
    //   grabber.set(false); // drop
    // } else {
    //   grabber.set(true); // grab
    // }
    // TODO do we need a grabber zone?

    if (desiredGrabberClosed) {
      grabber.set(false); // grab
    } else {
      grabber.set(true); // drop
    }
  }

  /**
   * Returns true if the lift is clear of the zone where the intake moves, so that the intake can
   * move as soon as the lift is clear of that zone
   */
  public boolean clearOfIntakeZone() {
    return armEncoder.getPosition() > Cal.Lift.ARM_INTAKE_ZONE_THRESHOLD_DEGREES;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    SendableHelper.addChild(builder, this, armController, "ArmController");
    SendableHelper.addChild(builder, this, elevatorController, "ElevatorController");
    builder.addDoubleProperty(
        "Elevator Position (in)",
        elevatorLeftEncoder::getPosition,
        elevatorLeftEncoder::setPosition);
    builder.addDoubleProperty("Elevator Vel (in/s)", elevatorLeftEncoder::getVelocity, null);
    builder.addDoubleProperty(
        "Elevator Left Abs Pos (deg)", elevatorLeftAbsEncoder::getPosition, null);
    builder.addDoubleProperty(
        "Elevator Right Abs Pos (deg)", elevatorRightAbsEncoder::getPosition, null);
    builder.addDoubleProperty(
        "Elevator Pos per abs (in)",
        () -> {

          // Set elevator encoder position from absolute encoders
          double elevatorDutyCycleEncodersDifferenceDegrees =
              AngleUtil.wrapAngleAroundZero(
                  (elevatorLeftAbsEncoder.getPosition() - elevatorRightAbsEncoder.getPosition()));
          return (elevatorDutyCycleEncodersDifferenceDegrees
                  * Constants.Lift.ELEVATOR_MOTOR_ENCODER_DIFFERENCES_SCALAR_INCHES_PER_DEGREE)
              - Cal.Lift.ELEVATOR_ABS_ENCODER_POS_AT_START_INCHES;
        },
        null);
    builder.addBooleanProperty("Clear of intake", this::clearOfIntakeZone, null);
    builder.addDoubleProperty(
        "Arm Abs Position (deg)", armAbsoluteEncoder::getPosition, armEncoder::setPosition);
    builder.addDoubleProperty(
        "Arm Position (deg)", armEncoder::getPosition, armEncoder::setPosition);
    builder.addDoubleProperty("Arm Vel (deg/s)", armEncoder::getVelocity, null);
    builder.addBooleanProperty("See Game Piece", this::seeGamePiece, null);
    builder.addBooleanProperty(
        "At desired position",
        () -> {
          return atPosition(desiredPosition);
        },
        null);
    builder.addBooleanProperty("At desired arm position", this::atDesiredArmPosition, null);
    builder.addBooleanProperty(
        "At desired elevator position", this::atDesiredElevatorPosition, null);
    builder.addStringProperty(
        "Desired position",
        () -> {
          return desiredPosition.toString();
        },
        null);
    builder.addStringProperty(
        "Latest position",
        () -> {
          return latestPosition.toString();
        },
        null);
    builder.addStringProperty(
        "Goal position",
        () -> {
          return goalPosition.toString();
        },
        null);
    builder.addDoubleProperty("Arm output", armMotor::get, null);
    builder.addStringProperty(
        "Score Loc Height",
        () -> {
          return scoreLoc.getScoreHeight().toString();
        },
        null);
    builder.addBooleanProperty(
        "Score Loc Cone",
        () -> {
          return scoreLoc.isCone();
        },
        null);
    builder.addStringProperty(
        "Score Loc Col",
        () -> {
          return scoreLoc.getScoreCol().toString();
        },
        null);
    builder.addBooleanProperty(
        "Elevator Left encoder connected", elevatorLeftAbsEncoderChecker::encoderConnected, null);
    builder.addBooleanProperty(
        "Elevator Right encoder connected", elevatorRightAbsEncoderChecker::encoderConnected, null);
    builder.addBooleanProperty(
        "Arm encoder connected", armAbsoluteEncoderChecker::encoderConnected, null);
  }

  /**
   * takes the column and height from ScoringLocationUtil.java and converts that to a LiftPosition
   * then gives the position to the given lift
   */
  public void ManualPrepScoreSequence(Lights lights) {
    ScoreHeight height = scoreLoc.getScoreHeight();

    // low for all columns is the same height
    if (height == ScoreHeight.LOW) {
      setDesiredPosition(LiftPosition.SCORE_LOW);
    }
    // left and right columns are for cones
    else if (scoreLoc.isCone()) {
      if (height == ScoreHeight.MID) {
        setDesiredPosition(LiftPosition.PRE_SCORE_MID_CONE);
      } else {
        setDesiredPosition(LiftPosition.PRE_SCORE_HIGH_CONE);
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

  public void rePrepScoreSequence(Lights lights) {
    boolean isPreScorePos = Arrays.asList(posToReprep).contains(desiredPosition);
    if (isPreScorePos && !scoringInProgress) {
      ManualPrepScoreSequence(lights);
    }
  }
}
