// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeSequence;
import frc.robot.commands.OuttakeSequence;
import frc.robot.commands.ShelfSequence;
import frc.robot.commands.autos.AutoScoreAndBalance;
import frc.robot.commands.autos.AutoScoreMobilityAndBalance;
import frc.robot.commands.autos.JustTwoGamePieces;
import frc.robot.commands.autos.OneFiveBalanceBump;
import frc.robot.commands.autos.OneFiveBalanceCenter;
import frc.robot.commands.autos.OneFiveBumpReturn;
import frc.robot.commands.autos.TwoGamePiecesThatEngage;
import frc.robot.commands.finishScore;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeLimelight;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.LiftPosition;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.TagLimelightV2;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.JoystickUtil;
import frc.robot.utils.ScoringLocationUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final ScoringLocationUtil scoreLoc = new ScoringLocationUtil();
  public final Lift lift = new Lift(scoreLoc);
  private final DriveSubsystem drive = new DriveSubsystem(lift::throttleForLift);
  public final Intake intake = new Intake(lift::clearOfIntakeZone);
  private final IntakeLimelight intakeLimelight =
      new IntakeLimelight(
          Constants.INTAKE_LIMELIGHT_PITCH_DEGREES,
          Constants.INTAKE_LIMELIGHT_HEIGHT_METERS,
          Constants.INTAKE_TARGET_HEIGHT_METERS);
  public final TagLimelightV2 tagLimelight = new TagLimelightV2(scoreLoc);
  private final Lights lights = new Lights();
  public final PneumaticHub pneumaticHub = new PneumaticHub();

  // A chooser for autonomous commands
  private SendableChooser<Command> autonChooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(RobotMap.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController =
      new CommandXboxController(RobotMap.OPERATOR_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    Shuffleboard.getTab("Subsystems").add(drive.getName(), drive);
    Shuffleboard.getTab("Subsystems").add(intake.getName(), intake);
    Shuffleboard.getTab("Subsystems").add(intakeLimelight.getName(), intakeLimelight);
    Shuffleboard.getTab("Subsystems").add(tagLimelight.getName(), tagLimelight);
    // Shuffleboard.getTab("Subsystems").add(lights.getName(), lights);
    Shuffleboard.getTab("Subsystems").add(lift.getName(), lift);
  }

  public void initialize() {
    // autons
    autonChooser.setDefaultOption(
        "Two plus balance Blue",
        new TwoGamePiecesThatEngage(false, lift, intake, drive, lights, tagLimelight, scoreLoc));
    autonChooser.addOption(
        "Two plus balance Red",
        new TwoGamePiecesThatEngage(true, lift, intake, drive, lights, tagLimelight, scoreLoc));
    autonChooser.addOption(
        "One plus balance", new AutoScoreAndBalance(lift, drive, lights, scoreLoc));
    autonChooser.addOption(
        "One plus mobility plus balance",
        new AutoScoreMobilityAndBalance(lift, drive, lights, scoreLoc));
    autonChooser.addOption(
        "Just two blue",
        new JustTwoGamePieces(false, lift, intake, drive, lights, tagLimelight, scoreLoc));
    autonChooser.addOption(
        "Just two red",
        new JustTwoGamePieces(true, lift, intake, drive, lights, tagLimelight, scoreLoc));
    autonChooser.addOption(
        "1.5 balance bump blue",
        new OneFiveBalanceBump(
            false, false, lift, intake, drive, lights, intakeLimelight, scoreLoc));
    autonChooser.addOption(
        "1.5 balance bump red",
        new OneFiveBalanceBump(
            true, false, lift, intake, drive, lights, intakeLimelight, scoreLoc));
    autonChooser.addOption(
        "1.5 return bump blue",
        new OneFiveBumpReturn(
            false, false, lift, intake, drive, lights, intakeLimelight, scoreLoc));
    autonChooser.addOption(
        "1.5 return bump red",
        new OneFiveBumpReturn(
            true, false, lift, intake, drive, lights, intakeLimelight, scoreLoc));
    autonChooser.addOption(
        "1.5 balance center",
        new OneFiveBalanceCenter(lift, drive, lights, scoreLoc, intakeLimelight, intake));

    // Put the chooser on the dashboard
    SmartDashboard.putData(autonChooser);

    // Encoder offset stuff
    intake.initialize();
    lift.initialize();

    pneumaticHub.enableCompressorAnalog(90, 110);

    burnFlashSparks();
  }

  /**
   * Run burnFlash() for all controllers initialized. The ideal use case for this call is to call it
   * once everything has been initialized. The burnFlash() call has the side effect of preventing
   * all communication *to* the device for up to 200ms or more, potentially including some messages
   * called before the burnFlash() call, and receiving messages *from* the device.
   *
   * <p>WARNING: This call will sleep the thread before and after burning flash. This is for your
   * safety.
   *
   * <p>Borrowed from 3005.
   */
  public void burnFlashSparks() {
    Timer.delay(0.25);
    lift.burnFlashSparks();
    intake.burnFlashSparks();
    drive.burnFlashSparks();
    Timer.delay(0.25);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driverController
        .b()
        .whileTrue(
            new OuttakeSequence(lift, lights)
                .finallyDo(
                    (boolean interrupted) -> {
                      lift.home();
                    }));
    driverController.a().onTrue(new InstantCommand(lift::cancelScore, lift));
    driverController
        .x()
        .onTrue(
            new InstantCommand(
                () -> {
                  drive.throttle(Cal.SwerveSubsystem.THROTTLE_FOR_SLOW_BUTTON);
                }));
    driverController
        .x()
        .onFalse(
            new InstantCommand(
                () -> {
                  drive.throttle(1.0);
                }));
    driverController.y().whileTrue(new RunCommand(drive::setX, drive));

    driverController
        .rightBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  lift.ManualPrepScoreSequence(lights);
                },
                lift));

    driverController.back().onTrue(new InstantCommand(lift::home, lift));

    driverController.start().onTrue(new InstantCommand(drive::resetYaw).ignoringDisable(true));

    driverController.leftBumper().onTrue(new ShelfSequence(lift, lights));
    driverController
        .leftBumper()
        .onFalse(
            new SequentialCommandGroup(
                new InstantCommand(lift::closeGrabber),
                new WaitCommand(0.2),
                new InstantCommand(
                    () -> {
                      lift.setDesiredPosition(LiftPosition.STARTING);
                    },
                    lift)));

    driverController
        .leftTrigger()
        .whileTrue(
            new IntakeSequence(intake, lift, lights)
                .beforeStarting(
                    new InstantCommand(
                        () -> {
                          drive.throttle(Cal.SwerveSubsystem.THROTTLE_FOR_INTAKING);
                        }))
                .finallyDo(
                    (boolean interrupted) -> {
                      drive.throttle(1.0);
                      lift.closeGrabber();
                      lift.home();
                      intake.stopIntakingGamePiece();
                      intake.setDesiredDeployed(false);
                      intake.setDesiredClamped(false);
                    }));
    driverController.rightTrigger().onTrue(new InstantCommand(lift::startScore, lift));
    driverController
        .rightTrigger()
        .onFalse(
            new ConditionalCommand(
                new InstantCommand(
                    () -> {
                      lift.finishScoreCancelled(lights);
                    },
                    lift),
                new finishScore(lift, lights),
                lift::getCancelScore));

    operatorController
        .povDown()
        .onTrue(
            new InstantCommand(() -> scoreLoc.setScoreHeight(ScoringLocationUtil.ScoreHeight.LOW))
                .ignoringDisable(true));
    operatorController
        .povRight()
        .onTrue(
            new InstantCommand(() -> scoreLoc.setScoreHeight(ScoringLocationUtil.ScoreHeight.MID))
                .ignoringDisable(true));
    operatorController
        .povLeft()
        .onTrue(
            new InstantCommand(() -> scoreLoc.setScoreHeight(ScoringLocationUtil.ScoreHeight.MID))
                .ignoringDisable(true));
    operatorController
        .povUp()
        .onTrue(
            new InstantCommand(() -> scoreLoc.setScoreHeight(ScoringLocationUtil.ScoreHeight.HIGH))
                .ignoringDisable(true));

    operatorController
        .x()
        .onTrue(
            new InstantCommand(() -> scoreLoc.setScoreCol(ScoringLocationUtil.ScoreCol.LEFT))
                .ignoringDisable(true));
    operatorController
        .a()
        .onTrue(
            new InstantCommand(() -> scoreLoc.setScoreCol(ScoringLocationUtil.ScoreCol.CENTER))
                .ignoringDisable(true));
    operatorController
        .y()
        .onTrue(
            new InstantCommand(() -> scoreLoc.setScoreCol(ScoringLocationUtil.ScoreCol.CENTER))
                .ignoringDisable(true));
    operatorController
        .b()
        .onTrue(
            new InstantCommand(() -> scoreLoc.setScoreCol(ScoringLocationUtil.ScoreCol.RIGHT))
                .ignoringDisable(true));

    operatorController
        .leftTrigger()
        .onTrue(
            new InstantCommand(
                () -> {
                  lights.toggleCode(Lights.LightCode.CONE);
                }));
    operatorController
        .rightTrigger()
        .onTrue(new InstantCommand(() -> lights.toggleCode(Lights.LightCode.CUBE), lights));
    operatorController
        .leftBumper()
        .onTrue(new InstantCommand(lift::deployArmLessFar).ignoringDisable(true));
    operatorController
        .rightBumper()
        .onTrue(new InstantCommand(lift::deployArmFurther).ignoringDisable(true));

    drive.setDefaultCommand(
        new RunCommand(
                () ->
                    drive.rotateOrKeepHeading(
                        MathUtil.applyDeadband(-driverController.getRightY(), 0.1),
                        MathUtil.applyDeadband(-driverController.getRightX(), 0.1),
                        JoystickUtil.squareAxis(
                            MathUtil.applyDeadband(-driverController.getLeftX(), 0.05)),
                        !driverController.getHID().getLeftBumper(),
                        driverController.getHID().getPOV()),
                drive)
            .withName("Manual Drive"));

    intake.setDefaultCommand(
        new InstantCommand(
            () -> {
              intake.setDesiredDeployed(false);
            },
            intake));
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
