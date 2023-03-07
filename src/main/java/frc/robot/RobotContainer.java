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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoChargeStationSequence;
import frc.robot.commands.AutoScoreAndBalance;
import frc.robot.commands.LookForTag;
import frc.robot.commands.IntakeSequence;
import frc.robot.commands.OuttakeSequence;
import frc.robot.commands.TwoGamePiecesThatEngage;
import frc.robot.commands.finishScore;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeLimelight;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.TagLimelightV2;
import frc.robot.subsystems.Lift.LiftPosition;
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
  private final DriveSubsystem drive = new DriveSubsystem();
  private final ScoringLocationUtil scoreLoc = new ScoringLocationUtil();
  public final Lift lift = new Lift(scoreLoc);
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
    // Shuffleboard.getTab("Subsystems").add(intake.getName(), intake);
    // Shuffleboard.getTab("Subsystems").add(intakeLimelight.getName(), intakeLimelight);
    Shuffleboard.getTab("Subsystems").add(tagLimelight.getName(), tagLimelight);
    // Shuffleboard.getTab("Subsystems").add(lights.getName(), lights);
    Shuffleboard.getTab("Subsystems").add(lift.getName(), lift);
  }

  public void initialize() {
    // autons

    autonChooser.setDefaultOption(
            // "One plus balance", new AutoScoreAndBalance(lift, drive, lights, scoreLoc));
        "Two plus balance Blue",
        new TwoGamePiecesThatEngage(false, lift, intake, drive, lights, tagLimelight, scoreLoc));
    autonChooser.addOption(
        "Score, balance",
        new AutoScoreAndBalance(lift, drive, lights, scoreLoc));
    autonChooser.addOption(
        "Two plus balance Red",
        new TwoGamePiecesThatEngage(true, lift, intake, drive, lights, tagLimelight, scoreLoc));
        
    // Put the chooser on the dashboard
    SmartDashboard.putData(autonChooser);

    // Encoder offset stuff
    intake.initialize();
    lift.initialize();

    pneumaticHub.enableCompressorAnalog(80, 110);

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
    driverController.b().whileTrue(new OuttakeSequence(lift).finallyDo(
                      (boolean interrupted) -> {
                        lift.home();
                      }));
    driverController.x().onTrue(new InstantCommand(lift::cancelScore, lift));

    driverController.rightBumper().onTrue(new InstantCommand(() -> {lift.ManualPrepScoreSequence(lights);}, lift));

    driverController.back().onTrue(new InstantCommand(lift::home, lift));

    driverController
    .start()
    .onTrue(
        new InstantCommand(drive::resetYaw));

    driverController.leftBumper().onTrue(
        new InstantCommand(() -> {lift.setDesiredPosition(LiftPosition.SHELF);})
        .andThen(new InstantCommand(lift::openGrabber))
    );
    driverController.leftBumper().onTrue(
        new InstantCommand(() -> {lift.setDesiredPosition(LiftPosition.STARTING);})
        .andThen(new InstantCommand(lift::closeGrabber))
    );

    // TODO Maybe: steal
    // TODO: implement autoscore command for teleop
    // // driverController.rightBumper().onTrue(new InstantCommand(lift::prepScore, lift));
    driverController
        .leftTrigger()
        .whileTrue(
            new IntakeSequence(intake, lift, lights)
            .beforeStarting(new InstantCommand(() -> {drive.throttle(0.75);}))    
            .finallyDo(
                    (boolean interrupted) -> {
                      drive.throttle(1.0);
                      lift.home();
                      intake.stopIntakingGamePiece();
                    }));
    driverController.rightTrigger().onTrue(new InstantCommand(lift::startScore, lift));
    driverController
        .rightTrigger()
        .onFalse(
            new ConditionalCommand(
                new InstantCommand(() -> { lift.finishScoreCancelled(lights); }, lift),
                new finishScore(lift, lights),
                lift::getCancelScore));

    operatorController
        .povDown()
        .onTrue(
            new InstantCommand(() -> scoreLoc.setScoreHeight(ScoringLocationUtil.ScoreHeight.LOW)));
    operatorController
        .povRight()
        .onTrue(
            new InstantCommand(() -> scoreLoc.setScoreHeight(ScoringLocationUtil.ScoreHeight.MID)));
    operatorController
        .povUp()
        .onTrue(
            new InstantCommand(
                () -> scoreLoc.setScoreHeight(ScoringLocationUtil.ScoreHeight.HIGH)));
                // operatorController.povRight().onTrue(new InstantCommand(() -> lights.togglePartyMode()));

    operatorController
        .x()
        .onTrue(new InstantCommand(() -> scoreLoc.setScoreCol(ScoringLocationUtil.ScoreCol.LEFT)));
    operatorController
        .a()
        .onTrue(
            new InstantCommand(() -> scoreLoc.setScoreCol(ScoringLocationUtil.ScoreCol.CENTER)));
    operatorController
        .y()
        .onTrue(
            new InstantCommand(() -> scoreLoc.setScoreCol(ScoringLocationUtil.ScoreCol.CENTER)));
    operatorController
        .b()
        .onTrue(new InstantCommand(() -> scoreLoc.setScoreCol(ScoringLocationUtil.ScoreCol.RIGHT)));

    operatorController
        .start()
        .onTrue(
            new InstantCommand(
                () -> {
                  intake.setDesiredDeployed(true);
                },
                intake));
    operatorController
        .back()
        .onTrue(
            new InstantCommand(
                () -> {
                  intake.setDesiredDeployed(false);
                },
                intake));

    operatorController
        .leftBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  lights.toggleCode(Lights.LightCode.CONE);
                }));
    operatorController
        .rightBumper()
        .onTrue(new InstantCommand(() -> lights.toggleCode(Lights.LightCode.CUBE), lights));
    operatorController.leftTrigger().onTrue(new InstantCommand(lift::openGrabber, lift));
    operatorController.leftTrigger().onFalse(new InstantCommand(lift::closeGrabber, lift));
    operatorController.rightTrigger().onTrue(new InstantCommand(scoreLoc::toggleMiddleGrid));
    operatorController.rightTrigger().onFalse(new InstantCommand(scoreLoc::toggleMiddleGrid));

    // TODO add manual arm and elevator control

    // Drive controls
    // driverController.start().whileTrue(new DriveToTagSimple(tagLimelight, drive));

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
