// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoChargeStationSequence;
import frc.robot.commands.AutoScoreAndBalance;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeLimelight;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.TagLimelight;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.JoystickUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Intake intake = new Intake();
  private final DriveSubsystem drive = new DriveSubsystem();
  private final Lift lift = new Lift();
  private final IntakeLimelight intakeLimelight;
  private final TagLimelight tagLimelight;

  // A chooser for autonomous commands
  private SendableChooser<Command> autonChooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(RobotMap.DRIVER_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // set up the limelights
    intakeLimelight =
        new IntakeLimelight(
            Constants.INTAKE_LIMELIGHT_PITCH_DEGREES,
            Constants.INTAKE_LIMELIGHT_HEIGHT_METERS,
            Constants.INTAKE_TARGET_HEIGHT_METERS);

    tagLimelight =
        new TagLimelight(
            Constants.TAG_LIMELIGHT_PITCH_DEGREES,
            Constants.TAG_LIMELIGHT_HEIGHT_METERS,
            Constants.TAG_TARGET_HEIGHT_METERS);
  }

  public void initialize() {
    // autons
    autonChooser.setDefaultOption("Balance", new AutoChargeStationSequence(true, drive));
    autonChooser.addOption("Score, balance", new AutoScoreAndBalance(true, lift, drive));

    // Put the chooser on the dashboard
    SmartDashboard.putData(autonChooser);

    // Encoder offset stuff
    intake.initialize();
    lift.initialize();
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
    driverController.a().whileTrue(new InstantCommand(intake::deploy, intake));
    // Drive controls
    drive.setDefaultCommand(
        new RunCommand(
                () ->
                    drive.rotateOrKeepHeading(
                        MathUtil.applyDeadband(-driverController.getLeftY(), 0.1),
                        MathUtil.applyDeadband(-driverController.getLeftX(), 0.1),
                        JoystickUtil.squareAxis(
                            MathUtil.applyDeadband(-driverController.getRightX(), 0.1)),
                        driverController.getHID().getLeftBumper(),
                        driverController.getHID().getPOV()),
                drive)
            .withName("Manual Drive"));
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
