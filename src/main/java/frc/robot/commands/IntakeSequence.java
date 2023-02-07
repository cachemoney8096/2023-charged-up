package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Cal;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.LiftPosition;

public class IntakeSequence extends SequentialCommandGroup {
  private Intake intake;
  private Lift lift;

  // deploy intake for specified amount of time
  private ParallelRaceGroup deployIntake =
      new RunCommand(intake::deploy, intake).withTimeout(Cal.Intake.AUTO_CLAMP_WAIT_TIME_SECONDS);

  // run intake, move lift to intake position, and open the grabber, until the robot sees an object
  // and the lift is in position
  private RunCommand runIntake = new RunCommand(intake::intakeGamePiece, intake);
  private InstantCommand liftToIntakePos =
      new InstantCommand(() -> lift.setDesiredPosition(LiftPosition.GRAB_FROM_INTAKE), lift);
  private InstantCommand openGrabber = new InstantCommand(lift::openGrabber, lift);
  private ParallelRaceGroup intakePrep =
      new ParallelCommandGroup(runIntake, liftToIntakePos, openGrabber).until(lift::readyToIntake);

  private InstantCommand closeGrabber = new InstantCommand(lift::closeGrabber, lift);
  
  private InstantCommand unclampIntake = new InstantCommand(intake::unclampIntake, intake);

  // this does not need a timeout even though it is a longer action, because it is the final action
  // in the sequence
  private InstantCommand liftToStartPos =
      new InstantCommand(() -> lift.setDesiredPosition(LiftPosition.STARTING), lift);

  public IntakeSequence(Intake intake, Lift lift) {
    this.intake = intake;
    this.lift = lift;
    addCommands(deployIntake, intakePrep, closeGrabber, unclampIntake, liftToStartPos);
  }
}
