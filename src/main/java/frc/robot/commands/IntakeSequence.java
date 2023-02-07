package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.LiftPosition;

public class IntakeSequence extends SequentialCommandGroup {
  private Intake intake;
  private Lift lift;

  private InstantCommand deployIntake = new InstantCommand(intake::deploy, intake);
  private InstantCommand runIntake = new InstantCommand(intake::intakeGamePiece, intake);
  private InstantCommand liftToIntakePos =
      new InstantCommand(() -> lift.setDesiredPosition(LiftPosition.GRAB_FROM_INTAKE), lift);
  private InstantCommand openGrabber = new InstantCommand(lift::openGrabber, lift);
  private ParallelRaceGroup intakePrep =
      new ParallelRaceGroup(runIntake, liftToIntakePos, openGrabber).until(lift::readyToIntake);
  private InstantCommand closeGrabber = new InstantCommand(lift::closeGrabber, lift);
  private InstantCommand unclampIntake = new InstantCommand(intake::unclampIntake, intake);
  private InstantCommand liftToStartPos =
      new InstantCommand(() -> lift.setDesiredPosition(LiftPosition.STARTING), lift);

  public IntakeSequence(Intake intake, Lift lift) {
    this.intake = intake;
    this.lift = lift;
    addCommands(deployIntake, intakePrep, closeGrabber, unclampIntake, liftToStartPos);
  }
}
