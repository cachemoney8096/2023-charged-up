package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Cal;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.LiftPosition;

public class IntakeSequence extends SequentialCommandGroup {
  private Intake intake;
  private Lift lift;

  public IntakeSequence(Intake intake, Lift lift) {
    this.intake = intake;
    this.lift = lift;
    // addCommands(deployIntake, intakePrep, stopIntake, closeGrabber, unclampIntake,
    // liftToStartPos);
    addCommands(
        // deploy intake for specified amount of time
        new RunCommand(intake::deploy, intake).until(intake::atDesiredPosition),

        // run intake, move lift to intake position, and open the grabber, until the robot sees an
        // object
        // and the lift is in position
        new ParallelCommandGroup(

                // run intake
                new InstantCommand(intake::intakeGamePiece, intake),

                // trigger the lift to move to the intake position. This does not need a timeout
                // because it is
                // running in the parallel group, which is controlled by readyToIntake()
                new InstantCommand(
                    () -> lift.setDesiredPosition(LiftPosition.GRAB_FROM_INTAKE), lift),

                // triggers the grabber to open when it is safe. Again, we do not care about time
                // because it is in
                // the parallel group.
                new InstantCommand(lift::openGrabber, lift))
            .andThen(new WaitUntilCommand(() -> (lift.atIntakePos() && intake.seeGamePiece()))),
        // stop intake
        new InstantCommand(intake::stopIntakingGamePiece, intake),

        // triggers the grabber to close
        new InstantCommand(lift::closeGrabber, lift)
            .andThen(new WaitCommand(Cal.Lift.GRABBER_CLOSE_TIME_SECONDS)),

        // immediately unclamps the intake.
        new InstantCommand(intake::unclampIntake, intake)
            .andThen(new WaitCommand(Cal.Intake.UNCLAMP_TIME_SECONDS)),

        // triggers the lift to move to the starting position. This does not need a timeout even
        // though it
        // is a longer action, because it is the final action in the sequence
        new InstantCommand(() -> lift.setDesiredPosition(LiftPosition.STARTING), lift));
  }
}
