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

  public IntakeSequence(Intake intake, Lift lift) {
    addRequirements(intake, lift);

    addCommands(
        // deploy intake for specified amount of time
        new RunCommand(
                () -> {
                  intake.setDesiredDeployed(true);
                },
                intake)
            .until(intake::atDesiredPosition),

        // run intake and move lift to intake position, until the robot sees an
        // object and the lift is in position
        new ParallelCommandGroup(

            // run intake
            new InstantCommand(intake::intakeGamePiece, intake),

            // trigger the lift to move to the intake position. This does not need a timeout because
            // it is running in the parallel group, which is controlled by readyToIntake()
            new InstantCommand(() -> lift.setDesiredPosition(LiftPosition.GRAB_FROM_INTAKE), lift)),

        // triggers the grabber to open when it is safe
        new InstantCommand(lift::openGrabber, lift),

        // wait until the lift is in position and the intake sees a game piece
        new WaitUntilCommand(
            () -> (lift.atPosition(Lift.LiftPosition.GRAB_FROM_INTAKE) && intake.seeGamePiece())),

        // stop intake
        new InstantCommand(intake::stopIntakingGamePiece, intake),

        // triggers the grabber to close
        new InstantCommand(lift::closeGrabber, lift),

        // wait until the grabber has closed
        new WaitCommand(Cal.Lift.GRABBER_CLOSE_TIME_SECONDS),

        // immediately unclamps the intake.
        // TODO this may not work
        new InstantCommand(intake::unclampIntake, intake),

        // give the intake time to unclamp
        new WaitCommand(Cal.Intake.UNCLAMP_TIME_SECONDS),

        // triggers the lift to move to the starting position. This does not need a timeout even
        // though it is a longer action, because it is the final action in the sequence
        new InstantCommand(() -> lift.setDesiredPosition(LiftPosition.STARTING), lift));
  }
}
