package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Cal;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.LiftPosition;

public class finishScore extends SequentialCommandGroup {
  public finishScore(Lift lift) {
    /** if scoring action is cancelled, then don't run finish score and reset cancelScore */
    if (lift.getCancelScore()) {
      lift.setCancelScore(false);
      this.cancel();
    }
    addRequirements(lift);
    addCommands(
        new InstantCommand(lift::openGrabber, lift),
        new WaitCommand(Cal.Lift.GRABBER_OPEN_TIME_SECONDS),
        new InstantCommand(lift::closeGrabber, lift),
        new ConditionalCommand(
            // TODO consider whether we even need POST_SCORE_HIGH
            new InstantCommand(() -> lift.setDesiredPosition(LiftPosition.POST_SCORE_HIGH), lift)
                .andThen(new WaitCommand(Cal.Lift.SAFE_TO_RETURN_TO_START_SECONDS)),
            new InstantCommand(() -> lift.setDesiredPosition(LiftPosition.STARTING)),
            lift.scoreLoc::isScoringHigh));
    new InstantCommand(() -> lift.setDesiredPosition(LiftPosition.STARTING));
  }
}
