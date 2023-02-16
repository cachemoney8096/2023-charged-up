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
    addRequirements(lift);
    addCommands(
        new InstantCommand(lift::openClaw, lift),
        new WaitCommand(Cal.Lift.CLAW_OPEN_TIME_SECONDS),
        new InstantCommand(lift::closeClaw, lift),
        new ConditionalCommand(
            // TODO consider whether we even need POST_SCORE_HIGH
            new InstantCommand(() -> lift.setDesiredPosition(LiftPosition.POST_SCORE_HIGH), lift)
                .andThen(new WaitCommand(Cal.Lift.SAFE_TO_RETURN_TO_START_SECONDS)),
            new InstantCommand(() -> lift.setDesiredPosition(LiftPosition.STARTING)),
            lift.scoreLoc::isScoringHigh),
        new InstantCommand(() -> lift.setDesiredPosition(LiftPosition.STARTING)));
  }
}
