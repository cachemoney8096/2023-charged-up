package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Cal;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.LiftPosition;

public class finishScore extends SequentialCommandGroup {
  public finishScore(Lift lift){
    addRequirements(lift);
    addCommands(
      new InstantCommand(lift::openGrabber, lift),
      new WaitCommand(Cal.Lift.GRABBER_OPEN_TIME_SECONDS),
      (lift.isScoringHigh() ?
        new InstantCommand(() -> lift.setDesiredPosition(LiftPosition.POST_SCORE_HIGH), lift).andThen(
          new WaitUntilCommand(() -> lift.atPosition(LiftPosition.POST_SCORE_HIGH))) : 
        null),
      new InstantCommand(() -> lift.setDesiredPosition(LiftPosition.STARTING))
    );
  }
}