package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Cal;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.LiftPosition;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.LightCode;

public class finishScore extends SequentialCommandGroup {
  public finishScore(Lift lift, Lights lights) {
    addRequirements(lift, lights);
    addCommands(
        new InstantCommand(() -> lift.setScoringInProgress(false)),
        new InstantCommand(lift::openGrabber, lift),
        new WaitCommand(Cal.Lift.GRABBER_OPEN_TIME_SECONDS),
        new InstantCommand(() -> lift.setDesiredPosition(LiftPosition.STARTING)),
        new InstantCommand(() -> lights.setLight(LightCode.OFF)));
  }
}
