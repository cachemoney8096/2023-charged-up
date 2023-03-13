package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.LiftPosition;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.LightCode;

/** moves lift to OUTTAKING position, opens the grabber, waits, closes the grabber */
public class OuttakeSequence extends SequentialCommandGroup {

  public OuttakeSequence(Lift lift, Lights lights) {
    addRequirements(lift);
    addCommands(
        new RunCommand(() -> lift.setDesiredPosition(LiftPosition.OUTTAKING), lift)
            .until(() -> lift.atPosition(LiftPosition.OUTTAKING)),
        new InstantCommand(lift::openGrabber, lift),
        new InstantCommand(
            () -> {
              lights.setLight(LightCode.OFF);
            }));
  }
}
