package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Cal;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.LiftPosition;

/** moves lift to OUTTAKING position, opens the grabber, waits, closes the grabber */
public class OuttakeSequence extends SequentialCommandGroup {

  public OuttakeSequence(Lift lift) {
    addCommands(
        new RunCommand(() -> lift.setDesiredPosition(LiftPosition.OUTTAKING), lift)
            .until(() -> lift.atPosition(LiftPosition.OUTTAKING)),
        new InstantCommand(lift::openGrabber, lift),
        new WaitCommand(Cal.Lift.OUTTAKE_GRABBER_WAIT_TIME_SECONDS),
        new InstantCommand(lift::closeGrabber, lift));
  }
}
