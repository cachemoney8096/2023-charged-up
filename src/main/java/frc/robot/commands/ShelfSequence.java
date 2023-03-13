package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.LiftPosition;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.LightCode;

/** moves lift to Shelf position, opens the grabber, waits, closes the grabber */
public class ShelfSequence extends SequentialCommandGroup {

  public ShelfSequence(Lift lift, Lights lights) {
    addRequirements(lift);
    addCommands(
        new InstantCommand(lift::openGrabber),
        new InstantCommand(
            () -> {
              lift.setDesiredPosition(LiftPosition.SHELF);
            }),
        new WaitUntilCommand(
            () -> {
              return lift.seeGamePiece();
            }),
        new InstantCommand(lift::closeGrabber),
        new InstantCommand(
            () -> {
              lights.setLight(LightCode.GAME_OBJECT);
            }),
        new WaitCommand(0.2),
        new InstantCommand(
            () -> {
              lift.setDesiredPosition(LiftPosition.STARTING);
            },
            lift));
  }
}
