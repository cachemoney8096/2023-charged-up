package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.LiftPosition;

public class OuttakeSequence extends SequentialCommandGroup {
  private Intake intake;
  private Lift lift;
  private LiftPosition scoringPosition;

  public OuttakeSequence(Intake intake, Lift lift, LiftPosition scoringPosition) {
    this.intake = intake;
    this.lift = lift;
    this.scoringPosition = scoringPosition;
    addCommands(liftToOuttakePos, openGrabber, waitForGrabber, closeGrabber);
  }

  /**
   * trigger the lift to move to the desired scoring position (shelf, score_mid, score_high) ends
   * when the lift reaches the desired position
   */
  private ParallelRaceGroup liftToOuttakePos =
      new RunCommand(() -> lift.setDesiredPosition(scoringPosition), lift)
          .until(() -> lift.atPosition(scoringPosition));

  /** opens the grabber */
  private InstantCommand openGrabber = new InstantCommand(lift::openGrabber, lift);

  /** waits for the amount fo time for the grabber to open */
  private WaitCommand waitForGrabber = new WaitCommand(Constants.Lift.GRABBER_WAIT_TIME_SECONDS);

  /** closes the grabber */
  private InstantCommand closeGrabber = new InstantCommand(lift::closeGrabber, lift);
}
