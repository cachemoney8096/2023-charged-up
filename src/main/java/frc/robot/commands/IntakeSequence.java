package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;

public class IntakeSequence extends CommandBase {
  private Intake intake;
  private Lift lift;

  public IntakeSequence(Intake intake, Lift lift) {
    this.intake = intake;
    this.lift = lift;
  }

  @Override
  public void initialize() {
    intake.deploy();
    lift.setDesiredPosition(Lift.LiftPosition.GRAB_FROM_INTAKE);
  }

  @Override
  public void execute() {
    if (!lift.seeGamePiece()) {
        intake.intakeGamePiece();
    } else {
        lift.grab();
    }
  }

  @Override
  public boolean isFinished(){
    return lift.holdingGamePiece() ? true : false;
  }

  @Override
  public void end(boolean interrupted){
    if (!interrupted){
      lift.setDesiredPosition(Lift.LiftPosition.STARTING);
      if (lift.clearOfIntakeZone()){
        intake.retract();
      }
    }
  }
}