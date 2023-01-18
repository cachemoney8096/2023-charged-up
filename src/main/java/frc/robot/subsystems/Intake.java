// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations;
import frc.robot.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The game piece intake.
 * 
 * Deploys out of the frame perimeter. Collect game pieces (cubes and cones)
 * from the ground and preps them to be grabbed by the lift.
 */
public class Intake extends SubsystemBase {

  private DoubleSolenoid deployLeft = new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM, RobotMap.INTAKE_DEPLOY_LEFT_FORWARD_CHANNEL,
      RobotMap.INTAKE_DEPLOY_LEFT_REVERSE_CHANNEL);
  private DoubleSolenoid deployRight = new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM, RobotMap.INTAKE_DEPLOY_RIGHT_FORWARD_CHANNEL,
      RobotMap.INTAKE_DEPLOY_RIGHT_REVERSE_CHANNEL);

  private CANSparkMax intakeLeft = new CANSparkMax(RobotMap.INTAKE_LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
  private CANSparkMax intakeRight = new CANSparkMax(RobotMap.INTAKE_RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);

  /** Creates a new Intake. */
  public Intake() {
    intakeLeft.restoreFactoryDefaults();

    intakeRight.restoreFactoryDefaults();
    intakeRight.follow(intakeLeft, true);
  }

  /** Deploys the intake out */
  public void deploy() {
    deployLeft.set(DoubleSolenoid.Value.kForward);
    deployRight.set(DoubleSolenoid.Value.kForward);
  }

  /** Brings the intake back in */
  public void retract() {
    deployLeft.set(DoubleSolenoid.Value.kReverse);
    deployRight.set(DoubleSolenoid.Value.kReverse);
  }

  /** Runs the intake wheels inward */
  public void intakeGamePiece() {
    intakeLeft.set(Calibrations.INTAKE_INTAKING_POWER);
  }

  /** Runs the intake wheels outward */
  public void ejectGamePiece() {
    intakeLeft.set(Calibrations.INTAKE_EJECTION_POWER);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
